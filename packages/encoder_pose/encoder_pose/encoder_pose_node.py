#!/usr/bin/env python3
"""Estimate of the Duckiebot pose (dead reckoning) from the wheel encoders.

Subscribes to the wheel encoder ticks, integrates the kinematic model (see ``odometry.estimate_pose``)
and publishes ``nav_msgs/Odometry`` on ``~/pose``. The estimate can be zeroed with the ``~/reset``
service (the PID controller node calls it when it stops).
"""
from typing import Optional

import numpy as np
import rclpy
from duckietown_msgs.msg import WheelEncoderStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger

from lx_ros2_tools.params import LiveParams
from lx_ros2_tools.robot import get_wheel_params
from odometry import delta_phi, estimate_pose


class EncoderPoseNode(Node):
    """
    Computes an estimate of the Duckiebot pose using the wheel encoders.

    Parameters:
        rate (float): rate [Hz] at which the pose is published (default 2.0)
    Publisher:
        ~/pose (nav_msgs/Odometry): the estimated pose (frame ``map``)
    Subscribers:
        left_wheel_encoder_driver_node/tick (duckietown_msgs/WheelEncoderStamped)
        right_wheel_encoder_driver_node/tick (duckietown_msgs/WheelEncoderStamped)
    Services:
        ~/reset (std_srvs/Trigger): zero the pose estimate
    """

    right_tick_prev: Optional[int]
    left_tick_prev: Optional[int]
    delta_phi_left: float
    delta_phi_right: float

    def __init__(self):
        super().__init__("encoder_pose_node")
        self.get_logger().info("Initializing...")
        # get the name of the robot
        self.veh = self.get_namespace().strip("/")

        # parameters (defaults also in config/encoder_pose_node/default.yaml)
        self.params = LiveParams(self, {"rate": 2.0})

        # Init the state
        self.resetParameters()

        # nominal R and L of this robot (calibration file, or DB21 defaults)
        wheels = get_wheel_params(self.veh)
        self.R = wheels.radius  # meters, wheel radius
        self.baseline = wheels.baseline  # meters, distance between the wheels
        self.get_logger().info(f"wheel radius {self.R} m, baseline {self.baseline} m ({wheels.source})")

        # Wheel encoder subscribers (relative names: the launch file puts the node in the robot namespace)
        self.create_subscription(WheelEncoderStamped, "left_wheel_encoder_driver_node/tick", self.cbLeftEncoder,
                                 qos_profile_sensor_data)
        self.create_subscription(WheelEncoderStamped, "right_wheel_encoder_driver_node/tick", self.cbRightEncoder,
                                 qos_profile_sensor_data)

        # Odometry publisher
        self.db_estimated_pose = self.create_publisher(Odometry, "~/pose", 10)

        # reset service (called by the controller on STOP, or from the notebook)
        self.create_service(Trigger, "~/reset", self.srv_reset)

        self.create_timer(1.0 / max(float(self.params.rate), 0.1), self.posePublisher)
        self.get_logger().info("Initialized.")

    def resetParameters(self):
        self.get_logger().info("Encoder pose resetting")
        self.delta_phi_left = 0.0
        self.left_tick_prev = None

        self.delta_phi_right = 0.0
        self.right_tick_prev = None

        # Initializing the odometry
        self.x_prev = 0.0
        self.y_prev = 0.0
        self.theta_prev = 0.0

    def srv_reset(self, _req, res):
        """Every time we STOP (or a new activity starts) the odometry is reset."""
        self.resetParameters()
        res.success, res.message = True, "pose reset to (0, 0, 0)"
        return res

    def cbLeftEncoder(self, encoder_msg: WheelEncoderStamped):
        """
        Wheel encoder callback
        Args:
            encoder_msg (:obj:`WheelEncoderStamped`) encoder ROS message.
        """
        # initializing ticks to stored absolute value
        if self.left_tick_prev is None:
            self.left_tick_prev = encoder_msg.data
            return

        left_ticks_curr = encoder_msg.data
        # running the delta_phi() function from the notebooks to calculate rotations
        dphi = delta_phi(left_ticks_curr, self.left_tick_prev, encoder_msg.resolution)
        if dphi == 0:
            return
        self.left_tick_prev = left_ticks_curr
        self.delta_phi_left += dphi

    def cbRightEncoder(self, encoder_msg: WheelEncoderStamped):
        """
        Wheel encoder callback, the rotation of the wheel.
        Args:
            encoder_msg (:obj:`WheelEncoderStamped`) encoder ROS message.
        """
        if self.right_tick_prev is None:
            self.right_tick_prev = encoder_msg.data
            return

        right_ticks_curr = encoder_msg.data
        # calculate rotation of right wheel
        dphi = delta_phi(right_ticks_curr, self.right_tick_prev, encoder_msg.resolution)
        if dphi == 0:
            return
        self.right_tick_prev = right_ticks_curr
        self.delta_phi_right += dphi

    def posePublisher(self):
        """
        Publish the pose of the Duckiebot given by the kinematic model using the encoders.
        Publish:
            ~/pose (:obj:`Odometry`): Duckiebot pose.
        """
        x_curr, y_curr, theta_curr = estimate_pose(
            self.R,
            self.baseline,
            self.x_prev,
            self.y_prev,
            self.theta_prev,
            self.delta_phi_left,
            self.delta_phi_right,
        )

        theta_curr = self.angle_clamp(theta_curr)  # angle always between -2pi, 2pi

        # Calculate new odometry only when new data from encoders arrives
        self.delta_phi_left = 0.0
        self.delta_phi_right = 0.0

        # Current estimate becomes previous estimate at next iteration
        self.x_prev = x_curr
        self.y_prev = y_curr
        self.theta_prev = theta_curr

        # Creating message to plot the pose (TrajectoryPlot in the notebooks)
        odom = Odometry()
        odom.header.frame_id = "map"
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.child_frame_id = f"{self.veh}/base"

        odom.pose.pose.position.x = float(x_curr)  # x position - estimate
        odom.pose.pose.position.y = float(y_curr)  # y position - estimate
        odom.pose.pose.position.z = 0.0  # z position - no flying allowed in Duckietown

        # these are quaternions! (yaw -> quaternion)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = float(np.sin(theta_curr / 2))
        odom.pose.pose.orientation.w = float(np.cos(theta_curr / 2))

        self.db_estimated_pose.publish(odom)

    @staticmethod
    def angle_clamp(theta):
        if theta > 2 * np.pi:
            return theta - 2 * np.pi
        elif theta < -2 * np.pi:
            return theta + 2 * np.pi
        else:
            return theta


def main(args=None):
    rclpy.init(args=args)
    node = EncoderPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
