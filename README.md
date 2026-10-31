# LX Recipe: Control (PID)

Recipe of the [`lx-control`](https://github.com/duckietown/lx-control) learning experience
(branch `ente-ros2`): the Duckiebot runs **ROS 2** (Jazzy, `rmw_zenoh_cpp`) and the student
drives the exercise from the notebooks of the code editor (parameter panels, start/stop buttons, live plots).

## What the recipe provides

| Path | Content |
| --- | --- |
| `Dockerfile` | workbench image (base `dt-ros2-commons`), builds every ament package under `packages/` with colcon |
| `Dockerfile.vscode` | code editor image with ROS 2, `rclpy` and the `duckietown_msgs` so notebooks talk to the robot |
| `launchers/default.sh` | `ros2 launch pid_controller pid_controller_node.launch.py veh:=ROBOT car_cmd_topic:=...` (the LX launch file also starts `encoder_pose_node`; `CAR_CMD_TOPIC=car_cmd` for robot images whose `kinematics_node` still reads `car_cmd`) |
| `packages/lx_ros2_tools` | shared helpers: `LiveParams`, `get_wheel_params`, `RosSession` and the notebook panels (`ParamPanel`, `ActionButtons`, `TrajectoryPlot`, ...) |
| `packages/encoder_pose` | ament_python package with `encoder_pose_node` (dead reckoning from the wheel encoders) |
| `packages/odometry` | plain-python `delta_phi` / `estimate_pose` used by the node and importable from the notebooks |
| `packages/simulators` | plain-python kinematic / dynamic simulators used by the tuning notebook |
| `assets/duckiematrix` | the `straight` map used by `dts code start_matrix` |

### `encoder_pose_node` (namespace `/ROBOT`)

- subscribes `left_wheel_encoder_driver_node/tick`, `right_wheel_encoder_driver_node/tick` (`duckietown_msgs/WheelEncoderStamped`, sensor-data QoS)
- publishes `encoder_pose_node/pose` (`nav_msgs/Odometry`, frame `map`) at `rate` Hz (parameter, default 2.0)
- service `encoder_pose_node/reset` (`std_srvs/Trigger`) zeroes the estimate; the PID controller calls it on `stop`
- wheel radius / baseline come from the robot calibration file through `lx_ros2_tools.robot.get_wheel_params`

The student node (`pid_controller_node`, in the LX) subscribes to the pose, exposes the gains and
references as ROS 2 parameters and `start` / `stop` services, and publishes `duckietown_msgs/Twist2DStamped`
on `car_cmd_switch_node/cmd`.

## Dependencies

`apt` and `pip` dependencies of the workbench image go in [dependencies-apt.txt](./dependencies-apt.txt)
and [dependencies-py3.txt](./dependencies-py3.txt). The editor image installs ROS 2 from
[dependencies-apt.vscode.txt](./dependencies-apt.vscode.txt) and the same pip list
(`ipywidgets`, `ipympl`, `matplotlib` are required by the notebook panels).

## Duckiematrix

Each LX is compatible with the [Duckiematrix](https://docs.duckietown.com/ente/devmanual-duckiematrix/intro.html).
The exact configuration of the environment lives in [assets/duckiematrix](./assets/duckiematrix)
(`settings.yaml` selects the map and the vehicle).

## Testing the recipe locally

```
docker buildx build --load -f Dockerfile --build-context recipe=. --build-context meat=../lx-control \
  --build-arg ARCH=arm64v8 --build-arg DISTRO=ente -t lxros2/control:test .
docker run --rm -e VEHICLE_NAME=testbot lxros2/control:test bash -c \
  "source /environment.sh; timeout 15 ros2 launch pid_controller pid_controller_node.launch.py veh:=testbot"
```
