import numpy as np
from dataclasses import dataclass
from utils.writer import load_gains

NOMINAL_WHEEL_RADIUS = 0.0318
NOMINAL_BASELINE = 0.1
NOMINAL_ENCODER_TICKS = 135
_ENCODER_RESOLUTION_RAD = 2 * np.pi / NOMINAL_ENCODER_TICKS
_MOTOR_CONSTANT = 27.0


# --- SE2 / se2 helpers (replaces PyGeometry-z6) ---

def _SE2_from_xytheta(xytheta):
    x, y, th = xytheta[0], xytheta[1], xytheta[2]
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1]], dtype=float)


def _xytheta_from_SE2(m):
    return np.array([m[0, 2], m[1, 2], np.arctan2(m[1, 0], m[0, 0])])


def _se2_from_linear_angular(linear, omega):
    vx, vy = float(linear[0]), float(linear[1])
    return np.array([[0, -omega, vx], [omega, 0, vy], [0, 0, 0]], dtype=float)


def _linear_angular_from_se2(m):
    return np.sqrt(m[0, 2] ** 2 + m[1, 2] ** 2), m[1, 0]


# --- Simplified DB18 dynamics (replaces duckietown_world) ---

@dataclass
class PWMCommands:
    motor_left: float
    motor_right: float


class _DynamicsState:
    def __init__(self, pose_se2, vel_se2):
        self._pose = pose_se2.copy()
        self._vel = vel_se2.copy()
        self.axis_left_obs_rad = 0.0
        self.axis_right_obs_rad = 0.0

        class _Params:
            encoder_resolution_rad = _ENCODER_RESOLUTION_RAD
        self.parameters = _Params()

    def integrate(self, dt, commands: PWMCommands):
        omega_l = commands.motor_left * _MOTOR_CONSTANT
        omega_r = commands.motor_right * _MOTOR_CONSTANT
        v_l = omega_l * NOMINAL_WHEEL_RADIUS
        v_r = omega_r * NOMINAL_WHEEL_RADIUS
        v = (v_l + v_r) / 2.0
        omega = (v_r - v_l) / NOMINAL_BASELINE

        x, y, th = _xytheta_from_SE2(self._pose)
        new_th = th + omega * dt
        new_x = x + v * np.cos(th) * dt
        new_y = y + v * np.sin(th) * dt

        new_state = _DynamicsState(
            _SE2_from_xytheta([new_x, new_y, new_th]),
            _se2_from_linear_angular(
                [v * np.cos(new_th), v * np.sin(new_th)], omega
            ),
        )
        new_state.axis_left_obs_rad = self.axis_left_obs_rad + omega_l * dt
        new_state.axis_right_obs_rad = self.axis_right_obs_rad + omega_r * dt
        return new_state

    def TSE2_from_state(self):
        return self._pose.copy(), self._vel.copy()


class _DB18Model:
    def initialize(self, c0, t0=0.0):
        return _DynamicsState(c0[0], c0[1])


def get_DB18_nominal(delay=0):
    return _DB18Model()


# --- Public helpers ---

def get_wheel_speed(omega, v_a, baseline=NOMINAL_BASELINE, radius=NOMINAL_WHEEL_RADIUS):
    omega_l = (v_a - 0.5 * omega * baseline) / radius
    omega_r = (v_a + 0.5 * omega * baseline) / radius
    return omega_l, omega_r


def pwm_commands_from_PID(omega, v_a, k_r=27, k_l=27, limit=1.0):
    omega_l, omega_r = get_wheel_speed(omega, v_a)
    u_l = np.clip(omega_l / k_l, -limit, limit)
    u_r = np.clip(omega_r / k_r, -limit, limit)
    return PWMCommands(motor_left=u_l, motor_right=u_r)


def get_measured_ticks(model: _DynamicsState):
    ticks_left = model.axis_left_obs_rad / model.parameters.encoder_resolution_rad
    ticks_right = model.axis_right_obs_rad / model.parameters.encoder_resolution_rad
    return ticks_left, ticks_right


def integrate_dynamics(
    initial_pose, initial_vel, y_ref, controller, odometry_function=None, delta_phi=None
):
    initial_pose[2] = np.deg2rad(initial_pose[2])
    v = initial_vel[0]
    omega = initial_vel[1]

    PIDcontroller = controller()
    kp, kd, ki = load_gains(filepath="../packages/solution/OFFSET_GAINS.yaml")
    PIDcontroller.SetGains(kp=float(kp), ki=float(ki), kd=float(kd))

    last_pose = _SE2_from_xytheta(initial_pose)
    last_vel = _se2_from_linear_angular(
        np.array([v * np.cos(initial_pose[2]), v * np.sin(initial_pose[2])]), omega
    )

    initial_time = 0.0
    timestep = 0.1
    t_max = 60
    n = int(t_max / timestep)

    nominal_duckie = get_DB18_nominal(delay=0)
    state = nominal_duckie.initialize(c0=(last_pose, last_vel), t0=initial_time)

    e = 0
    v_0 = 0.22

    pose_list = [last_pose]
    vel_list = [last_vel]
    e_list = [e]

    if odometry_function is not None:
        x_hat, y_hat, theta_hat = initial_pose[0:3]
        prev_ticks_left = prev_ticks_right = 0
        ticks_left = ticks_right = 0

    for _ in range(n):
        if odometry_function is None:
            y_hat = _xytheta_from_SE2(last_pose)[1]
        else:
            assert delta_phi is not None, "Need to pass a delta_phi function!"
            prev_ticks_left = ticks_left
            prev_ticks_right = ticks_right
            ticks_left, ticks_right = get_measured_ticks(state)
            delta_phi_left = delta_phi(ticks_left, prev_ticks_left, NOMINAL_ENCODER_TICKS)
            delta_phi_right = delta_phi(ticks_right, prev_ticks_right, NOMINAL_ENCODER_TICKS)
            x_hat, y_hat, theta_hat = odometry_function(
                R=NOMINAL_WHEEL_RADIUS,
                baseline=NOMINAL_BASELINE,
                x_prev=x_hat,
                y_prev=y_hat,
                theta_prev=theta_hat,
                delta_phi_left=delta_phi_left,
                delta_phi_right=delta_phi_right,
            )

        v_0, omega = PIDcontroller.OffsetControl(v_0, y_ref, y_hat, delta_t=timestep)
        e = y_ref - y_hat

        commands = pwm_commands_from_PID(omega, v_0)
        state = state.integrate(timestep, commands)
        last_pose, last_vel = state.TSE2_from_state()
        pose_list.append(last_pose)
        vel_list.append(last_vel)
        e_list.append(e)

    xs, ys, angles, omegas = [], [], [], []
    for pose_SE2 in pose_list:
        x, y, theta = _xytheta_from_SE2(pose_SE2)
        xs.append(x)
        ys.append(y)
        angles.append(np.rad2deg(theta))
    for v_se2 in vel_list:
        _, omega = _linear_angular_from_se2(v_se2)
        omegas.append(omega)

    return xs, ys, omegas, e_list, angles
