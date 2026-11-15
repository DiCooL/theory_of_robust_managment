from dataclasses import dataclass, replace
import math

INERTIA = 0.02
GRAVITY_GAIN = 0.3
MU = 2.5e-3

DT = 1e-3
T_FINAL = 20.0
SEED = 2

THESIS_OMEGA_N = 100.0
THESIS_ZETA = 1.01

OMEGA_N = 15.0
ZETA = 2.0
FILTER_T = 15.0
INTEGRAL_POLE = 2.0
AW_TRACKING_TIME = 0.05

TRAJ_AMPLITUDE = math.pi / 6
TRAJ_BIAS = math.pi / 6
TRAJ_OMEGA = math.pi / 10
TRAJ_PHASE = -math.pi / 2

CAMERA_RMSE_DEG = 0.45
RATE_NOISE_FRACTION = 0.01
RATE_SPAN = 2 * TRAJ_AMPLITUDE * TRAJ_OMEGA

ANGLE_NOISE_STD = math.radians(CAMERA_RMSE_DEG)
RATE_NOISE_STD = RATE_NOISE_FRACTION * RATE_SPAN
DISTURBANCE_STD = 0.004

ARM = 0.23
THRUST_AT_PWM_1600 = 5.5
THRUST_AT_PWM_1150 = 0.6
TORQUE_LIMIT = ARM * (THRUST_AT_PWM_1600 - THRUST_AT_PWM_1150)
DEADZONE_MAX = ARM * THRUST_AT_PWM_1150
DEADZONE = 0.05
DEADZONE_INVERSE_BAND = 0.02


@dataclass(frozen=True)
class Noise:
    angle_std: float = 0.0
    rate_std: float = 0.0
    disturbance_std: float = 0.0


@dataclass(frozen=True)
class Stand:
    torque_limit: float | None = None
    deadzone: tuple[float, float] = (0.0, 0.0)
    control_period: float = DT
    camera_period: float = DT
    camera_delay: float = 0.0
    gyro_delay: float = 0.0
    motor_tau: float = 0.0


NO_NOISE = Noise()
SENSOR_NOISE = Noise(ANGLE_NOISE_STD, RATE_NOISE_STD, DISTURBANCE_STD)
THESIS_NOISE = Noise(math.radians(0.3), math.radians(0.4), 0.004)

IDEAL_STAND = Stand()
SATURATED_STAND = Stand(torque_limit=TORQUE_LIMIT)
REAL_STAND = Stand(
    torque_limit=TORQUE_LIMIT,
    control_period=0.02,
    camera_period=0.033,
    camera_delay=0.05,
    gyro_delay=0.004,
    motor_tau=0.05,
)
DEADZONE_STAND = replace(REAL_STAND, deadzone=(-DEADZONE, DEADZONE))


def gains(omega_n: float = OMEGA_N, zeta: float = ZETA) -> tuple[float, float]:
    return -2.0 * zeta * omega_n, -omega_n**2


def integral_gain(omega_n: float = OMEGA_N, pole: float = INTEGRAL_POLE) -> float:
    return -pole * omega_n**2
