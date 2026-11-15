import math

from config import GRAVITY_GAIN, INERTIA, MU


def uav_model(torque: float, alpha: float, alpha_dot: float, dt: float) -> tuple[float, float, float]:
    alpha_ddot = (torque + GRAVITY_GAIN * math.sin(alpha) - 2 * MU * alpha_dot) / INERTIA
    alpha_dot_new = alpha_dot + alpha_ddot * dt
    alpha_new = alpha + alpha_dot * dt + alpha_ddot * dt**2 / 2
    return alpha_new, alpha_dot_new, alpha_ddot
