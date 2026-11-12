import numpy as np

from .params import PlantParams


def roll_acceleration(torque: float, alpha: float, alpha_dot: float, p: PlantParams) -> float:
    return (torque + p.gravity_gain * np.sin(alpha) - 2 * p.mu * alpha_dot) / p.inertia


def step(
    torque: float, alpha: float, alpha_dot: float, p: PlantParams, dt: float
) -> tuple[float, float, float]:
    alpha_ddot = roll_acceleration(torque, alpha, alpha_dot, p)
    alpha_dot_new = alpha_dot + alpha_ddot * dt
    alpha_new = alpha + alpha_dot * dt + alpha_ddot * dt**2 / 2
    return alpha_new, alpha_dot_new, alpha_ddot


def inverse_linear_model(alpha_dot: float, p: PlantParams) -> float:
    return 2 * p.mu * alpha_dot
