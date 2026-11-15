import math

from config import GRAVITY_GAIN, INERTIA, MU


def controller(
    alpha: float,
    alpha_dot: float,
    reference: tuple[float, float, float],
    a: float,
    b: float,
    feedforward: bool = False,
    c: float = 0.0,
    integral: float = 0.0,
) -> float:
    alpha_d, alpha_dot_d, alpha_ddot_d = reference
    compensation = -(GRAVITY_GAIN * math.sin(alpha) - 2 * MU * alpha_dot)
    if feedforward:
        feedback = alpha_ddot_d + a * (alpha_dot - alpha_dot_d) + b * (alpha - alpha_d)
    else:
        feedback = a * alpha_dot + b * (alpha - alpha_d)
    return compensation + (feedback + c * integral) * INERTIA
