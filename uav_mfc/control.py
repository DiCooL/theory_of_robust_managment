import numpy as np

from .params import ControllerParams, PlantParams


def program_control(
    alpha: float, alpha_dot: float, alpha_d: float, p: PlantParams, c: ControllerParams
) -> float:
    compensation = -(p.gravity_gain * np.sin(alpha) - 2 * p.mu * alpha_dot)
    feedback = (c.a * alpha_dot + c.b * (alpha - alpha_d)) * p.inertia
    return compensation + feedback


class CorrectionFilter:
    def __init__(self, T: float, dt: float):
        self.T = T
        self.dt = dt
        self.y = 0.0

    def reset(self) -> None:
        self.y = 0.0

    def update(self, u: float) -> float:
        self.y = self.y + self.dt * self.T * (u - self.y)
        return self.y
