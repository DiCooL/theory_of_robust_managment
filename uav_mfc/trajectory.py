from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Sinusoid:
    amplitude: float = np.pi / 6
    bias: float = np.pi / 6
    omega: float = np.pi / 10
    phase: float = -np.pi / 2

    def __call__(self, t: float) -> tuple[float, float, float]:
        arg = self.omega * t + self.phase
        alpha_d = self.amplitude * np.sin(arg) + self.bias
        alpha_dot_d = self.amplitude * self.omega * np.cos(arg)
        alpha_ddot_d = -self.amplitude * self.omega**2 * np.sin(arg)
        return alpha_d, alpha_dot_d, alpha_ddot_d


@dataclass(frozen=True)
class Arctan:
    amplitude: float = np.pi / 6
    bias: float = 0.0
    k: float = 0.8

    def __call__(self, t: float) -> tuple[float, float, float]:
        kt = self.k * t
        alpha_d = self.amplitude * np.arctan(kt) + self.bias
        alpha_dot_d = self.amplitude * self.k / (1 + kt**2)
        alpha_ddot_d = -2 * self.amplitude * self.k**3 * t / (1 + kt**2) ** 2
        return alpha_d, alpha_dot_d, alpha_ddot_d


@dataclass(frozen=True)
class Constant:
    value: float = 0.0

    def __call__(self, t: float) -> tuple[float, float, float]:
        return self.value, 0.0, 0.0


TRAJECTORIES = {
    "sin": Sinusoid,
    "atan": Arctan,
    "const": Constant,
}
