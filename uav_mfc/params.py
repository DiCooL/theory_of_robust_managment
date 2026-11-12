from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class PlantParams:
    inertia: float = 0.02
    gravity_gain: float = 0.3
    mu: float = 2.5e-3


@dataclass(frozen=True)
class ControllerParams:
    a: float = -202.0
    b: float = -10000.0

    @classmethod
    def from_natural(cls, omega_n: float, zeta: float) -> "ControllerParams":
        return cls(a=-2.0 * zeta * omega_n, b=-omega_n**2)

    def is_aperiodic(self) -> bool:
        return self.a < 0 and -(self.a**2) / 4 < self.b < 0


@dataclass(frozen=True)
class FilterParams:
    T: float = 15.0


@dataclass(frozen=True)
class NoiseParams:
    angle_std: float = 0.0
    rate_std: float = 0.0
    torque_std: float = 0.0
    mode: str = "state"

    @property
    def enabled(self) -> bool:
        return self.angle_std > 0 or self.rate_std > 0 or self.torque_std > 0


NO_NOISE = NoiseParams()
THESIS_NOISE = NoiseParams(np.deg2rad(0.3), np.deg2rad(0.4), 0.004)
STRONG_NOISE = NoiseParams(np.deg2rad(2.6), np.deg2rad(2.8), 0.04)


@dataclass(frozen=True)
class SimConfig:
    dt: float = 1e-3
    t_final: float = 20.0
    inner_loop: str = "mfc"
    seed: int | None = 2


INNER_LOOPS = ("none", "inverse_model", "mfc")
NOISE_MODES = ("state", "measurement")
