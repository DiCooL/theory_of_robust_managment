from dataclasses import dataclass, replace
from itertools import product
from typing import Iterable

import numpy as np

from .params import NO_NOISE, ControllerParams, FilterParams, NoiseParams, PlantParams, SimConfig
from .simulation import Trajectory, simulate


@dataclass(frozen=True)
class TuningPoint:
    omega_n: float
    zeta: float
    a: float
    b: float
    J: float


def grid_search(
    omega_values: Iterable[float],
    zeta_values: Iterable[float],
    trajectory: Trajectory | None = None,
    plant: PlantParams = PlantParams(),
    filt: FilterParams = FilterParams(),
    noise: NoiseParams = NO_NOISE,
    config: SimConfig = SimConfig(t_final=10.0),
) -> list[TuningPoint]:
    points = []
    for omega_n, zeta in product(omega_values, zeta_values):
        c = ControllerParams.from_natural(omega_n, zeta)
        if not c.is_aperiodic():
            continue
        res = simulate(trajectory, plant, c, filt, noise, replace(config))
        J = float(np.mean(res.error**2)) if np.all(np.isfinite(res.error)) else np.inf
        points.append(TuningPoint(float(omega_n), float(zeta), float(c.a), float(c.b), J))
    return sorted(points, key=lambda p: p.J)
