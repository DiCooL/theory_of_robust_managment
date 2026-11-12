from dataclasses import dataclass, field
from typing import Callable

import numpy as np

from .control import CorrectionFilter, program_control
from .params import (
    INNER_LOOPS,
    NOISE_MODES,
    NO_NOISE,
    ControllerParams,
    FilterParams,
    NoiseParams,
    PlantParams,
    SimConfig,
)
from .plant import inverse_linear_model, step
from .trajectory import Sinusoid

Trajectory = Callable[[float], tuple[float, float, float]]


@dataclass
class SimResult:
    time: np.ndarray
    alpha: np.ndarray
    alpha_dot: np.ndarray
    alpha_ddot: np.ndarray
    alpha_d: np.ndarray
    alpha_dot_d: np.ndarray
    alpha_ddot_d: np.ndarray
    torque: np.ndarray
    u_bar: np.ndarray
    u_delta: np.ndarray
    meta: dict = field(default_factory=dict)

    @property
    def error(self) -> np.ndarray:
        return self.alpha - self.alpha_d

    @property
    def rmse(self) -> float:
        return float(np.sqrt(np.mean(self.error**2)))

    @property
    def rmse_deg(self) -> float:
        return float(np.rad2deg(self.rmse))

    @property
    def max_error_deg(self) -> float:
        return float(np.rad2deg(np.max(np.abs(self.error))))

    @property
    def mean_abs_torque(self) -> float:
        return float(np.mean(np.abs(self.torque)))


def simulate(
    trajectory: Trajectory | None = None,
    plant: PlantParams = PlantParams(),
    controller: ControllerParams = ControllerParams(),
    filt: FilterParams = FilterParams(),
    noise: NoiseParams = NO_NOISE,
    config: SimConfig = SimConfig(),
    model: PlantParams | None = None,
) -> SimResult:
    if config.inner_loop not in INNER_LOOPS:
        raise ValueError(f"inner_loop must be one of {INNER_LOOPS}")
    if noise.mode not in NOISE_MODES:
        raise ValueError(f"noise.mode must be one of {NOISE_MODES}")

    trajectory = trajectory or Sinusoid()
    model = model or plant
    rng = np.random.default_rng(config.seed)
    dt = config.dt
    n_max = int(round(config.t_final / dt)) + 1

    rec = {key: np.zeros(n_max) for key in (
        "time", "alpha", "alpha_dot", "alpha_ddot", "alpha_d", "alpha_dot_d",
        "alpha_ddot_d", "torque", "u_bar", "u_delta",
    )}

    correction = CorrectionFilter(filt.T, dt)
    alpha, alpha_dot, _ = trajectory(0.0)
    u_delta = 0.0
    t = 0.0
    k = 0

    while t < config.t_final:
        alpha_d, alpha_dot_d, alpha_ddot_d = trajectory(t)

        if noise.enabled:
            alpha_meas = alpha + noise.angle_std * rng.standard_normal()
            alpha_dot_meas = alpha_dot + noise.rate_std * rng.standard_normal()
        else:
            alpha_meas, alpha_dot_meas = alpha, alpha_dot

        if noise.mode == "state":
            alpha, alpha_dot = alpha_meas, alpha_dot_meas

        u_bar = program_control(alpha_meas, alpha_dot_meas, alpha_d, model, controller)
        torque = u_bar - u_delta if config.inner_loop != "none" else u_bar
        if noise.enabled:
            torque += noise.torque_std * rng.standard_normal()

        alpha_new, alpha_dot_new, alpha_ddot_new = step(torque, alpha, alpha_dot, plant, dt)

        for key, value in (
            ("time", t), ("alpha", alpha_new), ("alpha_dot", alpha_dot_new),
            ("alpha_ddot", alpha_ddot_new), ("alpha_d", alpha_d),
            ("alpha_dot_d", alpha_dot_d), ("alpha_ddot_d", alpha_ddot_d),
            ("torque", torque), ("u_bar", u_bar), ("u_delta", u_delta),
        ):
            rec[key][k] = value

        alpha, alpha_dot = alpha_new, alpha_dot_new

        if config.inner_loop == "inverse_model":
            u_delta = inverse_linear_model(alpha_dot, model)
        elif config.inner_loop == "mfc":
            u_delta = inverse_linear_model(alpha_dot, model) - correction.update(torque)

        t += dt
        k += 1

    data = {key: value[:k] for key, value in rec.items()}
    meta = {
        "plant": plant, "model": model, "controller": controller, "filter": filt,
        "noise": noise, "config": config, "trajectory": trajectory,
    }
    return SimResult(**data, meta=meta)
