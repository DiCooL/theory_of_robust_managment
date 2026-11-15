import math

import numpy as np

from config import (
    AW_TRACKING_TIME,
    DEADZONE_INVERSE_BAND,
    DT,
    FILTER_T,
    IDEAL_STAND,
    INERTIA,
    NO_NOISE,
    OMEGA_N,
    SEED,
    T_FINAL,
    ZETA,
    Noise,
    Stand,
    gains,
)
from control.controller import controller
from control.function_correction import FunctionCorrection
from models.actuator import actuator, dead_zone_inverse, saturation_excess
from models.inverse_linear_model import inverse_linear_model
from models.uav_dynamics import uav_model
from trajectory.desired_trajectory import desired_trajectory

INNER_LOOPS = ("none", "inverse_model", "mfc")
DIVERGENCE_ANGLE = 4 * math.pi


def simulate(
    omega_n: float = OMEGA_N,
    zeta: float = ZETA,
    inner_loop: str = "none",
    noise: Noise = NO_NOISE,
    stand: Stand = IDEAL_STAND,
    feedforward: bool = True,
    filter_T: float = FILTER_T,
    t_final: float = T_FINAL,
    dt: float = DT,
    seed: int | None = SEED,
    c: float = 0.0,
    anti_windup: bool = True,
    aw_tracking_time: float = AW_TRACKING_TIME,
    deadzone_inverse: bool = False,
    deadzone_band: float = DEADZONE_INVERSE_BAND,
    deadzone_estimate: tuple[float, float] | None = None,
    pulse: tuple[float, float, float] | None = None,
) -> dict:
    if inner_loop not in INNER_LOOPS:
        raise ValueError(f"inner_loop must be one of {INNER_LOOPS}")

    a, b = gains(omega_n, zeta)
    u_minus, u_plus = stand.deadzone
    est_minus, est_plus = stand.deadzone if deadzone_estimate is None else deadzone_estimate
    rng = np.random.default_rng(seed)
    n_steps = int(round(t_final / dt))
    n_ctrl = max(1, int(round(stand.control_period / dt)))
    n_cam = max(1, int(round(stand.camera_period / dt)))
    n_cam_delay = int(round(stand.camera_delay / dt))
    n_gyro_delay = int(round(stand.gyro_delay / dt))
    ctrl_dt = n_ctrl * dt

    keys = ("time", "alpha", "alpha_dot", "alpha_ddot", "alpha_d", "alpha_dot_d", "alpha_ddot_d",
            "u", "torque_cmd", "torque_applied", "torque", "u_bar", "u_delta", "integral")
    rec = {key: np.full(n_steps, np.nan) for key in keys}

    correction = FunctionCorrection(filter_T, ctrl_dt)
    alpha, alpha_dot, _ = desired_trajectory(0.0)
    alpha_meas = alpha
    frame = -1
    u_bar = u = u_delta = 0.0
    torque_cmd = torque_applied = 0.0
    torque_act = None
    integral = 0.0
    saturated = 0
    diverged = False

    for k in range(n_steps):
        t = k * dt
        reference = desired_trajectory(t)
        rec["alpha"][k] = alpha
        rec["alpha_dot"][k] = alpha_dot

        if k % n_ctrl == 0:
            new_frame = max(k - n_cam_delay, 0) // n_cam
            if new_frame != frame:
                frame = new_frame
                alpha_meas = rec["alpha"][frame * n_cam] + noise.angle_std * rng.standard_normal()
            alpha_dot_meas = rec["alpha_dot"][max(k - n_gyro_delay, 0)] + noise.rate_std * rng.standard_normal()

            if k == 0 or inner_loop == "none":
                u_delta = 0.0
            elif inner_loop == "inverse_model":
                u_delta = inverse_linear_model(alpha_dot_meas)
            else:
                u_delta = inverse_linear_model(alpha_dot_meas) - correction.y

            u_bar = controller(alpha_meas, alpha_dot_meas, reference, a, b, feedforward, c, integral)
            u = u_bar - u_delta
            torque_cmd = dead_zone_inverse(u, est_minus, est_plus, deadzone_band) if deadzone_inverse else u
            torque_applied = actuator(torque_cmd, u_minus, u_plus, stand.torque_limit)
            excess = saturation_excess(torque_cmd, u_minus, u_plus, stand.torque_limit)
            if excess != 0.0:
                saturated += 1

            u_states = torque_applied if anti_windup else u
            correction.update(u_states)
            if c != 0.0:
                windup = excess / (INERTIA * c * aw_tracking_time) if anti_windup else 0.0
                integral += ctrl_dt * (alpha_meas - reference[0] + windup)

        if stand.motor_tau > 0:
            if torque_act is None:
                torque_act = torque_applied
            torque_act += dt / stand.motor_tau * (torque_applied - torque_act)
        else:
            torque_act = torque_applied

        torque = torque_act + noise.disturbance_std * rng.standard_normal()
        if pulse is not None and pulse[0] <= t < pulse[0] + pulse[1]:
            torque += pulse[2]
        alpha_new, alpha_dot_new, alpha_ddot = uav_model(torque, alpha, alpha_dot, dt)

        rec["time"][k] = t
        rec["alpha_ddot"][k] = alpha_ddot
        rec["alpha_d"][k], rec["alpha_dot_d"][k], rec["alpha_ddot_d"][k] = reference
        rec["u"][k] = u
        rec["torque_cmd"][k] = torque_cmd
        rec["torque_applied"][k] = torque_applied
        rec["torque"][k] = torque
        rec["u_bar"][k] = u_bar
        rec["u_delta"][k] = u_delta
        rec["integral"][k] = integral

        alpha, alpha_dot = alpha_new, alpha_dot_new
        if not math.isfinite(alpha) or abs(alpha) > DIVERGENCE_ANGLE:
            diverged = True
            break

    error = rec["alpha"] - rec["alpha_d"]
    rmse = math.inf if diverged else float(np.sqrt(np.mean(error**2)))
    n_ticks = math.ceil(n_steps / n_ctrl)
    rec.update(
        error=error,
        rmse=rmse,
        rmse_deg=math.degrees(rmse),
        max_error_deg=math.inf if diverged else float(np.degrees(np.max(np.abs(error)))),
        torque_rms=float(np.sqrt(np.nanmean(rec["torque_applied"] ** 2))),
        torque_max=float(np.nanmax(np.abs(rec["torque_applied"]))),
        saturation_share=saturated / n_ticks,
        diverged=diverged,
    )
    return rec
