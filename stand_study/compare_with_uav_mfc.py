import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

from config import NO_NOISE, SENSOR_NOISE, THESIS_NOISE
from simulation import simulate

HERE = Path(__file__).resolve().parent
UPSTREAM = next(p for p in (HERE / "_upstream", HERE.parent) if (p / "uav_mfc").is_dir())
sys.path.insert(0, str(UPSTREAM))

import uav_mfc

N_SEEDS = 30
LOOPS = ("none", "inverse_model", "mfc")


def upstream_noise(noise, mode):
    return uav_mfc.NoiseParams(noise.angle_std, noise.rate_std, noise.disturbance_std, mode)


def main():
    print("1) no noise: max |alpha - alpha_uav_mfc| over 20 s, RMSE")
    for loop in LOOPS:
        ours = simulate(inner_loop=loop, noise=NO_NOISE)
        ref = uav_mfc.simulate(config=uav_mfc.SimConfig(inner_loop=loop))
        diff = np.max(np.abs(ours["alpha"] - ref.alpha))
        print(f"   {loop:14} diff = {diff:.2e} rad   RMSE {ours['rmse_deg']:.4f} / {ref.rmse_deg:.4f} deg")

    for label, noise in (("thesis noise 0.3 deg, 0.4 deg/s, 0.004", THESIS_NOISE),
                         ("sensor noise 0.45 deg, 1% rate span, 0.004", SENSOR_NOISE)):
        print(f"2) {label}: RMSE mean ± std over {N_SEEDS} seeds, deg")
        for loop in LOOPS:
            ours, meas, state = [], [], []
            for seed in range(N_SEEDS):
                cfg = uav_mfc.SimConfig(inner_loop=loop, seed=seed)
                ours.append(simulate(inner_loop=loop, noise=noise, seed=seed)["rmse_deg"])
                meas.append(uav_mfc.simulate(noise=upstream_noise(noise, "measurement"), config=cfg).rmse_deg)
                state.append(uav_mfc.simulate(noise=upstream_noise(noise, "state"), config=cfg).rmse_deg)
            print(f"   {loop:14} fixed {np.mean(ours):.4f} ± {np.std(ours):.4f}   "
                  f"uav_mfc measurement {np.mean(meas):.4f} ± {np.std(meas):.4f}   "
                  f"uav_mfc state {np.mean(state):.4f} ± {np.std(state):.4f}")

    ours = simulate(inner_loop="mfc", noise=THESIS_NOISE, seed=2)
    ref = uav_mfc.simulate(noise=upstream_noise(THESIS_NOISE, "measurement"),
                           config=uav_mfc.SimConfig(inner_loop="mfc", seed=2))
    print("3) same seed, mfc, thesis noise, measurement mode:")
    print(f"   max |alpha - alpha_uav_mfc| = {np.max(np.abs(ours['alpha'] - ref.alpha)):.2e} rad")
    print(f"   max |torque_cmd - torque_uav_mfc| = {np.max(np.abs(ours['torque_cmd'] - ref.torque)):.2e} N m")


if __name__ == "__main__":
    main()
