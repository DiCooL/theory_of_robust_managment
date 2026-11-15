import math
from dataclasses import replace
from itertools import product
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from config import (
    ANGLE_NOISE_STD,
    IDEAL_STAND,
    INERTIA,
    OMEGA_N,
    RATE_NOISE_STD,
    REAL_STAND,
    SATURATED_STAND,
    SENSOR_NOISE,
    TORQUE_LIMIT,
    ZETA,
    gains,
)
from simulation import simulate

SEEDS = (0, 1, 2)
OMEGA_GRID = (2, 3, 4, 5, 6, 7, 8, 10, 12, 15, 20, 30, 50, 100)
ZETA_GRID = (0.7, 1.01, 1.5, 2.0, 3.0)
RESULTS = Path(__file__).resolve().parent / "results"


def evaluate(omega_n, zeta, stand, inner_loop="mfc", feedforward=False, seeds=SEEDS):
    runs = [simulate(omega_n, zeta, inner_loop, SENSOR_NOISE, stand, feedforward, seed=s) for s in seeds]
    return {
        "rmse": float(np.mean([r["rmse_deg"] for r in runs])),
        "max": float(np.max([r["max_error_deg"] for r in runs])),
        "torque_rms": float(np.mean([r["torque_rms"] for r in runs])),
        "sat": float(np.mean([r["saturation_share"] for r in runs])),
        "diverged": any(r["diverged"] for r in runs),
    }


def fmt(m):
    if m["diverged"]:
        return f"{'diverges':>9} {'-':>9} {m['torque_rms']:9.3f} {100 * m['sat']:7.1f}"
    return f"{m['rmse']:9.3f} {m['max']:9.3f} {m['torque_rms']:9.3f} {100 * m['sat']:7.1f}"


def scenario_table(title, omega_n, zeta, inner_loop, feedforward):
    scenarios = (
        ("ideal loop, no saturation", IDEAL_STAND),
        ("ideal loop, saturation", SATURATED_STAND),
        ("stand model, no saturation", replace(REAL_STAND, torque_limit=None)),
        ("stand model, saturation", REAL_STAND),
    )
    print(f"\n{title}: omega_n={omega_n}, zeta={zeta}, inner_loop={inner_loop}, feedforward={feedforward}")
    print(f"   {'scenario':28} {'RMSE,deg':>9} {'max,deg':>9} {'M rms':>9} {'sat,%':>7}")
    for name, stand in scenarios:
        print(f"   {name:28} {fmt(evaluate(omega_n, zeta, stand, inner_loop, feedforward))}")


def grid(stand, inner_loop, feedforward):
    rows = []
    for omega_n, zeta in product(OMEGA_GRID, ZETA_GRID):
        m = evaluate(omega_n, zeta, stand, inner_loop, feedforward)
        rows.append((omega_n, zeta, m))
    return rows


def best(rows, aperiodic_only):
    ok = [r for r in rows if not r[2]["diverged"] and (r[1] > 1 or not aperiodic_only)]
    return min(ok, key=lambda r: r[2]["rmse"]) if ok else None


def main():
    a, b = gains(OMEGA_N, ZETA)
    kp, kd = -b * INERTIA, -a * INERTIA
    noise_torque = math.hypot(kp * ANGLE_NOISE_STD, kd * RATE_NOISE_STD)
    print(f"torque limit = {TORQUE_LIMIT:.3f} N m")
    print(f"omega_n=100, zeta=1.01: Kp = {kp:.1f} N m/rad, Kd = {kd:.2f} N m s/rad")
    print(f"torque std from sensor noise alone = {noise_torque:.3f} N m "
          f"({noise_torque / TORQUE_LIMIT:.2f} of the limit)")

    scenario_table("current parameters, canonical law", OMEGA_N, ZETA, "mfc", False)

    variants = {}
    for inner_loop, feedforward in product(("none", "mfc"), (False, True)):
        rows = grid(REAL_STAND, inner_loop, feedforward)
        variants[(inner_loop, feedforward)] = rows
        print(f"\ngrid on stand model with saturation: inner_loop={inner_loop}, feedforward={feedforward}")
        print("   RMSE, deg (rows: omega_n, columns: zeta " + ", ".join(str(z) for z in ZETA_GRID) + ")")
        for omega_n in OMEGA_GRID:
            cells = []
            for zeta in ZETA_GRID:
                m = next(r[2] for r in rows if r[0] == omega_n and r[1] == zeta)
                cells.append(f"{'div':>8}" if m["diverged"] else f"{m['rmse']:8.3f}")
            print(f"   {omega_n:5} " + " ".join(cells))
        for label, flag in (("best overall", False), ("best with zeta > 1", True)):
            r = best(rows, flag)
            if r:
                print(f"   {label}: omega_n={r[0]}, zeta={r[1]}: {fmt(r[2])}")

    choice_rows = variants[("none", True)]
    chosen = best(choice_rows, True)
    omega_c, zeta_c = chosen[0], chosen[1]
    scenario_table("selected parameters, feedforward law", omega_c, zeta_c, "none", True)
    scenario_table("selected parameters, canonical law", omega_c, zeta_c, "mfc", False)
    scenario_table("current parameters, feedforward law", OMEGA_N, ZETA, "none", True)

    print(f"\nsensitivity of the selected point (omega_n={omega_c}, zeta={zeta_c}, feedforward, saturation)")
    print(f"   {'motor_tau':>9} {'cam_delay':>9} {'RMSE,deg':>9} {'max,deg':>9} {'M rms':>9} {'sat,%':>7}")
    for tau, delay in product((0.03, 0.05, 0.1, 0.25), (0.03, 0.05, 0.1)):
        stand = replace(REAL_STAND, motor_tau=tau, camera_delay=delay)
        print(f"   {tau:9.2f} {delay:9.2f} {fmt(evaluate(omega_c, zeta_c, stand, 'none', True))}")

    RESULTS.mkdir(exist_ok=True)
    cases = (
        (f"current: omega_n={OMEGA_N:g}, zeta={ZETA}", OMEGA_N, ZETA, "mfc", False),
        (f"selected: omega_n={omega_c:g}, zeta={zeta_c}", omega_c, zeta_c, "none", True),
    )
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
    for col, (label, omega_n, zeta, inner_loop, feedforward) in enumerate(cases):
        for stand, style, name in ((replace(REAL_STAND, torque_limit=None), "C0", "no saturation"),
                                   (REAL_STAND, "C3", "saturation")):
            r = simulate(omega_n, zeta, inner_loop, SENSOR_NOISE, stand, feedforward, seed=2)
            axes[0, col].plot(r["time"], np.degrees(r["alpha"]), style, linewidth=1, label=name)
            axes[1, col].plot(r["time"], r["torque_cmd"], style, linewidth=0.8, label=name)
            ref = r
        axes[0, col].plot(ref["time"], np.degrees(ref["alpha_d"]), "k--", linewidth=1, label="reference")
        axes[0, col].set_title(label + ", stand model")
        axes[0, col].set_ylabel("Angle, deg")
        axes[0, col].set_ylim(-30, 90)
        axes[1, col].axhline(TORQUE_LIMIT, color="gray", linestyle=":")
        axes[1, col].axhline(-TORQUE_LIMIT, color="gray", linestyle=":")
        axes[1, col].set_ylabel("Commanded torque, N m")
        axes[1, col].set_xlabel("Time, s")
        axes[1, col].set_ylim(-3 * TORQUE_LIMIT, 3 * TORQUE_LIMIT)
        for ax in axes[:, col]:
            ax.grid(True)
            ax.legend(loc="upper right")
    fig.tight_layout()
    path = RESULTS / "stand_comparison.png"
    fig.savefig(path, dpi=130)
    print(f"\nfigure saved to {path}")


if __name__ == "__main__":
    main()
