from dataclasses import replace
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from config import DEADZONE, DEADZONE_MAX, REAL_STAND, SENSOR_NOISE, TORQUE_LIMIT, integral_gain
from simulation import simulate

SEEDS = (0, 1, 2)
DEADZONES = (0.0, 0.02, DEADZONE, 0.1, DEADZONE_MAX)
ESTIMATE_FACTORS = (0.0, 0.5, 1.0, 1.5, 2.0)
PULSES = ((5.0, 1.0, 0.8), (5.0, 1.0, 1.0))
RESULTS = Path(__file__).resolve().parent / "results"


def evaluate(**kwargs):
    runs = [simulate(noise=SENSOR_NOISE, seed=s, **kwargs) for s in SEEDS]
    return {
        "rmse": float(np.mean([r["rmse_deg"] for r in runs])),
        "max": float(np.max([r["max_error_deg"] for r in runs])),
        "sat": float(np.mean([r["saturation_share"] for r in runs])),
        "diverged": any(r["diverged"] for r in runs),
    }


def fmt(m):
    if m["diverged"]:
        return f"{'falls':>16}"
    return f"{m['rmse']:7.3f} /{m['max']:7.2f}"


def dead_zone_table(c):
    variants = (
        ("no compensation", {}),
        ("inverse", {"deadzone_inverse": True}),
        ("integral + AW", {"c": c}),
        ("inverse + integral + AW", {"c": c, "deadzone_inverse": True}),
    )
    print("\n1) dead zone ±d on the stand model: RMSE / max error, deg")
    print(f"   {'d, N m':>8} " + " ".join(f"{name:>24}" for name, _ in variants))
    for d in DEADZONES:
        stand = replace(REAL_STAND, deadzone=(-d, d))
        print(f"   {d:8.3f} " + " ".join(f"{fmt(evaluate(stand=stand, **kw)):>24}" for _, kw in variants))


def estimate_table(c):
    stand = replace(REAL_STAND, deadzone=(-DEADZONE, DEADZONE))
    print(f"\n2) wrong dead-zone estimate in the inverse, true d = {DEADZONE} N m: RMSE / max error, deg")
    print(f"   {'estimate':>9} {'inverse':>18} {'inverse + integral':>20}")
    for factor in ESTIMATE_FACTORS:
        est = (-factor * DEADZONE, factor * DEADZONE)
        m1 = evaluate(stand=stand, deadzone_inverse=True, deadzone_estimate=est)
        m2 = evaluate(stand=stand, deadzone_inverse=True, deadzone_estimate=est, c=c)
        print(f"   {factor:8.1f}d {fmt(m1):>18} {fmt(m2):>20}")


def windup_table(c):
    variants = (
        ("no integral", {}),
        ("integral, no AW", {"c": c, "anti_windup": False}),
        ("integral + AW", {"c": c}),
    )
    print(f"\n3) torque pulse during tracking, limit {TORQUE_LIMIT:.3f} N m: RMSE / max error, deg")
    for t0, duration, amplitude in PULSES:
        pulse = (t0, duration, amplitude)
        cells = " ".join(f"{name}: {fmt(evaluate(stand=REAL_STAND, pulse=pulse, **kw)).strip()}" for name, kw in variants)
        print(f"   {amplitude} N m for {duration} s at t = {t0} s -> {cells}")


def figure(c):
    RESULTS.mkdir(exist_ok=True)
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), sharex="col")
    stand = replace(REAL_STAND, deadzone=(-DEADZONE, DEADZONE))
    for name, kw, style in (("no compensation", {}, "C3"),
                            ("inverse", {"deadzone_inverse": True}, "C0"),
                            ("integral + AW", {"c": c}, "C2")):
        r = simulate(noise=SENSOR_NOISE, stand=stand, seed=2, **kw)
        axes[0, 0].plot(r["time"], np.degrees(r["error"]), style, linewidth=0.9, label=name)
        axes[1, 0].plot(r["time"], r["torque_applied"], style, linewidth=0.7, label=name)
    axes[0, 0].set_title(f"Dead zone ±{DEADZONE} N m")
    pulse = PULSES[0]
    for name, kw, style in (("no integral", {}, "C3"),
                            ("integral, no AW", {"c": c, "anti_windup": False}, "C1"),
                            ("integral + AW", {"c": c}, "C2")):
        r = simulate(noise=SENSOR_NOISE, stand=REAL_STAND, seed=2, pulse=pulse, **kw)
        axes[0, 1].plot(r["time"], np.degrees(r["alpha"]), style, linewidth=1, label=name)
        axes[1, 1].plot(r["time"], r["u"], style, linewidth=0.7, label=name)
    axes[0, 1].plot(r["time"], np.degrees(r["alpha_d"]), "k--", linewidth=1, label="reference")
    axes[0, 1].axvspan(pulse[0], pulse[0] + pulse[1], color="0.9")
    axes[0, 1].set_title(f"Pulse {pulse[2]} N m for {pulse[1]} s, saturation ±{TORQUE_LIMIT:.2f} N m")
    for ax in axes[1, :]:
        ax.axhline(TORQUE_LIMIT, color="gray", linestyle=":")
        ax.axhline(-TORQUE_LIMIT, color="gray", linestyle=":")
        ax.set_xlabel("Time, s")
    axes[0, 0].set_ylabel("Error, deg")
    axes[1, 0].set_ylabel("Applied torque, N m")
    axes[0, 1].set_ylabel("Angle, deg")
    axes[1, 1].set_ylabel("Controller output u, N m")
    axes[1, 1].set_ylim(-4 * TORQUE_LIMIT, 4 * TORQUE_LIMIT)
    for ax in axes.flat:
        ax.grid(True)
        ax.legend(loc="upper right", fontsize=9)
    fig.tight_layout()
    path = RESULTS / "actuator_study.png"
    fig.savefig(path, dpi=130)
    print(f"\nfigure saved to {path}")


def main():
    c = integral_gain()
    print(f"integral gain c = {c:g}, torque limit = {TORQUE_LIMIT:.3f} N m, dead zone upper bound = {DEADZONE_MAX:.3f} N m")
    dead_zone_table(c)
    estimate_table(c)
    windup_table(c)
    figure(c)


if __name__ == "__main__":
    main()
