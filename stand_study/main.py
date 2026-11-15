import numpy as np
import matplotlib.pyplot as plt

from config import SENSOR_NOISE
from simulation import simulate


def plot_result(res: dict) -> None:
    deg = 180 / np.pi
    t = res["time"]

    fig1, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    ax1.plot(t, res["alpha_dot"] * deg, "b-", linewidth=2, label="Actual angular velocity")
    ax1.plot(t, res["alpha_dot_d"] * deg, "r--", linewidth=2, label="Desired angular velocity")
    ax1.set_xlabel("Time, s")
    ax1.set_ylabel("Angular velocity, deg/s")
    ax1.legend(loc="best")
    ax1.grid(True)
    ax2.plot(t, res["torque_cmd"], "b-", linewidth=1)
    ax2.set_xlabel("Time, s")
    ax2.set_ylabel("Commanded torque, N m")
    ax2.grid(True)
    fig1.tight_layout()

    fig2, ax = plt.subplots(figsize=(12, 5))
    ax.plot(t, res["error"] * deg, "k-", linewidth=2)
    ax.axhline(y=0, color="gray", linestyle="--", linewidth=0.5)
    ax.set_xlabel("Time, s")
    ax.set_ylabel("Error, deg")
    ax.grid(True)
    fig2.tight_layout()

    fig3, ax = plt.subplots(figsize=(12, 5))
    ax.plot(t, res["alpha"] * deg, "b-", linewidth=2, label="Actual angle")
    ax.plot(t, res["alpha_d"] * deg, "r--", linewidth=2, label="Desired angle")
    ax.set_xlabel("Time, s")
    ax.set_ylabel("Angle, deg")
    ax.legend(loc="best")
    ax.grid(True)
    fig3.tight_layout()

    plt.show()


def exp3_1() -> dict:
    res = simulate(inner_loop="mfc", noise=SENSOR_NOISE)
    print(f"RMSE = {res['rmse_deg']:.4f} deg")
    print(f"max|e| = {res['max_error_deg']:.4f} deg")
    print(f"torque: rms = {res['torque_rms']:.3f} N m, max = {res['torque_max']:.3f} N m")
    return res


if __name__ == "__main__":
    plot_result(exp3_1())
