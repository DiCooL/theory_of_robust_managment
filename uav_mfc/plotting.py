import matplotlib.pyplot as plt
import numpy as np

from .simulation import SimResult


def plot_result(res: SimResult, title: str = "", show: bool = True, save_path: str | None = None):
    deg = np.rad2deg
    fig, axes = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
    rows = [
        (res.alpha, res.alpha_d, "Angle, deg", deg),
        (res.alpha_dot, res.alpha_dot_d, "Angular velocity, deg/s", deg),
        (res.alpha_ddot, res.alpha_ddot_d, "Angular acceleration, deg/s²", deg),
    ]
    for ax, (actual, desired, label, conv) in zip(axes, rows):
        ax.plot(res.time, conv(actual), "b", lw=1.5, label="Actual")
        ax.plot(res.time, conv(desired), "r--", lw=1.5, label="Desired")
        ax.set_ylabel(label)
        ax.legend(loc="best")
        ax.grid(True)
    axes[3].plot(res.time, deg(res.error), "k", lw=1.2)
    axes[3].set_ylabel("Error, deg")
    axes[3].set_xlabel("Time, s")
    axes[3].grid(True)
    fig.suptitle(f"{title}  RMSE = {res.rmse_deg:.4f} deg".strip())
    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=150)
    if show:
        plt.show()
    return fig
