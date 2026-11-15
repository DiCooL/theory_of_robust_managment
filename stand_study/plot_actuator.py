from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyBboxPatch

from models.actuator import actuator, dead_zone, saturation

FIGURES = Path(__file__).resolve().parent.parent / "docs" / "figures"
U_MINUS, U_PLUS = -1.0, 1.0
SIGMA = 2.0
INK = "#1f3a93"
GUIDE = "#9a9a9a"
ACTIVE = "#e8eefc"


def axes_cross(ax, xlim, ylim, xlabel, ylabel):
    ax.axhline(0, color="black", linewidth=0.8)
    ax.axvline(0, color="black", linewidth=0.8)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_xticks([])
    ax.set_yticks([])
    for side in ("top", "right", "left", "bottom"):
        ax.spines[side].set_visible(False)
    ax.text(xlim[1], -0.12 * ylim[1], xlabel, ha="right", va="top", fontsize=12)
    ax.text(0.1, ylim[1], ylabel, ha="left", va="top", fontsize=12)


def tick_x(ax, x, label, ylim):
    ax.plot([x, x], [-0.04 * ylim[1], 0.04 * ylim[1]], color="black", linewidth=0.8)
    ax.text(x, 0.08 * ylim[1], label, ha="center", va="bottom", fontsize=11)


def tick_y(ax, y, label, xlim):
    ax.plot([-0.03 * xlim[1], 0.03 * xlim[1]], [y, y], color="black", linewidth=0.8)
    ax.text(-0.06 * xlim[1], y, label, ha="right", va="center", fontsize=11)


def main():
    u = np.linspace(-4.5, 4.5, 901)
    xlim, ylim = (-4.5, 4.5), (-3.0, 3.0)
    fig, axes = plt.subplots(2, 2, figsize=(11, 8.5))

    ax = axes[0, 0]
    axes_cross(ax, xlim, ylim, "u", "μ(u)")
    ax.plot(u, [dead_zone(x, U_MINUS, U_PLUS) for x in u], color=INK, linewidth=2.2)
    ax.plot([U_MINUS, U_PLUS], [0, 0], color=INK, linewidth=4)
    tick_x(ax, U_MINUS, "u₋", ylim)
    tick_x(ax, U_PLUS, "u₊", ylim)
    ax.set_title("Зона нечувствительности μ(u)", fontsize=13)

    ax = axes[0, 1]
    axes_cross(ax, xlim, ylim, "u", "σ_sat(u)")
    ax.plot(u, u, color=GUIDE, linestyle="--", linewidth=1)
    ax.text(2.2, 2.55, "σ = u", color=GUIDE, fontsize=11, ha="right")
    ax.plot(u, [saturation(x, SIGMA) for x in u], color=INK, linewidth=2.2)
    tick_y(ax, SIGMA, "σ₊", xlim)
    tick_y(ax, -SIGMA, "σ₋", xlim)
    ax.set_title("Насыщение σ_sat(u)", fontsize=13)

    ax = axes[1, 0]
    axes_cross(ax, xlim, ylim, "u", "u* = σ_sat(μ(u))")
    lo, hi = U_MINUS - SIGMA, U_PLUS + SIGMA
    ax.axvspan(lo, U_MINUS, color=ACTIVE)
    ax.axvspan(U_PLUS, hi, color=ACTIVE)
    ax.plot(u, [actuator(x, U_MINUS, U_PLUS, SIGMA) for x in u], color=INK, linewidth=2.2)
    tick_x(ax, U_MINUS, "u₋", ylim)
    tick_x(ax, U_PLUS, "u₊", ylim)
    tick_y(ax, SIGMA, "σ₊", xlim)
    tick_y(ax, -SIGMA, "σ₋", xlim)
    ax.text(0.5 * (U_PLUS + hi), -1.0, "активный\nучасток", ha="center", va="top", fontsize=10, color=INK)
    ax.text(0.5 * (lo + U_MINUS), 1.0, "активный\nучасток", ha="center", va="bottom", fontsize=10, color=INK)
    ax.set_title("Зона нечувствительности + насыщение", fontsize=13)

    ax = axes[1, 1]
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 6)
    ax.axis("off")
    for x0, label in ((2.2, "μ"), (5.4, "σ_sat")):
        ax.add_patch(FancyBboxPatch((x0, 2.4), 1.8, 1.2, boxstyle="round,pad=0.05", fill=False, edgecolor="black"))
        ax.text(x0 + 0.9, 3.0, label, ha="center", va="center", fontsize=15)
    arrow = dict(arrowstyle="-|>", color="black", linewidth=1.2)
    ax.annotate("", xy=(2.2, 3.0), xytext=(0.6, 3.0), arrowprops=arrow)
    ax.annotate("", xy=(5.4, 3.0), xytext=(4.0, 3.0), arrowprops=arrow)
    ax.annotate("", xy=(8.9, 3.0), xytext=(7.2, 3.0), arrowprops=arrow)
    ax.text(0.6, 3.25, "u", fontsize=13)
    ax.text(8.5, 3.25, "u*", fontsize=13)
    ax.text(5.0, 1.4, "ẋ = Ax + B u*,   u* = σ_sat(μ(u))", ha="center", fontsize=12)
    ax.text(5.0, 0.7, "сначала зона нечувствительности, затем насыщение", ha="center", fontsize=10, color="#555555")
    ax.set_title("Порядок звеньев", fontsize=13)

    fig.tight_layout()
    FIGURES.mkdir(parents=True, exist_ok=True)
    for ext in ("png", "svg"):
        fig.savefig(FIGURES / f"actuator_nonlinearities.{ext}", dpi=150)
    print(f"saved to {FIGURES}")


if __name__ == "__main__":
    main()
