import numpy as np

from uav_mfc import NO_NOISE, STRONG_NOISE, THESIS_NOISE, SimConfig, simulate

CASES = [
    ("exp1", "none", NO_NOISE, 0.1280),
    ("exp2", "inverse_model", NO_NOISE, 0.1281),
    ("exp3", "mfc", NO_NOISE, 0.1360),
    ("exp1_1", "none", THESIS_NOISE, 1.0294),
    ("exp2_1", "inverse_model", STRONG_NOISE, 8.8486),
    ("exp3_1", "mfc", THESIS_NOISE, 1.0129),
]

N_SEEDS = 30


def main():
    print(f"{'case':8} {'inner loop':14} {'Python RMSE, deg':>22} {'Octave RMSE, deg':>17}")
    for name, loop, noise, octave in CASES:
        if noise.enabled:
            values = [simulate(noise=noise, config=SimConfig(inner_loop=loop, seed=s)).rmse_deg for s in range(N_SEEDS)]
            py = f"{np.mean(values):.4f} ± {np.std(values):.4f}"
        else:
            py = f"{simulate(config=SimConfig(inner_loop=loop)).rmse_deg:.4f}"
        print(f"{name:8} {loop:14} {py:>22} {octave:>17.4f}")
    print(f"\nNoisy cases: mean ± std over {N_SEEDS} seeds; MATLAB/Octave random streams differ from NumPy.")
    print("Thesis (MATLAB, rng(2)) reports RMSE = 1.002 deg for exp3_1.")
    print("Octave/MATLAB RMSE pairs alpha(t_k+1) with alpha_d(t_k); Python pairs both at t_k.")


if __name__ == "__main__":
    main()
