import argparse
from dataclasses import replace

from uav_mfc import (
    NO_NOISE,
    STRONG_NOISE,
    THESIS_NOISE,
    TRAJECTORIES,
    ControllerParams,
    FilterParams,
    SimConfig,
    simulate,
)
from uav_mfc.params import INNER_LOOPS, NOISE_MODES

NOISES = {"none": NO_NOISE, "thesis": THESIS_NOISE, "strong": STRONG_NOISE}


def parse_args():
    parser = argparse.ArgumentParser(description="Roll-channel MFC simulation")
    parser.add_argument("--loop", choices=INNER_LOOPS, default="mfc")
    parser.add_argument("--noise", choices=NOISES, default="thesis")
    parser.add_argument("--noise-mode", choices=NOISE_MODES, default="state")
    parser.add_argument("--trajectory", choices=TRAJECTORIES, default="sin")
    parser.add_argument("--omega-n", type=float, default=100.0)
    parser.add_argument("--zeta", type=float, default=1.01)
    parser.add_argument("--T", type=float, default=15.0)
    parser.add_argument("--dt", type=float, default=1e-3)
    parser.add_argument("--t-final", type=float, default=20.0)
    parser.add_argument("--seed", type=int, default=2)
    parser.add_argument("--plot", action="store_true")
    parser.add_argument("--save", default=None)
    return parser.parse_args()


def main():
    args = parse_args()
    noise = replace(NOISES[args.noise], mode=args.noise_mode)
    res = simulate(
        trajectory=TRAJECTORIES[args.trajectory](),
        controller=ControllerParams.from_natural(args.omega_n, args.zeta),
        filt=FilterParams(T=args.T),
        noise=noise,
        config=SimConfig(dt=args.dt, t_final=args.t_final, inner_loop=args.loop, seed=args.seed),
    )
    print(f"loop={args.loop} noise={args.noise}/{args.noise_mode} trajectory={args.trajectory}")
    print(f"RMSE = {res.rmse_deg:.4f} deg, max|e| = {res.max_error_deg:.4f} deg, mean|u| = {res.mean_abs_torque:.4f}")
    if args.plot or args.save:
        from uav_mfc.plotting import plot_result

        plot_result(res, title=f"{args.loop}, noise={args.noise}", show=args.plot, save_path=args.save)


if __name__ == "__main__":
    main()
