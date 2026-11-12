import argparse

import numpy as np

from uav_mfc import FilterParams, SimConfig, grid_search


def parse_args():
    parser = argparse.ArgumentParser(description="Grid search over (omega_n, zeta) by J = mean(e^2)")
    parser.add_argument("--omega", nargs=3, type=float, default=[10, 100, 10], metavar=("START", "STOP", "STEP"))
    parser.add_argument("--zeta", nargs=3, type=float, default=[1.00, 1.02, 0.01], metavar=("START", "STOP", "STEP"))
    parser.add_argument("--T", type=float, default=15.0)
    parser.add_argument("--t-final", type=float, default=10.0)
    parser.add_argument("--top", type=int, default=10)
    return parser.parse_args()


def grid(start, stop, step):
    return np.round(np.arange(start, stop + step / 2, step), 10)


def main():
    args = parse_args()
    points = grid_search(
        grid(*args.omega),
        grid(*args.zeta),
        filt=FilterParams(T=args.T),
        config=SimConfig(t_final=args.t_final, inner_loop="mfc"),
    )
    print(f"{'omega_n':>9} {'zeta':>6} {'a':>10} {'b':>12} {'J, rad^2':>14}")
    for p in points[: args.top]:
        print(f"{p.omega_n:9.2f} {p.zeta:6.3f} {p.a:10.2f} {p.b:12.2f} {p.J:14.4e}")


if __name__ == "__main__":
    main()
