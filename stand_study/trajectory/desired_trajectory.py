import math

from config import TRAJ_AMPLITUDE, TRAJ_BIAS, TRAJ_OMEGA, TRAJ_PHASE


def desired_trajectory(t: float) -> tuple[float, float, float]:
    arg = TRAJ_OMEGA * t + TRAJ_PHASE
    alpha_d = TRAJ_AMPLITUDE * math.sin(arg) + TRAJ_BIAS
    alpha_dot_d = TRAJ_AMPLITUDE * TRAJ_OMEGA * math.cos(arg)
    alpha_ddot_d = -TRAJ_AMPLITUDE * TRAJ_OMEGA**2 * math.sin(arg)
    return alpha_d, alpha_dot_d, alpha_ddot_d
