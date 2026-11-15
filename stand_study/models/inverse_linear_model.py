from config import MU


def inverse_linear_model(alpha_dot: float) -> float:
    return 2 * MU * alpha_dot
