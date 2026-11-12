import numpy as np
import pytest

from uav_mfc import (
    THESIS_NOISE,
    ControllerParams,
    CorrectionFilter,
    PlantParams,
    SimConfig,
    Sinusoid,
    roll_acceleration,
    simulate,
)


def test_natural_parametrisation():
    c = ControllerParams.from_natural(100, 1.01)
    assert c.a == pytest.approx(-202.0)
    assert c.b == pytest.approx(-10000.0)
    assert c.is_aperiodic()
    assert not ControllerParams.from_natural(100, 0.9).is_aperiodic()


def test_upright_position_is_unstable_equilibrium():
    p = PlantParams()
    assert roll_acceleration(0.0, 0.0, 0.0, p) == 0.0
    assert roll_acceleration(0.0, 1e-3, 0.0, p) > 0.0


def test_filter_step_response_matches_first_order_lag():
    dt, T = 1e-3, 15.0
    f = CorrectionFilter(T, dt)
    y = [f.update(1.0) for _ in range(int(0.1 / dt))]
    assert y[-1] == pytest.approx(1 - np.exp(-0.1 * T), abs=5e-3)


def test_trajectory_derivatives_are_consistent():
    traj = Sinusoid()
    t, h = 3.7, 1e-6
    a_p, ad_p, _ = traj(t + h)
    a_m, ad_m, _ = traj(t - h)
    _, ad, add = traj(t)
    assert (a_p - a_m) / (2 * h) == pytest.approx(ad, rel=1e-6)
    assert (ad_p - ad_m) / (2 * h) == pytest.approx(add, rel=1e-6)


def test_noisy_run_is_reproducible_with_seed():
    r1 = simulate(noise=THESIS_NOISE, config=SimConfig(seed=7))
    r2 = simulate(noise=THESIS_NOISE, config=SimConfig(seed=7))
    np.testing.assert_array_equal(r1.alpha, r2.alpha)


def test_model_defaults_to_plant():
    r1 = simulate(config=SimConfig(t_final=2.0))
    r2 = simulate(model=PlantParams(), config=SimConfig(t_final=2.0))
    np.testing.assert_array_equal(r1.alpha, r2.alpha)
