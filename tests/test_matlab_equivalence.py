from pathlib import Path

import numpy as np
import pytest

from uav_mfc import SimConfig, simulate

REFERENCE = np.loadtxt(Path(__file__).parent / "data" / "octave_reference.csv", delimiter=",")
CASES = [(1, "none", 0.134618102459), (2, "inverse_model", 0.134784701946), (3, "mfc", 0.142265749700)]


@pytest.mark.parametrize("case_id, loop, rmse_deg", CASES)
def test_matches_matlab_code_run_in_octave(case_id, loop, rmse_deg):
    res = simulate(config=SimConfig(inner_loop=loop))
    ref = REFERENCE[REFERENCE[:, 0] == case_id]
    idx = np.arange(0, len(res.alpha), 100)
    assert len(res.alpha) == 20000
    np.testing.assert_allclose(res.time[idx], ref[:, 1], atol=1e-12)
    np.testing.assert_allclose(res.alpha[idx + 1], ref[:, 2], atol=1e-12)
    np.testing.assert_allclose(res.alpha_dot[idx + 1], ref[:, 3], atol=1e-10)
    np.testing.assert_allclose(res.torque[idx], ref[:, 4], atol=1e-12)
    assert res.rmse_deg == pytest.approx(rmse_deg, abs=1e-9)
