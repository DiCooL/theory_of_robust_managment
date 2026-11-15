import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "stand_study"))

from models.actuator import actuator, dead_zone, dead_zone_inverse, saturation, saturation_excess


def test_saturation():
    assert saturation(2.0, 1.0) == 1.0
    assert saturation(-2.0, 1.0) == -1.0
    assert saturation(0.5, 1.0) == 0.5
    assert saturation(5.0, None) == 5.0


def test_dead_zone():
    assert dead_zone(0.03, -0.05, 0.05) == 0.0
    assert dead_zone(0.15, -0.05, 0.05) == pytest.approx(0.1)
    assert dead_zone(-0.15, -0.05, 0.05) == pytest.approx(-0.1)


@pytest.mark.parametrize("v", [-0.7, -0.2, 0.0, 0.2, 0.7])
def test_dead_zone_inverse_cancels_dead_zone(v):
    assert dead_zone(dead_zone_inverse(v, -0.05, 0.05), -0.05, 0.05) == pytest.approx(v)


def test_actuator_order_and_excess():
    assert actuator(2.0, -0.05, 0.05, 1.0) == 1.0
    assert actuator(0.04, -0.05, 0.05, 1.0) == 0.0
    assert saturation_excess(2.0, -0.05, 0.05, 1.0) == pytest.approx(1.0 - 1.95)
    assert saturation_excess(0.5, -0.05, 0.05, 1.0) == 0.0
