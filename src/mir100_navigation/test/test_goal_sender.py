import math

import pytest

from mir100_navigation.goal_math import quaternion_from_yaw


def test_quaternion_from_yaw_returns_planar_orientation():
    x, y, z, w = quaternion_from_yaw(math.pi / 2.0)

    assert x == pytest.approx(0.0)
    assert y == pytest.approx(0.0)
    assert z == pytest.approx(math.sqrt(0.5))
    assert w == pytest.approx(math.sqrt(0.5))
