# SPDX-License-Identifier: Apache-2.0
import math

import pytest

from robot_node.velocity_filter import VelocityFilter


def test_startup_timeout_and_recovery():
    limiter = VelocityFilter(command_timeout=0.5)
    assert limiter.sample(10.0) == (0.0, 0.0)
    assert limiter.receive((0.2, 0.0, 0.0), (0.0, 0.0, -0.4), 10.0)
    assert limiter.sample(10.49) == (0.2, -0.4)
    assert limiter.sample(10.5) == (0.0, 0.0)
    assert limiter.sample(20.0) == (0.0, 0.0)
    assert limiter.receive((-0.1, 0.0, 0.0), (0.0, 0.0, 0.2), 21.0)
    assert limiter.sample(21.1) == (-0.1, 0.2)


@pytest.mark.parametrize('sign', [-1.0, 1.0])
def test_limits_and_differential_drive_axes(sign):
    limiter = VelocityFilter(max_linear=0.3, max_angular=1.0)
    limiter.receive((sign * 8.0, 2.0, 3.0), (4.0, 5.0, sign * 9.0), 0.0)
    assert limiter.sample(0.1) == (sign * 0.3, sign * 1.0)


@pytest.mark.parametrize('invalid', [math.nan, math.inf, -math.inf])
@pytest.mark.parametrize('component', range(6))
def test_invalid_command_clears_previous_motion(invalid, component):
    limiter = VelocityFilter()
    limiter.receive((0.2, 0.0, 0.0), (0.0, 0.0, 0.7), 0.0)
    values = [0.1, 0.0, 0.0, 0.0, 0.0, 0.2]
    values[component] = invalid
    assert not limiter.receive(values[:3], values[3:], 0.1)
    assert limiter.sample(0.2) == (0.0, 0.0)


def test_latest_valid_command_renews_deadline():
    limiter = VelocityFilter(command_timeout=0.5)
    limiter.receive((0.1, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0)
    limiter.receive((0.2, 0.0, 0.0), (0.0, 0.0, 0.0), 0.4)
    assert limiter.sample(0.6) == (0.2, 0.0)
    assert limiter.sample(0.91) == (0.0, 0.0)


@pytest.mark.parametrize('name', ['max_linear', 'max_angular', 'command_timeout'])
@pytest.mark.parametrize('value', [0.0, -1.0, math.nan, math.inf])
def test_invalid_configuration(name, value):
    with pytest.raises(ValueError, match=name):
        VelocityFilter(**{name: value})


def test_explicit_stop_and_invalid_clock():
    limiter = VelocityFilter()
    limiter.receive((0.2, 0.0, 0.0), (0.0, 0.0, 0.0), 10.0)
    limiter.stop()
    assert limiter.sample(10.1) == (0.0, 0.0)
    limiter.receive((0.2, 0.0, 0.0), (0.0, 0.0, 0.0), 10.0)
    assert limiter.sample(9.0) == (0.0, 0.0)
    assert not limiter.receive((0.2, 0.0, 0.0), (0.0, 0.0, 0.0), math.nan)
