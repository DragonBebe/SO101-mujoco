import pytest
from joint_control import bounded_goal

def test_delta_is_relative_to_measured_position():
    assert bounded_goal({'shoulder_pan': 2}, {1: 2020}, {1: (853,3349)}) == {1: 2043}

@pytest.mark.parametrize('delta', [{'shoulder_pan': 6}, {'shoulder_pan': float('nan')}, {'bad': 1}])
def test_invalid_step_rejected(delta):
    with pytest.raises(ValueError): bounded_goal(delta, {1: 2020}, {1:(853,3349)})

def test_hardware_limit_rejected():
    with pytest.raises(ValueError): bounded_goal({'shoulder_pan': 5}, {1:3340}, {1:(853,3349)})
