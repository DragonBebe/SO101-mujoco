import importlib.util
from pathlib import Path

path = Path('logs/2026-09-21-real-joint-grasp/tools/joint_control.py')
spec = importlib.util.spec_from_file_location('joint_control', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
actual = {1: 2048}
limits = {1: (1000, 3000)}
for degrees in (-10, -5, -0.5, 0, 0.5, 5, 10):
    goal = module.bounded_goal({'shoulder_pan': degrees}, actual, limits)
    assert goal == {1: 2048 + round(degrees * 4095 / 360)}, goal
for degrees in (-10.01, 10.01, float('nan'), float('inf'), True):
    try:
        module.bounded_goal({'shoulder_pan': degrees}, actual, limits)
    except ValueError:
        pass
    else:
        raise AssertionError(f'accepted invalid delta: {degrees}')
try:
    module.bounded_goal({'shoulder_pan': 10}, {1: 2990}, limits)
except ValueError:
    pass
else:
    raise AssertionError('accepted target outside joint range')
print('PASS: 7 valid amplitudes, 5 invalid amplitudes, and joint range rejection; no serial port opened')
