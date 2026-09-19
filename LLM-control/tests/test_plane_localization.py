"""Depth-free 3D from RGB: the plane assumption, end to end.

The estimate under test consumes only rendered images and calibration.
Ground-truth object poses appear solely as the test oracle, after the fact.
"""
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nexus_vision.cameras import CameraSuite
from nexus_vision.perception import intersect_pixel_with_plane
from nexus_vision.workspace import WorkspaceSimulation

CUBE_HEIGHT = 0.025


def workspace(directory, modality='rgb', placement='side', seed=4):
    return WorkspaceSimulation(directory, seed=seed, cameras=CameraSuite(
        modality=modality, placement=placement))


def red_cube(sim):
    """Oracle only: the carried cube's true centre."""
    slot = sim.env._slots[0]
    return sim.data.qpos[slot.qpos_addr:slot.qpos_addr + 3].copy()


def grasp_estimate(sim, frame_id=None, camera=None, plane_z=None):
    command = {'action': 'propose', 'frame_id': frame_id or sim.frame_id,
               'color': 'red', 'object_height': CUBE_HEIGHT}
    if camera:
        command['camera'] = camera
    if plane_z is not None:
        command['plane_z'] = plane_z
    proposals = [item for item in sim.execute(command)['proposals'] if item.get('support')]
    if not proposals:
        return None
    return max(proposals, key=lambda item: item['support']['cells'])['support']


def test_plane_intersection_is_exact_monocular_geometry():
    # Camera two metres up, looking straight down, +X right and +Y up.
    calibration = {'width': 101, 'height': 101, 'fx': 50.0, 'fy': 50.0,
                   'cx': 50.0, 'cy': 50.0, 'position': [0.0, 0.0, 2.0],
                   'rotation': np.eye(3)}
    # Principal ray lands directly under the camera on the chosen plane.
    np.testing.assert_allclose(
        intersect_pixel_with_plane(calibration, [50, 50], 0.5), [0, 0, 0.5], atol=1e-12)
    # One focal length off-axis at 1.5 m range is 1.5 m across, and image rows
    # count downward while world +Y goes up in the frame.
    np.testing.assert_allclose(
        intersect_pixel_with_plane(calibration, [100, 50], 0.5), [1.5, 0, 0.5], atol=1e-12)
    np.testing.assert_allclose(
        intersect_pixel_with_plane(calibration, [50, 0], 0.5), [0, 1.5, 0.5], atol=1e-12)
    # A plane above a downward-looking camera is never reached.
    with pytest.raises(ValueError, match='never reaches'):
        intersect_pixel_with_plane(calibration, [50, 50], 3.0)


@pytest.mark.parametrize('placement', ['side', 'overhead'])
def test_plane_localize_agrees_with_depth_on_the_support_surface(tmp_path, placement):
    """Two independent estimators of the same table point must coincide.

    Depth measures it; the plane assumption derives it. Agreement checks the
    new placement's calibration from both directions at once.
    """
    with workspace(tmp_path / 'world', 'rgbd', placement) as sim:
        _, depth, calibration = sim.frames[placement]
        for pixel in ([320, 240], [260, 300], [400, 220]):
            measured = sim.execute({'action': 'localize', 'frame_id': sim.frame_id,
                                    'pixel': pixel})['surface_world']
            if abs(measured[2]) > 0.002:
                continue  # Not bare table: an object or the arm is in the way.
            derived = sim.execute({'action': 'localize', 'frame_id': sim.frame_id,
                                   'pixel': pixel, 'method': 'plane',
                                   'plane_z': 0.0})
            assert derived['source']['method'] == 'ray_plane_intersection'
            np.testing.assert_allclose(derived['surface_world'], measured, atol=0.002)


@pytest.mark.parametrize('placement', ['side', 'overhead'])
def test_rgb_plane_carve_locates_the_cube_from_images_alone(tmp_path, placement):
    with workspace(tmp_path / 'world', 'rgb', placement) as sim:
        support = grasp_estimate(sim)
        assert support is not None and support['method'] == 'rgb_two_plane_carve'
        assert support['assumed_plane_z'] == 0.0
        centre = red_cube(sim)
        estimate = np.asarray(support['grasp_center'])
        assert abs(estimate[2] - centre[2]) < 0.001
        assert np.linalg.norm(estimate[:2] - centre[:2]) < 0.005
        # The carve bounds the footprint from outside, so the extent is the
        # cube's size plus a small margin -- the caller's sanity check that a
        # blob really is the object it expected.
        assert all(0.02 <= value <= 0.05 for value in support['extent']), support


def test_rgb_only_pipeline_grasps_and_lifts_the_cube(tmp_path):
    """The headline claim: images in, a lifted cube out, with no depth at all."""
    with workspace(tmp_path / 'world', 'rgb', 'side') as sim:
        support = grasp_estimate(sim)
        x, y, z = support['grasp_center']
        sim.execute({'action': 'gripper', 'opening': 1.0, 'seconds': 1.0})
        # A fully-down gripper cannot hold that pose arbitrarily high at this
        # reach, so back off the clearance rather than give up.
        for clearance in (0.09, 0.07, 0.055, 0.04):
            try:
                sim.execute({'action': 'move', 'position': [x, y, z + clearance],
                             'orientation': 'down', 'seconds': 2})
                break
            except ValueError:
                continue
        else:
            pytest.fail('no reachable hover pose above the estimated grasp centre')
        # Refine from the wrist camera before committing, as the protocol advises.
        refined = grasp_estimate(sim, camera='wrist')
        assert refined is not None
        x, y, z = refined['grasp_center']
        sim.execute({'action': 'move', 'position': [x, y, z], 'orientation': 'down',
                     'seconds': 2, 'linear': True})
        sim.execute({'action': 'gripper', 'opening': 0.0, 'seconds': 1.5})
        lifted = sim.execute({'action': 'move', 'position': [x, y, z + clearance],
                              'orientation': 'down', 'seconds': 2})
        sim.execute({'action': 'wait', 'seconds': 0.5})
        assert not list(Path(sim.directory).glob('*.npy'))
        assert red_cube(sim)[2] > 0.05, 'the cube never left the table'

        # Confirming the lift from RGB is a consistency test against forward
        # kinematics, not a measurement: a held cube's underside sits about
        # half a cube below the gripper, so the carve run at that plane lands
        # near the gripper while the table plane -- now a false assumption --
        # throws the estimate far away. That gap is the evidence.
        tool = np.asarray(lifted['robot']['tcp'][:3])
        held = grasp_estimate(sim, camera='side', plane_z=tool[2] - CUBE_HEIGHT / 2)
        on_table = grasp_estimate(sim, camera='side', plane_z=0.0)
        assert held is not None and on_table is not None
        near = np.linalg.norm(np.asarray(held['center'])[:2] - tool[:2])
        far = np.linalg.norm(np.asarray(on_table['center'])[:2] - tool[:2])
        assert near < 0.03 and far > 0.04, (near, far)
