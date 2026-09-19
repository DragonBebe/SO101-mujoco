"""A persistent multi-object tabletop using the existing Nexus physics."""
from contextlib import nullcontext
from pathlib import Path
import uuid

import mujoco
from so101_nexus.objects import CubeObject, CylinderObject

from .simulation import VisualSimulation


class WorkspaceSimulation(VisualSimulation):
    def __init__(self, directory, seed=4, viewer=False, realtime=False, camera_viewer=False,
                 cameras=None):
        super().__init__(
            'PickAndPlace-v2', directory, seed, viewer, realtime, camera_viewer,
            config_overrides={
                'n_distractors': 3,
                'distractors': [CubeObject(color='green'),
                                CylinderObject(color='yellow'),
                                CubeObject(color='purple', half_size=.0175)],
                'spawn_min_radius': .15, 'spawn_max_radius': .28,
                'spawn_angle_half_range_deg': 65,
                'min_object_separation': .025,
                'min_object_target_separation': .05,
                'target_disc_radius': .035,
            },
            cameras=cameras,
        )

    def reset_world(self, directory, seed):
        """Reuse the compiled world and windows; avoid overlapping GUI lifetimes."""
        directory = Path(directory).resolve()
        directory.mkdir(parents=True, exist_ok=True)
        with self.viewer.lock() if self.viewer else nullcontext():
            self.env.reset(seed=seed)
            mujoco.mj_forward(self.model, self.data)
            self.target = self.data.qpos[self.qadr].copy()
        self.directory, self.seed = directory, seed
        self.episode_token = uuid.uuid4().hex[:16]
        self.frame_index, self.frame_id = 0, ''
        self.frames.clear()
        self.points.clear()


SCENE_DESCRIPTION = {
    'name': 'workbench',
    'objects': ['red cube (25 mm)', 'green cube (25 mm)',
                'yellow cylinder (diameter/height 25 mm)', 'purple cube (35 mm)'],
    'regions': ['blue goal disc (radius 35 mm)'],
    'coordinates': 'world XYZ in meters; z=0 is the support surface',
    'perception': 'RGB-D only; object locations must be observed, not assumed',
    'motion': 'IK with joint limits; no general collision-free path planner',
}


def scene_description(cameras):
    """Scene facts with the perception line matching the configured cameras."""
    description = dict(SCENE_DESCRIPTION)
    if cameras.depth_available:
        description['perception'] = (
            f'RGB-D from the {cameras.environment_camera} and wrist cameras; '
            'object locations must be observed, not assumed')
    else:
        description['perception'] = (
            f'RGB only from the {cameras.environment_camera} and wrist cameras; '
            'no depth. 3D comes from the plane assumption: intersect a pixel ray '
            'with a stated horizontal plane, or give propose an object_height to '
            'estimate where an upright object of that size stands on the table. '
            'Object locations must still be observed, not assumed')
    return description
