"""A persistent multi-object tabletop using the existing Nexus physics."""
from contextlib import nullcontext
from pathlib import Path
import uuid

import mujoco
from so101_nexus.objects import CubeObject, CylinderObject

from .simulation import VisualSimulation


class WorkspaceSimulation(VisualSimulation):
    def __init__(self, directory, seed=4, viewer=False, realtime=False, camera_viewer=False):
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
