"""Single source of truth for the observation cameras.

Two independent choices, both fixed when the service starts:

``modality``
    ``'rgbd'`` renders and saves optical-Z depth next to every RGB frame;
    ``'rgb'`` renders colour only.  Depth is genuinely absent in ``'rgb'``:
    nothing writes a ``.npy``, and no zero-filled array stands in for it.

``placement``
    Where the environment camera sits.  ``'overhead'`` is the original
    top-down view kept for regression and comparison.  ``'side'`` is a
    world-fixed bystander camera looking obliquely down at the workspace,
    the way a laptop webcam on the corner of the desk would.  It is a
    simulation starting point, not a calibrated copy of real hardware.

The wrist camera is part of the robot model and rides the arm in either
placement; only the environment camera moves between the two.

Geometry is derived from the scene's own spawn parameters rather than
copied coordinates: :meth:`CameraSuite.side_camera` fits the camera
distance so the whole reachable workspace stays inside the frustum.
"""
from dataclasses import dataclass, replace

import numpy as np

MODALITIES = ('rgb', 'rgbd')
PLACEMENTS = ('overhead', 'side')
WRIST = 'wrist'


def _choice(value, allowed, name):
    if value not in allowed:
        raise ValueError(f'{name} must be one of {", ".join(allowed)}; got {value!r}')
    return value


def _positive_integer(value, name):
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f'{name} must be a positive integer')
    return value


def _angle(value, low, high, name):
    value = float(value)
    if not np.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{name} must be a finite number in [{low}, {high}]')
    return value


def workspace_points(spawn_center, spawn_max_radius, angle_half_deg, margin, height):
    """Points the environment camera has to keep in frame, in world metres.

    Derived from the scene's own spawn geometry rather than copied
    coordinates: the outer spawn arc (radius plus ``margin``, swept over the
    configured angular half-range) bounds where objects can appear, and the
    column at the robot base bounds the arm itself.  Both are sampled at the
    support surface and at ``height`` so lifted objects and a raised gripper
    stay visible.  An axis-aligned box would instead include the empty near
    corner beside the robot and push the camera needlessly far back.
    """
    center_x, center_y = (float(value) for value in spawn_center)
    reach = float(spawn_max_radius) + float(margin)
    angles = np.radians(np.linspace(-float(angle_half_deg), float(angle_half_deg), 9))
    arc = np.column_stack((center_x + reach * np.cos(angles),
                           center_y + reach * np.sin(angles),
                           np.zeros_like(angles)))
    base = np.array([[0.0, 0.0, 0.0]])
    ground = np.vstack((arc, base))
    raised = ground + [0.0, 0.0, float(height)]
    return np.vstack((ground, raised))


def _orbit_axes(azimuth_deg, elevation_deg):
    """Camera forward/right/up for MuJoCo free-camera orbit angles."""
    azimuth = np.radians(azimuth_deg)
    elevation = np.radians(elevation_deg)
    forward = np.array([np.cos(elevation) * np.cos(azimuth),
                        np.cos(elevation) * np.sin(azimuth),
                        np.sin(elevation)])
    right = np.array([-np.sin(azimuth), np.cos(azimuth), 0.0])
    return forward, right, np.cross(right, forward)


def fit_orbit_distance(points, azimuth_deg, elevation_deg, fov_deg, aspect, fill):
    """Lookat and the smallest orbit distance that frames ``points``.

    The camera sits at ``lookat - distance * forward``.  For a point ``w``
    measured from the lookat, the in-frame condition on the right axis is
    ``|w.right| <= tan(hfov/2) * (w.forward + distance)``, which solves
    directly for the distance that point needs; the up axis is the same.
    ``fill`` shrinks the usable half-angles so the workspace keeps a border.
    """
    if not 0 < fill <= 1:
        raise ValueError('fill must be in (0, 1]')
    points = np.asarray(points, dtype=float)
    forward, right, up = _orbit_axes(azimuth_deg, elevation_deg)
    lookat = (points.min(axis=0) + points.max(axis=0)) / 2
    half_vertical = np.arctan(np.tan(np.radians(fov_deg) / 2) * fill)
    half_horizontal = np.arctan(np.tan(np.radians(fov_deg) / 2) * aspect * fill)
    offsets = points - lookat
    depth = offsets @ forward
    needed = [np.abs(offsets @ right) / np.tan(half_horizontal) - depth,
              np.abs(offsets @ up) / np.tan(half_vertical) - depth]
    return lookat, float(max(np.max(needed), 0.05))


@dataclass(frozen=True)
class CameraSuite:
    """Immutable camera configuration shared by capture, preview and logs.

    The side-view angles are deliberately grouped here rather than spread
    through the control code: changing the simulated laptop-camera viewpoint
    is a one-line edit or a CLI flag, never a code change in ``simulation``.
    """

    modality: str = 'rgbd'
    placement: str = 'overhead'
    width: int = 640
    height: int = 480
    #: Orbit angles of the side view.  Azimuth is the direction the camera
    #: looks along in the XY plane, so 130 deg puts the camera off the robot's
    #: front-right corner, looking back across the workspace toward the base.
    side_azimuth_deg: float = 130.0
    side_elevation_deg: float = -32.0
    #: Vertical field of view; ~50 deg is laptop-webcam-like once the 4:3
    #: aspect widens it to ~64 deg horizontally.
    side_fov_deg: float = 50.0
    #: Framing: metres of padding beyond the spawn arc, headroom above the
    #: table that must stay visible, and the fraction of the frame the
    #: workspace is allowed to fill.
    side_margin: float = 0.06
    side_height: float = 0.25
    side_fill: float = 0.92
    overhead_fov_deg: float = 45.0
    wrist_fov_deg: float = 60.0
    wrist_pitch_deg: float = -32.66

    def __post_init__(self):
        _choice(self.modality, MODALITIES, 'camera modality')
        _choice(self.placement, PLACEMENTS, 'environment camera')
        _positive_integer(self.width, 'camera width')
        _positive_integer(self.height, 'camera height')
        _angle(self.side_azimuth_deg, -360, 360, 'side_azimuth_deg')
        _angle(self.side_elevation_deg, -89, -1, 'side_elevation_deg')
        _angle(self.side_fov_deg, 5, 150, 'side_fov_deg')
        _angle(self.overhead_fov_deg, 5, 150, 'overhead_fov_deg')

    @property
    def depth_available(self):
        return self.modality == 'rgbd'

    @property
    def environment_camera(self):
        """Name of the world-fixed camera; it is also its placement."""
        return self.placement

    @property
    def names(self):
        return (self.environment_camera, WRIST)

    @property
    def aspect(self):
        return self.width / self.height

    def mounting(self, name):
        return 'world_fixed' if name == self.environment_camera else 'robot_wrist'

    def environment_fov_deg(self):
        return self.side_fov_deg if self.placement == 'side' else self.overhead_fov_deg

    def side_camera(self, spawn_center, spawn_max_radius, spawn_angle_half_deg):
        """Resolve the side view against the live scene scale.

        Returns the MuJoCo free-camera parameters (``lookat``, ``distance``,
        ``azimuth``, ``elevation``) plus the vertical FOV to render with.
        """
        points = workspace_points(spawn_center, spawn_max_radius,
                                  spawn_angle_half_deg, self.side_margin,
                                  self.side_height)
        lookat, distance = fit_orbit_distance(
            points, self.side_azimuth_deg, self.side_elevation_deg,
            self.side_fov_deg, self.aspect, self.side_fill)
        forward, _, _ = _orbit_axes(self.side_azimuth_deg, self.side_elevation_deg)
        return {'lookat': lookat, 'distance': distance,
                'azimuth': self.side_azimuth_deg, 'elevation': self.side_elevation_deg,
                'fov_deg': self.side_fov_deg, 'eye': lookat - distance * forward,
                'framed': {'low': points.min(axis=0).tolist(),
                           'high': points.max(axis=0).tolist()}}

    def preview_view(self):
        """Picklable description the preview window labels its tiles with."""
        return {'modality': self.modality, 'cameras': list(self.names),
                'environment_camera': self.environment_camera}

    def describe(self):
        """Configuration as recorded in session state and every event row."""
        return {'modality': self.modality, 'depth_available': self.depth_available,
                'environment_camera': self.environment_camera,
                'environment_placement': self.placement,
                'environment_mounting': 'world_fixed',
                'wrist_camera': WRIST, 'wrist_mounting': 'robot_wrist',
                'cameras': list(self.names),
                'resolution': {'width': self.width, 'height': self.height},
                'environment_fov_deg': self.environment_fov_deg(),
                'side_angles_deg': {'azimuth': self.side_azimuth_deg,
                                    'elevation': self.side_elevation_deg},
                'note': ('side placement is a simulation starting point, '
                         'not a calibrated copy of physical hardware')}

    def replace(self, **changes):
        return replace(self, **changes)


def suite_from_options(modality=None, placement=None, **changes):
    """Build a suite from CLI-style options, keeping the original defaults."""
    suite = CameraSuite()
    if modality is not None:
        suite = suite.replace(modality=modality)
    if placement is not None:
        suite = suite.replace(placement=placement)
    return suite.replace(**changes) if changes else suite
