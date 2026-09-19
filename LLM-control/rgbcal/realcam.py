"""The calibrated real environment camera, in the form the RGB pipeline uses.

``nexus_vision.perception`` was written for a simulated pinhole camera: no
lens distortion, a rotation in OpenGL camera axes, and a horizontal support
plane ``z = plane_z``.  The real camera differs on every one of those, and
this module is where each difference is handled once, explicitly:

Distortion
    Frames are undistorted with the measured ``K``/``D`` into a new pinhole
    camera ``K_new``.  Perception only ever sees the undistorted image
    together with ``K_new`` -- never the raw image with ``K_new`` and never
    the undistorted image with ``K``/``D``.

Axes
    Everything calibrated here is OpenCV (x right, y down, z forward).  The
    rotation handed to perception is converted to OpenGL (x right, y up,
    z backward) by :func:`rgbcal.transforms.opencv_rotation_to_opengl`.

The support plane
    The measured table is tilted about 2 degrees in the robot base frame.
    Rather than approximate it as horizontal, localisation runs in a *table
    frame* whose z axis is the measured table normal and whose z = 0 is the
    table surface.  ``plane_z = 0`` then means "on the table" exactly, and
    ``object_height`` is measured perpendicular to the table, which is what
    stacking needs.  Results are mapped back with ``T_base_table``.

Nothing here makes a coordinate safe to move to.  Every result carries
``usable_for_motion``, which is false unless the extrinsic has passed a
supervised motion check and the capture configuration matches the
calibration.
"""
from dataclasses import dataclass, field
from pathlib import Path
import json

import cv2
import numpy as np

from . import transforms as tf
from .capture import CameraSettings, RealCamera, check_compatible, timestamp

ROOT = Path(__file__).resolve().parent.parent
CALIB = ROOT / 'calib'
#: The calibration this setup was verified with.  Each file is checked on load.
DEFAULTS = {
    'intrinsics': CALIB / 'c920-intrinsics-01/intrinsics.json',
    'extrinsics': CALIB / 'c920-handeye-03/extrinsics.json',
    'table': CALIB / 'c920-table/table.json',
    'profile': CALIB / 'profiles/c920.json',
    'setup': CALIB / 'setup-02/setup.json',
    'motion_check': CALIB / 'c920-motion-check/motion_check.json',
}


def table_frame(table):
    """``T_base_table`` from a measured plane.

    Origin: the point of the table directly below the base origin, along the
    table normal.  z: the table normal.  x: the base x axis projected onto
    the table, so "forward" keeps its meaning.
    """
    normal = np.asarray(table['plane']['normal_base'], dtype=float)
    normal = normal / np.linalg.norm(normal)
    offset = float(table['plane']['offset_m'])
    origin = normal * offset                      # foot of the base origin on the plane
    x_axis = np.array([1.0, 0.0, 0.0]) - normal[0] * normal
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(normal, x_axis)
    return tf.transform(np.column_stack([x_axis, y_axis, normal]), origin)


@dataclass
class Calibration:
    """Everything needed to turn a pixel into robot base coordinates."""

    intrinsics: dict
    extrinsics: dict
    table: dict
    sources: dict = field(default_factory=dict)

    @property
    def K(self):
        return np.array(self.intrinsics['K'])

    @property
    def D(self):
        return np.array(self.intrinsics['D'])

    @property
    def size(self):
        return (self.intrinsics['resolution']['width'],
                self.intrinsics['resolution']['height'])

    @property
    def T_base_camera(self):
        return np.array(self.extrinsics['T_base_camera'])

    @property
    def T_base_table(self):
        return table_frame(self.table)

    @property
    def T_table_camera(self):
        return tf.invert(self.T_base_table) @ self.T_base_camera

    def undistortion(self, alpha=0.0):
        """``K_new`` and the remap tables for a distortion-free image.

        ``alpha = 0`` keeps only valid pixels, so the undistorted image has no
        black border that a colour threshold could mistake for an object.
        """
        width, height = self.size
        new_matrix, _ = cv2.getOptimalNewCameraMatrix(self.K, self.D, (width, height),
                                                      alpha, (width, height))
        maps = cv2.initUndistortRectifyMap(self.K, self.D, None, new_matrix,
                                           (width, height), cv2.CV_16SC2)
        return new_matrix, maps

    def nexus_calibration(self, new_matrix, frame='table'):
        """The dict ``nexus_vision.perception`` expects, for UNDISTORTED images.

        ``position``/``rotation`` place the camera in the chosen frame; the
        rotation is converted to OpenGL camera axes because that is what
        perception assumes.  With ``frame='table'`` a plane of ``plane_z = 0``
        is the table surface.
        """
        pose = self.T_table_camera if frame == 'table' else self.T_base_camera
        width, height = self.size
        return {'width': width, 'height': height,
                'fx': float(new_matrix[0, 0]), 'fy': float(new_matrix[1, 1]),
                'cx': float(new_matrix[0, 2]), 'cy': float(new_matrix[1, 2]),
                'position': pose[:3, 3].tolist(),
                'rotation': tf.opencv_rotation_to_opengl(pose[:3, :3]).tolist(),
                'frame': frame, 'distortion': 'none: undistorted image only'}


def load(intrinsics=None, extrinsics=None, table=None):
    paths = {'intrinsics': Path(intrinsics or DEFAULTS['intrinsics']),
             'extrinsics': Path(extrinsics or DEFAULTS['extrinsics']),
             'table': Path(table or DEFAULTS['table'])}
    # The extrinsic's status is the most important reason to refuse, so it is
    # checked before anything else is even read.
    extrinsics = json.loads(paths['extrinsics'].read_text())
    if extrinsics.get('status') == 'EXPERIMENTAL-UNVERIFIED':
        raise RuntimeError(f"{paths['extrinsics']} is experimental; it cannot be used "
                           'for localisation')
    data = {key: json.loads(path.read_text()) for key, path in paths.items()
            if key != 'extrinsics'}
    data['extrinsics'] = extrinsics
    if data['table'].get('extrinsics') and \
            Path(data['table']['extrinsics']).name != paths['extrinsics'].name:
        raise RuntimeError('the table plane was measured with a different extrinsic')
    return Calibration(data['intrinsics'], data['extrinsics'], data['table'],
                       {key: str(path) for key, path in paths.items()})


def motion_status(calibration, settings, motion_check=None):
    """Why a result may or may not be used to move the arm.  Never silent."""
    reasons = []
    try:
        check_compatible(calibration.intrinsics, settings)
    except RuntimeError as error:
        reasons.append(str(error))
    if calibration.extrinsics.get('status') != 'solved':
        reasons.append(f"extrinsic status is {calibration.extrinsics.get('status')!r}")
    path = Path(motion_check or DEFAULTS['motion_check'])
    if not path.exists():
        reasons.append('no supervised motion check has passed for this extrinsic yet')
    else:
        record = json.loads(path.read_text())
        if record.get('extrinsics') != calibration.sources.get('extrinsics') or \
                not record.get('passed'):
            reasons.append('the recorded motion check does not cover this extrinsic')
    return {'usable_for_motion': not reasons, 'reasons': reasons}


def table_to_base(calibration, point_table):
    point = np.append(np.asarray(point_table, dtype=float), 1.0)
    return (calibration.T_base_table @ point)[:3]


class EnvironmentCamera:
    """Capture undistorted frames with a calibration that matches them."""

    def __init__(self, settings, calibration):
        check_compatible(calibration.intrinsics, settings)
        self.settings = settings
        self.calibration = calibration
        self.new_matrix, self.maps = calibration.undistortion()
        self.camera = RealCamera(settings)
        self.camera.warm_up(15)
        self.index = 0

    def frame(self):
        raw = self.camera.canonical_frame()
        undistorted = cv2.remap(raw, *self.maps, cv2.INTER_LINEAR)
        self.index += 1
        return {'raw': raw, 'undistorted': undistorted, 'index': self.index,
                'captured_at': timestamp(),
                'calibration': self.calibration.nexus_calibration(self.new_matrix)}

    def close(self):
        self.camera.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
