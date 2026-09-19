"""Export the calibrated real environment camera for the MuJoCo simulation.

The simulation's world frame has ``z = 0`` on the support surface and the
robot base standing on it.  The real calibration's matching frame is the
*table frame* of :func:`rgbcal.realcam.table_frame` (z = measured table
normal, origin on the table below the base origin, x = base x projected), so
the simulated camera is placed at ``T_table_camera``.  Then ``plane_z = 0``
means "on the table" in both worlds and the plane-assumption localisation
runs on the same geometry.

The intrinsics exported are the *undistorted* ``K_new`` the real pipeline
hands to perception: a simulated pinhole image is distortion free, so it is
the counterpart of the undistorted real frame, not of the raw one.

The file is plain JSON so the simulation (which has no OpenCV) can read it
without importing this package.
"""
from datetime import datetime
from pathlib import Path
import json
import math

import numpy as np

from . import realcam
from . import transforms as tf

SCHEMA = 'rgbcal/sim-camera/1'
DEFAULT_OUTPUT = realcam.CALIB / 'c920-sim-camera/sim_camera.json'


def _relative(path):
    path = Path(path).resolve()
    try:
        return str(path.relative_to(realcam.ROOT))
    except ValueError:
        return str(path)


def describe(calibration):
    """The simulated-camera record for a loaded :class:`realcam.Calibration`."""
    new_matrix, _ = calibration.undistortion()
    width, height = calibration.size
    fx, fy = float(new_matrix[0, 0]), float(new_matrix[1, 1])
    cx, cy = float(new_matrix[0, 2]), float(new_matrix[1, 2])
    pose = calibration.T_table_camera
    base = tf.invert(calibration.T_base_table)      # T_table_base
    forward = pose[:3, 2]
    return {
        'schema': SCHEMA,
        'exported_at': datetime.now().astimezone().isoformat(timespec='seconds'),
        'camera': calibration.intrinsics.get('device'),
        'frame': ('simulation world == real table frame: z = 0 is the measured table '
                  'surface, origin on the table below the robot base origin, x = base x '
                  'projected onto the table'),
        'image': {'width': width, 'height': height,
                  'content': 'undistorted frame (alpha = 0), the image the real RGB '
                             'pipeline gives to perception'},
        'K': new_matrix.tolist(),
        'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy,
        'pixel_convention': 'OpenCV: [0, 0] is the centre of the top-left pixel',
        'distortion': 'none (undistorted K_new; the raw lens K/D are not used here)',
        'fov_deg': {
            'horizontal': math.degrees(math.atan((cx + 0.5) / fx) +
                                       math.atan((width - 0.5 - cx) / fx)),
            'vertical': math.degrees(math.atan((cy + 0.5) / fy) +
                                     math.atan((height - 0.5 - cy) / fy))},
        'T_world_camera_opencv': pose.tolist(),
        'position': pose[:3, 3].tolist(),
        'rotation_opengl': tf.opencv_rotation_to_opengl(pose[:3, :3]).tolist(),
        'look_down_deg': math.degrees(math.asin(-forward[2])),
        'robot_base_in_world': {
            'T_world_base': base.tolist(),
            'height_m': float(base[2, 3]),
            'tilt_deg': math.degrees(math.acos(np.clip(base[2, 2], -1.0, 1.0))),
            'note': ('the simulation stands the base upright at the world origin; the '
                     'calibrated base frame sits this far above and tilted from the '
                     'table, largely absorbing joint-zero/extrinsic compensation, so the '
                     'robot as seen by the simulated camera differs from the real image '
                     'by about this much')},
        'usable_for_motion': False,
        'usable_for_motion_note': ('a simulation viewpoint only; it says nothing about '
                                   'whether real-arm motion from this camera is safe'),
        'sources': {key: _relative(path) for key, path in calibration.sources.items()},
    }


def export(output=None, intrinsics=None, extrinsics=None, table=None):
    calibration = realcam.load(intrinsics, extrinsics, table)
    record = describe(calibration)
    output = Path(output or DEFAULT_OUTPUT)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(record, indent=2) + '\n')
    return output, record
