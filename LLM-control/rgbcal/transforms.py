"""Rigid transforms, named so the direction cannot be misread.

Every transform in this project is written ``T_a_b`` and means: takes
coordinates expressed in frame ``b`` and returns them in frame ``a``.  So
``T_base_camera @ p_camera == p_base``, and the camera's own origin in base
coordinates is ``T_base_camera[:3, 3]``.

Two camera axis conventions appear and they are not the same:

``opencv``
    x right, y down, z forward along the optical axis.  This is what
    ``solvePnP``, ``projectPoints`` and every intrinsic in this package use.

``opengl``
    x right, y up, z backwards.  This is what MuJoCo's renderer reports and
    what ``nexus_vision.perception`` expects in its ``rotation`` field.

They differ by a 180 degree turn about x, so a rotation converted between
them is *not* a relabelling of axes that can be skipped.
"""
import numpy as np

#: opencv <-> opengl camera axes.  Its own inverse.
CV_TO_GL = np.diag([1.0, -1.0, -1.0])


def transform(rotation, translation):
    matrix = np.eye(4)
    matrix[:3, :3] = np.asarray(rotation, dtype=float).reshape(3, 3)
    matrix[:3, 3] = np.asarray(translation, dtype=float).reshape(3)
    return matrix


def invert(matrix):
    rotation = matrix[:3, :3]
    result = np.eye(4)
    result[:3, :3] = rotation.T
    result[:3, 3] = -rotation.T @ matrix[:3, 3]
    return result


def from_rvec_tvec(rvec, tvec):
    import cv2

    rotation, _ = cv2.Rodrigues(np.asarray(rvec, dtype=float).reshape(3, 1))
    return transform(rotation, np.asarray(tvec, dtype=float).reshape(3))


def to_rvec_tvec(matrix):
    import cv2

    rvec, _ = cv2.Rodrigues(matrix[:3, :3])
    return rvec.reshape(3), matrix[:3, 3].copy()


def rotation_angle(matrix):
    """Rotation magnitude of a transform, in radians."""
    trace = np.clip((np.trace(matrix[:3, :3]) - 1) / 2, -1.0, 1.0)
    return float(np.arccos(trace))


def screw(matrix):
    """Rotation angle and translation along the rotation axis.

    ``A`` and ``B`` in ``A X = X B`` are conjugate, ``A = X B inv(X)``, and
    conjugation preserves both numbers.  The angle alone does not: negating
    every joint of a serial arm can leave the angles untouched while the
    screw translation changes, which is exactly the mirror-image case that
    a rotation-only check cannot see.
    """
    angle = rotation_angle(matrix)
    if angle < 1e-9:
        return angle, float(np.linalg.norm(matrix[:3, 3]))
    import cv2

    axis = cv2.Rodrigues(matrix[:3, :3])[0].reshape(3)
    axis = axis / max(np.linalg.norm(axis), 1e-12)
    return angle, float(axis @ matrix[:3, 3])


def log_se3(matrix):
    """6-vector (rotation then translation) used as an optimisation residual."""
    import cv2

    rvec, _ = cv2.Rodrigues(matrix[:3, :3])
    return np.concatenate([rvec.reshape(3), matrix[:3, 3]])


def exp_se3(vector):
    """Inverse of :func:`log_se3` for the parameterisation used here."""
    return from_rvec_tvec(vector[:3], vector[3:])


def opencv_rotation_to_opengl(rotation):
    """Convert a camera rotation from OpenCV axes to OpenGL axes.

    ``rotation`` maps OpenCV camera coordinates into some world frame; the
    result maps OpenGL camera coordinates into that same world frame, which
    is the form ``nexus_vision.perception`` consumes.
    """
    return np.asarray(rotation, dtype=float) @ CV_TO_GL


def opengl_rotation_to_opencv(rotation):
    return np.asarray(rotation, dtype=float) @ CV_TO_GL
