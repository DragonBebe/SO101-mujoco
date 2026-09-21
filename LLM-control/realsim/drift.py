"""Has the camera moved since the calibration was locked, and by how much.

``rgbcal check-setup`` answers a narrower question: it correlates saved
anchor patches and asks whether they agree on one image *translation*.  That
is the right test for a nudge, but a camera that rotates, or moves toward the
scene, shifts different parts of the image by different amounts, so no single
translation fits and the tool reports that it cannot tell -- which reads like
"no information" when it is in fact a symptom.

This answers the question the mapping actually needs: the rigid motion
between the locked reference frame and one taken now, from feature
correspondences and the measured intrinsics.  What it gives:

displacement
    How far matched scene points moved, in pixels.  This is the robust
    number and it is what the verdict rests on: the calibration's own rule is
    that more than 3 pixels means the extrinsic has to be re-measured, and at
    this focal length 0.1 degrees of camera rotation is already 2.4 pixels.

rotation
    Exact from noiseless correspondences, but a two-view essential matrix is
    weakly conditioned when the camera's baseline is small next to the scene
    depth, which is the usual case for a nudged tripod.  Measured on
    synthetic scenes, 0.1-0.6 pixels of feature noise moves a 2 degree
    estimate by up to 0.6 degrees.  So it is reported as an order of
    magnitude, never as the acceptance criterion.

translation
    Direction only.  Two views cannot fix its length without a known length
    in the scene, so none is claimed.

Two further degradations are reported rather than hidden: a scene where much
has genuinely moved (people, chairs, the blocks themselves) lowers the inlier
ratio, and a motion that is nearly a pure rotation about the camera centre
makes the essential matrix ill-conditioned, which shows up as a homography
fitting the same matches just as well.  With *no* motion at all the essential
matrix is entirely degenerate, so below the displacement limit no rotation is
reported at all rather than the arbitrary one it would otherwise produce.
"""
from pathlib import Path

import cv2
import numpy as np

from . import DISPLACEMENT_LIMIT_PX          # noqa: F401  (re-exported here)


def correspondences(reference, current, features=6000, ratio=0.75):
    """SIFT matches between two frames, with the ratio test applied."""
    grey = [cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) for image in (reference, current)]
    sift = cv2.SIFT.create(nfeatures=features)
    found = [sift.detectAndCompute(image, None) for image in grey]
    if any(descriptors is None for _, descriptors in found):
        raise RuntimeError('one of the frames has no features to match')
    pairs = cv2.BFMatcher().knnMatch(found[0][1], found[1][1], k=2)
    good = [first for first, second in pairs if first.distance < ratio * second.distance]
    if len(good) < 12:
        raise RuntimeError(f'only {len(good)} matches between the two frames; they do '
                           'not show the same scene')
    return (np.float32([found[0][0][match.queryIdx].pt for match in good]).reshape(-1, 1, 2),
            np.float32([found[1][0][match.trainIdx].pt for match in good]).reshape(-1, 1, 2))


def motion(points_reference, points_current, K, D, threshold_px=1.5,
           limit_px=None):
    """Rigid camera motion between two views, from matched image points.

    Points are undistorted with the measured ``K``/``D`` first, so the result
    is in the same geometry everything else in this project uses.
    """
    K = np.asarray(K, dtype=float)
    normalised = [cv2.undistortPoints(points, K, np.asarray(D, dtype=float))
                  for points in (points_reference, points_current)]
    identity = np.eye(3)
    essential, mask = cv2.findEssentialMat(
        normalised[0], normalised[1], identity, method=cv2.RANSAC, prob=0.9999,
        threshold=threshold_px / K[0, 0])
    if essential is None or mask is None:
        raise RuntimeError('no consistent camera motion fits these matches')
    _, rotation, translation, _ = cv2.recoverPose(
        essential, normalised[0], normalised[1], identity, mask=mask.copy())
    angle = float(np.degrees(np.arccos(
        np.clip((np.trace(rotation) - 1) / 2, -1.0, 1.0))))
    vector = np.degrees(cv2.Rodrigues(rotation)[0].reshape(3))
    inliers = mask.ravel() == 1
    displacement = (points_current - points_reference).reshape(-1, 2)[inliers]
    median = np.median(np.linalg.norm(displacement, axis=1))

    # Does a single homography explain the same matches?  If it does, the
    # motion is close to a pure rotation and the translation is unreliable.
    undistorted = [cv2.undistortPoints(points, K, np.asarray(D, dtype=float), P=K)
                   for points in (points_reference, points_current)]
    homography, homography_mask = cv2.findHomography(
        undistorted[0], undistorted[1], cv2.RANSAC, 3.0, maxIters=20000,
        confidence=0.9999)
    planar = 0.0
    if homography is not None and homography_mask is not None:
        planar = float(homography_mask.sum()) / len(undistorted[0])

    per_pixel = float(np.degrees(1.0 / K[0, 0]))       # degrees per pixel
    # With no real motion there is no baseline, the essential matrix is
    # degenerate, and recoverPose returns an arbitrary rotation -- 180 degrees
    # as often as not.  Below the limit the honest answer is that there is no
    # motion to decompose, not a number.
    limit = DISPLACEMENT_LIMIT_PX if limit_px is None else float(limit_px)
    determined = bool(median > limit)
    return {
        'matches': int(len(points_reference)),
        'inliers': int(inliers.sum()),
        'inlier_fraction': float(inliers.mean()),
        'rotation_deg': angle if determined else None,
        'rotation_determined': determined,
        'rotation_note': (
            'good to a few tenths of a degree: a two-view essential matrix is '
            'weakly conditioned when the baseline is small next to the scene '
            'depth. The verdict uses the displacement, not this'
            if determined else
            'not reported: the displacement is below the limit, so there is no '
            'baseline and the two-view geometry is degenerate'),
        'rotation_vector_deg': ({'pitch_x': float(vector[0]),
                                 'yaw_y': float(vector[1]),
                                 'roll_z': float(vector[2])} if determined else None),
        'translation_direction': ([float(value) for value in translation.reshape(3)]
                                  if determined else None),
        'translation_note': ('direction only, in the reference camera frame; two '
                             'views cannot give its length without a known length '
                             'in the scene'),
        'median_displacement_px': float(median),
        'displacement_median_xy_px': np.median(displacement, axis=0).tolist(),
        'degrees_per_pixel': per_pixel,
        'homography_inlier_fraction': planar,
        'homography_note': ('a high fraction means one plane, or a near-pure '
                            'rotation, explains the matches; the rotation stays '
                            'usable but the translation direction does not'),
    }


def report(reference_path, current, calibration, limit_px=DISPLACEMENT_LIMIT_PX):
    """Compare a locked reference frame with one taken now, and judge it."""
    reference_path = Path(reference_path)
    reference = cv2.imread(str(reference_path))
    if reference is None:
        raise RuntimeError(f'cannot read the reference frame {reference_path}')
    if reference.shape != current.shape:
        raise RuntimeError(
            f'the reference frame is {reference.shape[1]}x{reference.shape[0]} and '
            f'this capture is {current.shape[1]}x{current.shape[0]}; they are not '
            'comparable')
    points = correspondences(reference, current)
    result = motion(points[0], points[1], calibration.K, calibration.D,
                    limit_px=limit_px)
    moved = result['median_displacement_px'] > limit_px
    result.update({
        'reference': str(reference_path),
        'limit_px': float(limit_px),
        'camera_moved': bool(moved),
        'verdict': (
            f"the camera has moved: matched scene points are a median "
            f"{result['median_displacement_px']:.0f} px apart, of the order of "
            f"{result['rotation_deg']:.1f} degrees of rotation, against a "
            f'{limit_px:.0f} px limit. The extrinsic, the table plane and the '
            'sim-camera export all have to be redone'
            if moved else
            f"no motion above the {limit_px:.0f} px limit: median displacement "
            f"{result['median_displacement_px']:.1f} px"),
        'what_to_redo': ([
            'run_rgbcal.sh lock (a new setup session for the current viewpoint)',
            'run_rgbcal.sh handeye-collect / handeye-solve / handeye-verify',
            'run_rgbcal.sh table-plane',
            'run_rgbcal.sh sim-camera',
            'run_realsim.sh scale-check, until the error is inside the tolerance',
        ] if moved else []),
        'caveats': ('feature matching counts everything that moved, including '
                    'people, chairs and the blocks; a low inlier fraction means '
                    'the scene changed a lot, not that the camera did not move'),
    })
    return result
