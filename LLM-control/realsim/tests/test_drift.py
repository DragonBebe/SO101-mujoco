"""Recovering camera motion, checked against motions we chose ourselves.

Synthetic 3-D points projected into two cameras with a known rigid motion:
no images, so this tests the geometry rather than SIFT's taste in corners.
"""
import math

import numpy as np
import pytest

pytest.importorskip('cv2')

import cv2                                                      # noqa: E402

from realsim import drift                                       # noqa: E402

K = np.array([[1380.0, 0.0, 959.5], [0.0, 1380.0, 539.5], [0.0, 0.0, 1.0]])
D = np.zeros(5)


def scene(count=300, seed=7):
    """Points spread in depth, so the motion is not a degenerate one plane."""
    rng = np.random.default_rng(seed)
    return np.column_stack([rng.uniform(-0.6, 0.6, count),
                            rng.uniform(-0.4, 0.4, count),
                            rng.uniform(0.8, 2.5, count)])


def views(rotation_deg_vector, translation, noise_px=0.0, seed=3):
    points = scene()
    rotation = cv2.Rodrigues(np.radians(np.asarray(rotation_deg_vector,
                                                   dtype=float)))[0]
    moved = (points - np.asarray(translation)) @ rotation.T

    def project(p):
        uv = (p / p[:, 2:3]) @ K.T
        return uv[:, :2]

    first, second = project(points), project(moved)
    if noise_px:
        rng = np.random.default_rng(seed)
        first = first + rng.normal(0, noise_px, first.shape)
        second = second + rng.normal(0, noise_px, second.shape)
    keep = np.all((second > 0) & (second < [1920, 1080]), axis=1)
    keep &= np.all((first > 0) & (first < [1920, 1080]), axis=1)
    return (first[keep].reshape(-1, 1, 2).astype(np.float32),
            second[keep].reshape(-1, 1, 2).astype(np.float32))


def test_a_known_rotation_comes_back_exactly_without_noise():
    first, second = views([0.0, -2.0, 0.0], [0.05, 0.0, 0.02])
    result = drift.motion(first, second, K, D)
    assert result['rotation_deg'] == pytest.approx(2.0, abs=0.01)
    assert result['rotation_vector_deg']['yaw_y'] == pytest.approx(-2.0, abs=0.01)


def test_feature_noise_moves_the_rotation_by_tenths_of_a_degree():
    """Why the verdict is based on displacement and not on this angle.

    A small baseline next to the scene depth weakly conditions the essential
    matrix, so sub-pixel feature noise is worth a few tenths of a degree.
    """
    angles = [drift.motion(*views([0.0, -2.0, 0.0], [0.05, 0.0, 0.02],
                                  noise_px=noise, seed=seed), K, D)['rotation_deg']
              for noise, seed in ((0.1, 1), (0.3, 2), (0.6, 3))]
    assert all(abs(angle - 2.0) < 1.0 for angle in angles)
    assert max(angles) - min(angles) > 0.2


def test_a_camera_that_did_not_move_claims_no_rotation_at_all():
    """With no baseline the essential matrix is degenerate; it must not guess.

    Left to itself ``recoverPose`` answers 180 degrees here as readily as 0.
    """
    first, second = views([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], noise_px=0.2)
    result = drift.motion(first, second, K, D)
    assert result['median_displacement_px'] < drift.DISPLACEMENT_LIMIT_PX
    assert result['rotation_determined'] is False
    assert result['rotation_deg'] is None
    assert result['rotation_vector_deg'] is None
    assert 'degenerate' in result['rotation_note']


def test_displacement_is_what_the_limit_is_judged_on():
    """A 0.2 degree rotation is small, and still five pixels at this focal length."""
    first, second = views([0.0, -0.2, 0.0], [0.0, 0.0, 0.0], noise_px=0.2)
    result = drift.motion(first, second, K, D)
    assert result['rotation_determined'] is True
    expected = math.radians(0.2) * K[0, 0]
    assert result['median_displacement_px'] == pytest.approx(expected, rel=0.25)
    assert result['median_displacement_px'] > drift.DISPLACEMENT_LIMIT_PX
    assert result['degrees_per_pixel'] == pytest.approx(
        math.degrees(1 / K[0, 0]), rel=1e-6)


def test_a_near_pure_rotation_is_flagged_by_the_homography_fraction():
    rotated = views([0.0, -1.5, 0.0], [0.0, 0.0, 0.0], noise_px=0.2)
    translated = views([0.0, -1.5, 0.0], [0.10, 0.0, 0.05], noise_px=0.2)
    assert (drift.motion(*rotated, K, D)['homography_inlier_fraction'] >
            drift.motion(*translated, K, D)['homography_inlier_fraction'])


def test_frames_of_different_sizes_are_refused(tmp_path):
    from rgbcal import realcam

    reference = tmp_path / 'reference.png'
    cv2.imwrite(str(reference), np.zeros((100, 200, 3), dtype=np.uint8))

    class Calibration:
        K, D = globals()['K'], globals()['D']

    with pytest.raises(RuntimeError, match='not\\s+comparable|comparable'):
        drift.report(reference, np.zeros((50, 80, 3), dtype=np.uint8), Calibration())


def test_a_missing_reference_is_named():
    class Calibration:
        K, D = globals()['K'], globals()['D']

    with pytest.raises(RuntimeError, match='cannot read the reference'):
        drift.report('/nonexistent/reference.png',
                     np.zeros((10, 10, 3), dtype=np.uint8), Calibration())
