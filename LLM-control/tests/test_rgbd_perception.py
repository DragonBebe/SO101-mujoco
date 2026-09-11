import sys
from pathlib import Path

import numpy as np
import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nexus_vision.perception import locate_color, project_world, unproject_pixel


def calibration(width=4, height=3, rotation=None):
    return {
        "width": width,
        "height": height,
        "fx": 2.0,
        "fy": 4.0,
        "cx": 1.0,
        "cy": 1.0,
        "position": [10.0, 20.0, 30.0],
        "rotation": np.eye(3) if rotation is None else rotation,
    }


def test_unproject_pixel_uses_optical_z_and_integer_pixel_centers():
    depth = np.ones((3, 4), dtype=float)
    depth[0, 3] = 2.0

    point = unproject_pixel(depth, calibration(), [3, 0])

    # Camera local: x=(3-1)*2/2=2, y=-(0-1)*2/4=0.5, z=-2.
    np.testing.assert_allclose(point, [12.0, 20.5, 28.0])


def test_camera_rotation_maps_local_point_into_world_coordinates():
    rotation = np.array(
        [
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    depth = np.ones((3, 4), dtype=float)
    depth[0, 3] = 2.0

    point = unproject_pixel(depth, calibration(rotation=rotation), [3.0, 0.0])

    np.testing.assert_allclose(point, [9.5, 22.0, 28.0])


def test_project_world_is_the_inverse_of_rotated_unprojection():
    rotation = np.array(
        [
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )

    projected = project_world([9.5, 22.0, 28.0], calibration(rotation=rotation))

    np.testing.assert_allclose(projected, [3.0, 0.0, 2.0])


@pytest.mark.parametrize("bad_depth", [0.0, -0.1, np.nan, np.inf])
def test_unproject_rejects_depth_that_cannot_describe_a_visible_surface(bad_depth):
    depth = np.ones((3, 4), dtype=float)
    depth[1, 1] = bad_depth

    with pytest.raises(ValueError, match="depth"):
        unproject_pixel(depth, calibration(), [1, 1])


@pytest.mark.parametrize(
    "pixel",
    [
        [-1, 0],
        [4, 0],
        [0, 3],
        [1.25, 1],
        [np.nan, 1],
        [1],
    ],
)
def test_unproject_rejects_invalid_pixel_coordinates(pixel):
    with pytest.raises(ValueError, match="pixel"):
        unproject_pixel(np.ones((3, 4)), calibration(), pixel)


@pytest.mark.parametrize(
    ("depth", "calibration_update"),
    [
        (np.ones((3, 4, 1)), {}),
        (np.ones((2, 4)), {}),
        (np.ones((3, 4)), {"fx": 0.0}),
        (np.ones((3, 4)), {"fx": np.array([2.0])}),
        (np.ones((3, 4)), {"position": [0.0, np.nan, 0.0]}),
        (np.ones((3, 4)), {"rotation": np.eye(2)}),
        (np.ones((3, 4), dtype=complex), {}),
    ],
)
def test_unproject_rejects_malformed_depth_or_calibration(depth, calibration_update):
    camera = calibration()
    camera.update(calibration_update)

    with pytest.raises(ValueError):
        unproject_pixel(depth, camera, [1, 1])


def test_project_world_rejects_points_behind_the_camera_or_nonfinite():
    with pytest.raises(ValueError, match="front"):
        project_world([10.0, 20.0, 31.0], calibration())
    with pytest.raises(ValueError, match="point"):
        project_world([10.0, np.nan, 29.0], calibration())


def test_locate_color_returns_components_without_green_distractor():
    rgb = np.zeros((8, 10, 3), dtype=np.uint8)
    rgb[1:3, 2:5] = [230, 30, 20]
    rgb[4:7, 6:9] = [20, 220, 40]
    depth = np.full((8, 10), 2.0)
    camera = {
        "width": 10,
        "height": 8,
        "fx": 10.0,
        "fy": 10.0,
        "cx": 4.5,
        "cy": 3.5,
        "position": [0.0, 0.0, 0.0],
        "rotation": np.eye(3),
    }

    proposals = locate_color(rgb, depth, camera, "red")

    assert proposals == [
        {
            "pixel": [3, 1],
            "bbox": [2, 1, 5, 3],
            "area": 6,
            "surface_world": pytest.approx([-0.3, 0.5, -2.0]),
        }
    ]


@pytest.mark.parametrize(
    ("color", "rgb_value"),
    [
        ("red", [230, 25, 25]),
        ("green", [25, 220, 45]),
        ("blue", [30, 60, 230]),
        ("yellow", [240, 220, 20]),
        ("orange", [240, 120, 20]),
    ],
)
def test_locate_color_supports_rendered_primary_color_proposals(color, rgb_value):
    rgb = np.zeros((3, 4, 3), dtype=np.uint8)
    rgb[1, 2] = rgb_value
    depth = np.full((3, 4), 1.5)

    proposals = locate_color(rgb, depth, calibration(), color)

    assert len(proposals) == 1
    assert proposals[0]["pixel"] == [2, 1]
    assert proposals[0]["bbox"] == [2, 1, 3, 2]
    assert proposals[0]["area"] == 1


def test_locate_color_uses_nearest_mask_pixel_with_valid_depth():
    rgb = np.zeros((5, 6, 3), dtype=np.uint8)
    rgb[1:4, 2:5] = [255, 0, 0]
    depth = np.full((5, 6), np.nan)
    depth[2, 4] = 2.0
    camera = calibration(width=6, height=5)

    proposals = locate_color(rgb, depth, camera, "red")

    assert proposals[0]["pixel"] == [4, 2]
    assert proposals[0]["area"] == 9
    np.testing.assert_allclose(proposals[0]["surface_world"], [13.0, 19.5, 28.0])


def test_locate_color_omits_components_without_valid_depth_and_handles_empty_mask():
    rgb = np.zeros((3, 4, 3), dtype=np.uint8)
    rgb[1, 2] = [255, 0, 0]
    depth = np.full((3, 4), np.nan)

    assert locate_color(rgb, depth, calibration(), "red") == []
    assert locate_color(np.zeros_like(rgb), np.ones((3, 4)), calibration(), "red") == []


def test_locate_color_validates_color_and_registered_array_shapes():
    rgb = np.zeros((3, 4, 3), dtype=np.uint8)
    depth = np.ones((3, 4))

    with pytest.raises(ValueError, match="color"):
        locate_color(rgb, depth, calibration(), "purple")
    with pytest.raises(ValueError, match="RGB"):
        locate_color(rgb[:, :, :2], depth, calibration(), "red")
    with pytest.raises(ValueError, match="registered"):
        locate_color(rgb, depth[:2], calibration(), "red")
