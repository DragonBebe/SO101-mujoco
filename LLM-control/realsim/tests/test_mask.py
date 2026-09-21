"""The mask this package builds is the one ``perception`` thresholds.

``locate_color`` returns components but not their masks, so ``estimate``
repeats its saturation and value thresholds.  That duplication is only safe
while it stays equal, which is what these tests hold it to: if
``perception`` ever changes its threshold, this fails instead of the
estimator silently fitting a different region.
"""
import numpy as np

from nexus_vision import perception
from realsim import estimate
from realsim.tests.test_estimate import CUBE, draw
from realsim.tests.test_geometry import camera


def scene():
    calibration = camera()
    image = draw(calibration, [(CUBE, 0.24, -0.06, 0.1, 0.0),
                               (CUBE, 0.28, 0.07, 0.6, 0.0)])
    return calibration, image


def test_components_match_locate_color_exactly():
    calibration, image = scene()
    proposals = perception.locate_color(image, None, calibration, 'red')
    labels, count = estimate.colour_labels(image, 'red')
    assert count == len(proposals)
    for proposal in proposals:
        mask = estimate.component_mask(labels, proposal['pixel'])
        assert int(mask.sum()) == proposal['area']
        rows, columns = np.nonzero(mask)
        assert [int(columns.min()), int(rows.min()),
                int(columns.max()) + 1, int(rows.max()) + 1] == proposal['bbox']


def test_a_dim_or_washed_out_region_is_excluded_by_both():
    calibration = camera()
    image = draw(calibration, [(CUBE, 0.26, 0.0, 0.2, 0.0)])
    # A barely saturated pink patch: below perception's saturation floor.
    image[100:160, 100:160] = (255, 215, 215)
    proposals = perception.locate_color(image, None, calibration, 'red')
    labels, count = estimate.colour_labels(image, 'red')
    assert count == len(proposals) == 1
    assert not labels[100:160, 100:160].any()


def test_masks_of_different_colours_do_not_overlap():
    calibration = camera()
    from realsim.tests.test_estimate import BAR

    image = draw(calibration, [(CUBE, 0.24, -0.06, 0.1, 0.0),
                               (BAR, 0.29, 0.07, 0.6, 0.0)])
    red, _ = estimate.colour_labels(image, 'red')
    green, _ = estimate.colour_labels(image, 'green')
    assert red.any() and green.any()
    assert not np.any((red > 0) & (green > 0))


def test_a_pixel_outside_every_component_gives_an_empty_mask():
    calibration, image = scene()
    labels, _ = estimate.colour_labels(image, 'red')
    assert not estimate.component_mask(labels, [5, 5]).any()
