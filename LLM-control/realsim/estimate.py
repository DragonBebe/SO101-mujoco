"""Single-frame pose of a known block on a known plane, from one RGB image.

What is measured and what is assumed
------------------------------------
Assumed, and repeated in every result: the block is a rigid box of the size
the catalogue gives, it lies flat on a support plane whose height in the
table frame the caller names (0 for the table itself), and it is not badly
occluded.  Those assumptions supply the block's **centre height** and its
**roll and pitch**.  None of them is a camera measurement, and calling the
centre height a depth would be false -- it is the support height plus half a
known block.

Measured from the image, under those assumptions: **x**, **y** and **yaw** in
the plane's frame.  Three unknowns, and a silhouette that depends on all
three, which is why no depth sensor is needed here.

Method
------
``nexus_vision.perception`` already turns a frame into connected colour
components, and ``plane_support_estimate`` already carves a footprint out of
one component under exactly this plane assumption.  That carve is an *outer
bound* on the footprint -- all a silhouette can guarantee -- so it is kept as
the starting guess and as an independent screen, not as the answer: on a 25
mm cube it reports 34-38 mm of extent, because the bound is loose once the
cube is rotated.

The answer comes from fitting the box itself.  A box is convex, so the convex
hull of its eight projected corners is exactly its silhouette; a candidate
``(x, y, yaw)`` is scored by the intersection-over-union of that silhouette
with the observed colour mask.  A coarse sweep over yaw and position is
refined by Nelder-Mead on a sub-pixel-sampled score.  Because a box's
silhouette repeats with its own symmetry, the reported yaw is folded into
``[0, symmetry_step)``; see :func:`realsim.geometry.canonical_yaw`.

Where it must refuse
--------------------
A plane-assumption estimate is wrong by an unbounded amount exactly when the
assumption is: a lifted, toppled or stacked block still projects a plausible
outline.  Three checks guard that, and a failed one marks the estimate
invalid instead of shading its numbers:

``iou``
    whether a flat-lying box of this size explains the mask at all;
``area_ratio``
    observed mask area over model area; it falls when the block is occluded
    and rises when two colour regions merge;
``height_offset``
    the elevation above the assumed plane that best explains the silhouette,
    with x, y and yaw re-fitted at each height.  A block resting on the plane
    gives about zero.  How sharply a frame can tell depends on the viewing
    angle, so the result also carries the score's sensitivity to height and
    the check is reported rather than quietly trusted.
"""
from dataclasses import dataclass
import math

import numpy as np
from scipy import ndimage
from scipy.optimize import minimize

from nexus_vision import perception

from . import geometry

#: The same 4-connectivity ``locate_color`` labels with.  With a block's
#: default colour band, :func:`colour_labels` reproduces that function's
#: components pixel for pixel, and ``tests/test_mask.py`` holds it to that --
#: so a change in ``perception`` fails a test instead of silently drifting.
CONNECTIVITY = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]])


@dataclass(frozen=True)
class FitSettings:
    """Search extent and acceptance thresholds, in metres and radians."""

    #: Half-width of the coarse position sweep around the carve's centre.
    coarse_range_m: float = 0.012
    coarse_step_m: float = 0.003
    #: Coarse yaw samples across one symmetry period.
    yaw_samples: int = 24
    #: Sub-scan-lines per pixel row when scoring.
    subrows: int = 4
    #: Acceptance.
    min_iou: float = 0.70
    area_ratio_range: tuple = (0.75, 1.30)
    max_height_offset_m: float = 0.008
    #: How much better the best height must fit than the assumed plane before
    #: the check may refuse anything.  Measured on rendered 25 mm cubes at
    #: this viewpoint: a cube resting on the plane gains at most 0.020, and
    #: that worst case is a real degeneracy -- with the cube's faces square to
    #: the camera its silhouette is nearly a rectangle, which a raised cube
    #: reproduces almost exactly.  A 20 mm lift gains 0.048-0.053 and a 40 mm
    #: lift more.  0.025 therefore separates them, at the price of not
    #: refusing a 10 mm lift (0.021-0.022), which at this block size and
    #: viewing angle is genuinely not separable from a pose on the plane.
    #: ``run_realsim.sh selftest`` re-measures this floor and reports it.
    min_height_margin: float = 0.025
    #: A component smaller than this cannot support a fit.
    min_mask_px: int = 150
    #: Heights probed by the on-plane check, as multiples of the block height.
    height_probe_fractions: tuple = (-0.4, -0.2, 0.0, 0.2, 0.4, 0.8, 1.6)
    #: Skip the height check to make a live loop several times faster.
    check_height: bool = True
    #: Pixels to erode the colour mask by before fitting.  A colour threshold
    #: accepts partly-covered edge pixels, so a real blob carries a boundary
    #: layer of roughly half a pixel: measured on the archived C920 frames,
    #: eroding exactly one pixel moves the fitted edge of a 30 mm cube down by
    #: 0.75 mm in every frame, and takes the observed/model area ratio from
    #: 1.03-1.08 to 0.98-1.05.  The default is 0, so what is fitted is what
    #: the threshold returned; the bias is reported through ``area_ratio``
    #: rather than tuned away, and :func:`measure_size` brackets a size by
    #: fitting both ways.
    erode_px: int = 0
    #: When the tracker supplies the previous pose, the sweep only has to
    #: cover how far a block can have moved between frames, which is what
    #: makes continuous updates affordable.  A seed further than
    #: ``seed_gate_m`` from the fresh carve is ignored, so a block that was
    #: picked up and put down elsewhere is found by the full search.
    seed_range_m: float = 0.005
    seed_step_m: float = 0.0025
    seed_yaw_span_rad: float = 0.35
    seed_yaw_samples: int = 9
    seed_gate_m: float = 0.04


def colour_labels(rgb, colour):
    """Connected components matching one colour band.

    ``colour`` is a :class:`realsim.blocks.ColourSpec`, or a colour name for
    ``perception``'s own band.  This exists because ``locate_color`` returns
    components but not their *masks*, and because a real block's paint does
    not have to sit inside a band drawn for rendered colours.
    """
    from .blocks import default_colour_spec

    if isinstance(colour, str):
        colour = default_colour_spec(colour.strip().lower())
    image = perception._normalized_rgb(rgb)
    hue, saturation, value = perception._hsv_channels(image)
    mask = (saturation >= colour.min_saturation) & (value >= colour.min_value)
    hue_mask = np.zeros(mask.shape, dtype=bool)
    for low, high in colour.hue_ranges:
        hue_mask |= (hue >= low) & (hue < high)
    return ndimage.label(mask & hue_mask, structure=CONNECTIVITY)


def component_mask(labels, pixel):
    """The one component containing ``pixel``, as a boolean mask."""
    u, v = (int(value) for value in pixel)
    label = int(labels[v, u])
    if label == 0:
        return np.zeros(labels.shape, dtype=bool)
    return labels == label


def _eroded(mask, pixels):
    """Shrink a mask by whole pixels; ``0`` returns it untouched."""
    if not pixels:
        return mask
    return ndimage.binary_erosion(mask, iterations=int(pixels))


def component_regions(labels, count, min_area):
    """Each component's mask, bounding box and representative pixel.

    The representative is the component pixel nearest its centroid, the same
    choice ``locate_color`` makes, so a pixel quoted here means what it means
    everywhere else in this project.
    """
    regions = []
    for index in range(1, count + 1):
        mask = labels == index
        coordinates = np.argwhere(mask)
        if len(coordinates) < min_area:
            continue
        centroid = coordinates.mean(axis=0)
        nearest = coordinates[np.argmin(
            np.sum((coordinates - centroid) ** 2, axis=1))]
        rows, columns = coordinates[:, 0], coordinates[:, 1]
        regions.append({
            'mask': mask, 'area': int(len(coordinates)),
            'pixel': [int(nearest[1]), int(nearest[0])],
            'bbox': [int(columns.min()), int(rows.min()),
                     int(columns.max()) + 1, int(rows.max()) + 1]})
    regions.sort(key=lambda entry: (-entry['area'], entry['bbox'][1],
                                    entry['bbox'][0]))
    return regions


def fit_pose(scorer, calibration, half_extent, centre_z, initial_xy,
             symmetry_step, settings=None, seed_yaw=None):
    """Best ``(x, y, yaw)`` for a box whose centre height is already fixed.

    ``seed_yaw`` narrows the sweep to a neighbourhood of a previous estimate.
    It only ever changes how long the search takes, never what a converged
    fit means, and the refinement afterwards is identical either way.
    """
    settings = settings or FitSettings()

    def score(vector):
        outline = geometry.silhouette(
            half_extent, np.array([vector[0], vector[1], centre_z]), vector[2],
            calibration)
        if outline is None:
            return 0.0
        return scorer.iou(outline)[0]

    if seed_yaw is None:
        offsets = np.arange(-settings.coarse_range_m,
                            settings.coarse_range_m + 1e-9, settings.coarse_step_m)
        yaws = np.linspace(0.0, symmetry_step, settings.yaw_samples, endpoint=False)
    else:
        offsets = np.arange(-settings.seed_range_m,
                            settings.seed_range_m + 1e-9, settings.seed_step_m)
        yaws = seed_yaw + np.linspace(-settings.seed_yaw_span_rad,
                                      settings.seed_yaw_span_rad,
                                      settings.seed_yaw_samples)
    best, best_vector = -1.0, np.array([initial_xy[0], initial_xy[1], 0.0])
    for dx in offsets:
        for dy in offsets:
            for yaw in yaws:
                vector = np.array([initial_xy[0] + dx, initial_xy[1] + dy, yaw])
                value = score(vector)
                if value > best:
                    best, best_vector = value, vector
    # Nelder-Mead needs a simplex whose steps mean the same in both units:
    # metres and radians are two orders of magnitude apart, so it is built
    # explicitly from about 2 mm and about 2 degrees.
    step = np.array([0.002, 0.002, math.radians(2.0)])
    simplex = np.vstack([best_vector] +
                        [best_vector + step * unit for unit in np.eye(3)])
    result = minimize(lambda vector: -score(vector), best_vector,
                      method='Nelder-Mead',
                      options={'initial_simplex': simplex, 'xatol': 2e-5,
                               'fatol': 1e-6, 'maxiter': 600})
    vector = result.x if -result.fun >= best else best_vector
    outline = geometry.silhouette(half_extent,
                                  np.array([vector[0], vector[1], centre_z]),
                                  vector[2], calibration)
    iou, model_area = (0.0, 0.0) if outline is None else scorer.iou(outline)
    return {'x': float(vector[0]), 'y': float(vector[1]),
            'yaw': geometry.canonical_yaw(vector[2], symmetry_step),
            'centre_z': float(centre_z), 'iou': float(iou),
            'model_area_px': float(model_area),
            'mask_area_px': float(scorer.mask_area), 'outline': outline}


def height_consistency(scorer, calibration, half_extent, plane_z, initial_xy,
                       symmetry_step, block_height, settings=None):
    """Which elevation above the plane best explains this silhouette.

    The pose is re-fitted at every probed height, because a block held higher
    also appears displaced; holding x and y fixed would confuse the two.
    """
    settings = settings or FitSettings()
    probes = sorted({round(fraction * block_height, 6)
                     for fraction in settings.height_probe_fractions})
    scores = []
    for elevation in probes:
        centre_z = plane_z + elevation + half_extent[2]
        if centre_z <= 0.2 * half_extent[2]:
            continue
        fit = fit_pose(scorer, calibration, half_extent, centre_z, initial_xy,
                       symmetry_step, settings)
        scores.append({'elevation_m': float(elevation), 'iou': fit['iou']})
    if not scores:
        return None
    best = max(scores, key=lambda entry: entry['iou'])
    at_plane = min(scores, key=lambda entry: abs(entry['elevation_m']))
    higher = min((entry for entry in scores
                  if entry['elevation_m'] >= block_height * 0.7),
                 key=lambda entry: entry['elevation_m'], default=None)
    margin = float(best['iou'] - at_plane['iou'])
    off_plane = abs(best['elevation_m']) > settings.max_height_offset_m
    decisive = margin >= settings.min_height_margin
    if off_plane and decisive:
        conclusion = 'not on the assumed plane'
    elif decisive:
        conclusion = 'on the assumed plane'
    elif off_plane:
        conclusion = 'cannot tell from this viewpoint'
    else:
        conclusion = 'on the assumed plane'
    return {'best_elevation_m': best['elevation_m'], 'best_iou': best['iou'],
            'iou_at_assumed_plane': at_plane['iou'],
            'margin': margin, 'decisive': decisive, 'conclusion': conclusion,
            'iou_one_block_higher': higher['iou'] if higher else None,
            'sensitivity': (None if higher is None
                            else float(at_plane['iou'] - higher['iou'])),
            'probes': scores,
            'note': ('elevation is re-fitted together with x, y and yaw. The check '
                     'only refuses when the best height beats the assumed plane by '
                     'min_height_margin; below that the score landscape is flat and '
                     'this viewpoint genuinely cannot separate height from position')}


def _nearest_seed(seeds, centre_xy, gate):
    """Yaw of the closest previous pose within ``gate`` metres, if any."""
    best, best_distance = None, gate
    for seed in seeds or ():
        distance = float(np.hypot(seed['position'][0] - centre_xy[0],
                                  seed['position'][1] - centre_xy[1]))
        if distance <= best_distance:
            best, best_distance = float(seed['yaw_rad']), distance
    return best


def estimate_block(rgb, calibration, block, plane_z=0.0, settings=None,
                   expected_count=1, seeds=()):
    """Pose candidates for one catalogue block in one frame.

    ``calibration`` must describe the image given, expressed in the frame
    ``plane_z`` is measured in.  For the real camera that is
    ``realcam.Calibration.nexus_calibration(K_new, frame='table')``, so the
    results come out in the table frame -- which is also the simulation
    world.  Results are ordered best first; invalid ones are kept, with their
    reasons, because a rejected detection is evidence too.
    """
    settings = settings or FitSettings()
    labels, count = colour_labels(rgb, block.colour_spec)
    candidates = []
    for proposal in component_regions(labels, count, settings.min_mask_px):
        mask = _eroded(proposal['mask'], settings.erode_px)
        if not mask.any():
            continue
        # perception's own two-plane carve, called directly on this mask: an
        # outer bound on the footprint under the same plane assumption, used
        # as the starting guess and as an independent screen.
        support = perception.plane_support_estimate(mask, calibration, plane_z,
                                                    block.height_m)
        if not support:
            candidates.append({
                'block_id': block.id, 'valid': False,
                'reasons': ['no part of this colour region meets the assumed plane; '
                            'it is not where a block resting on it would appear'],
                'pixel': proposal['pixel'], 'bbox': proposal['bbox'],
                'quality': {'mask_area_px': float(proposal['area'])},
                'method': 'rgb_silhouette_fit_on_known_plane'})
            continue
        extent = support['extent']
        # The carve bounds the footprint from outside, so it screens out a
        # hand, an arm, a chair or two merged blocks *before* the fit is paid
        # for.  A region this far from the right size cannot become the right
        # answer, and fitting it anyway costs more than everything else in
        # this loop put together.
        if not 0.5 * block.diagonal_m <= max(extent) <= 1.8 * block.diagonal_m:
            candidates.append({
                'block_id': block.id, 'valid': False,
                'reasons': [f'carved footprint {1000 * extent[0]:.0f} x '
                            f'{1000 * extent[1]:.0f} mm is not one '
                            f'{1000 * block.size_m[0]:.0f} mm block seen from here; '
                            'the pose fit was not attempted'],
                'pixel': proposal['pixel'], 'bbox': proposal['bbox'],
                'quality': {'mask_area_px': float(proposal['area']),
                            'carved_footprint_m': [float(value) for value in extent]},
                'method': 'rgb_silhouette_fit_on_known_plane'})
            continue
        reasons = []
        scorer = geometry.MaskScorer(mask, settings.subrows)
        centre_z = float(plane_z) + float(block.half_extent[2])
        seed_yaw = _nearest_seed(seeds, support['center'][:2], settings.seed_gate_m)
        fit = fit_pose(scorer, calibration, block.half_extent, centre_z,
                       support['center'][:2], block.symmetry_step_rad, settings,
                       seed_yaw=seed_yaw)
        area_ratio = (fit['mask_area_px'] / fit['model_area_px']
                      if fit['model_area_px'] > 0 else 0.0)
        if fit['iou'] < settings.min_iou:
            reasons.append(f"silhouette fit iou {fit['iou']:.2f} is below "
                           f'{settings.min_iou:.2f}')
        low, high = settings.area_ratio_range
        if not low <= area_ratio <= high:
            reasons.append(
                f'observed/model silhouette area {area_ratio:.2f} is outside '
                f'[{low}, {high}]: occluded, merged with another region, or not '
                'this block')
        height = None
        if settings.check_height:
            height = height_consistency(
                scorer, calibration, block.half_extent, plane_z,
                (fit['x'], fit['y']), block.symmetry_step_rad, block.height_m,
                settings)
            if height and height['conclusion'] == 'not on the assumed plane':
                reason = ('the silhouette is best explained '
                          f"{1000 * height['best_elevation_m']:+.0f} mm from the "
                          f"assumed plane (margin {height['margin']:.3f}): the block "
                          'is not resting on it (lifted, stacked, or the wrong '
                          'support height was given)')
                # A block bigger than the catalogue says, and a block held
                # above the plane, both enlarge the silhouette: this check
                # cannot separate them, and saying which it saw would be a
                # guess.  The area ratio is the evidence, so it is quoted.
                if area_ratio > 1.02:
                    reason += (f'. The mask is also {100 * (area_ratio - 1):.0f}% '
                               'larger than the model, so a block larger than the '
                               f"catalogue's {1000 * block.size_m[0]:.1f} mm, or a "
                               'colour mask inflated by a bright edge, would look '
                               'the same from here')
                reasons.append(reason)
        candidates.append({
            'block_id': block.id,
            'valid': not reasons, 'reasons': reasons,
            'position': [fit['x'], fit['y'], fit['centre_z']],
            'yaw_rad': fit['yaw'], 'yaw_deg': math.degrees(fit['yaw']),
            'symmetry_step_deg': math.degrees(block.symmetry_step_rad),
            'quaternion_wxyz': geometry.yaw_quaternion(fit['yaw']).tolist(),
            'quality': {
                'iou': fit['iou'], 'area_ratio': area_ratio,
                'mask_area_px': fit['mask_area_px'],
                'model_area_px': fit['model_area_px'],
                'carved_footprint_m': [float(value) for value in extent],
                'carve_centre_m': [float(value) for value in support['center']],
                'height_check': height},
            'pixel': proposal['pixel'], 'bbox': proposal['bbox'],
            'outline_uv': (fit['outline'].tolist() if fit['outline'] is not None
                           else None),
            'measured': ['x', 'y', 'yaw'],
            'assumed': {
                'support_plane_z_m': float(plane_z),
                'block_height_m': block.height_m,
                'roll_pitch': 'zero: the block is assumed to lie flat on the plane',
                'centre_height_m': fit['centre_z'],
                'centre_height_origin': ('support plane + half the known block '
                                         'height; a consequence of known geometry, '
                                         'not a depth measurement')},
            'method': 'rgb_silhouette_fit_on_known_plane',
            'seeded': seed_yaw is not None,
        })
    candidates.sort(key=lambda entry: (not entry['valid'],
                                       -entry['quality'].get('iou', 0.0)))
    accepted = [entry for entry in candidates if entry['valid']]
    for extra in accepted[expected_count:]:
        extra['valid'] = False
        extra['reasons'].append(
            f'more {block.colour} regions fit than the catalogue has blocks of this '
            f'type ({expected_count}); the better-fitting ones were kept')
    return candidates


def measure_size(rgb, calibration, block, plane_z=0.0, settings=None,
                 scale_range=(0.6, 1.8)):
    """Fit the block's own size along with its pose, for one colour region.

    A block resting on a known plane and seen by a calibrated camera has an
    apparent size that depends on exactly one unknown once its pose is fitted:
    how big it is.  Fitting that as a free scale turns the camera into a
    (crude) measuring tool, which is useful for one thing -- catching a
    catalogue size that is wrong.

    It is not a substitute for a calliper.  The result inherits the metric
    scale of the whole calibration chain, so a board whose printed square is
    not the size it claims shifts every number here by the same factor.  Use
    it to decide whether to go and measure, and to check a measurement, not
    instead of one.
    """
    settings = settings or FitSettings()
    labels, count = colour_labels(rgb, block.colour_spec)
    results = []
    for proposal in component_regions(labels, count, settings.min_mask_px):
        mask = _eroded(proposal['mask'], settings.erode_px)
        if not mask.any():
            continue
        support = perception.plane_support_estimate(mask, calibration, plane_z,
                                                    block.height_m)
        if not support:
            continue
        scorer = geometry.MaskScorer(mask, settings.subrows)
        start = fit_pose(scorer, calibration, block.half_extent,
                         plane_z + block.half_extent[2],
                         support['center'][:2],
                         block.symmetry_step_rad, settings)

        def score(vector):
            scale = float(np.clip(vector[3], *scale_range))
            half = block.half_extent * scale
            centre = np.array([vector[0], vector[1], plane_z + half[2]])
            outline = geometry.silhouette(half, centre, vector[2], calibration)
            if outline is None:
                return 0.0
            return scorer.iou(outline)[0]

        start_vector = np.array([start['x'], start['y'], start['yaw'], 1.0])
        step = np.array([0.002, 0.002, math.radians(2.0), 0.03])
        simplex = np.vstack([start_vector] +
                            [start_vector + step * unit for unit in np.eye(4)])
        result = minimize(lambda vector: -score(vector), start_vector,
                          method='Nelder-Mead',
                          options={'initial_simplex': simplex, 'xatol': 2e-5,
                                   'fatol': 1e-6, 'maxiter': 900})
        scale = float(np.clip(result.x[3], *scale_range))
        # The same fit on a mask eroded by one pixel.  The truth is between
        # the two: the colour threshold's boundary layer is real but is not a
        # whole pixel wide, and it is the largest error in this measurement.
        tight = _fit_scale(_eroded(proposal['mask'], settings.erode_px + 1),
                           calibration, block, plane_z, settings, scale_range)
        results.append({
            'block_id': block.id,
            'pixel': proposal['pixel'], 'bbox': proposal['bbox'],
            'catalogue_size_mm': [1000 * value for value in block.size_m],
            'fitted_scale': scale,
            'fitted_size_mm': [1000 * value * scale for value in block.size_m],
            'size_bracket_mm': (
                sorted([1000 * block.size_m[0] * scale,
                        1000 * block.size_m[0] * tight])
                if tight else None),
            'bracket_note': ('the two ends are the fit on the colour mask as '
                             'thresholded and on the same mask eroded by one pixel; '
                             'the block is between them, nearer the middle'),
            'iou_at_catalogue_size': start['iou'],
            'iou_at_fitted_size': float(-result.fun),
            'position_m': [float(result.x[0]), float(result.x[1]),
                           float(plane_z + block.half_extent[2] * scale)],
            'yaw_deg': math.degrees(geometry.canonical_yaw(result.x[2],
                                                           block.symmetry_step_rad)),
            'note': ('the size inherits the metric scale of the intrinsics board, '
                     'the extrinsic and the table fit; it is a cross-check on the '
                     'catalogue, not a replacement for measuring the block'),
        })
    results.sort(key=lambda entry: -entry['iou_at_fitted_size'])
    return results


def _fit_scale(mask, calibration, block, plane_z, settings, scale_range):
    """Best size scale for one mask, or ``None`` if it cannot be fitted."""
    if not mask.any():
        return None
    support = perception.plane_support_estimate(mask, calibration, plane_z,
                                                block.height_m)
    if not support:
        return None
    scorer = geometry.MaskScorer(mask, settings.subrows)
    start = fit_pose(scorer, calibration, block.half_extent,
                     plane_z + block.half_extent[2], support['center'][:2],
                     block.symmetry_step_rad, settings)

    def score(vector):
        scale = float(np.clip(vector[3], *scale_range))
        half = block.half_extent * scale
        outline = geometry.silhouette(
            half, np.array([vector[0], vector[1], plane_z + half[2]]), vector[2],
            calibration)
        return 0.0 if outline is None else scorer.iou(outline)[0]

    start_vector = np.array([start['x'], start['y'], start['yaw'], 1.0])
    step = np.array([0.002, 0.002, math.radians(2.0), 0.03])
    simplex = np.vstack([start_vector] +
                        [start_vector + step * unit for unit in np.eye(4)])
    result = minimize(lambda vector: -score(vector), start_vector,
                      method='Nelder-Mead',
                      options={'initial_simplex': simplex, 'xatol': 2e-5,
                               'fatol': 1e-6, 'maxiter': 900})
    return float(np.clip(result.x[3], *scale_range))


def colour_report(rgb, block, min_area=400):
    """How well this block's colour band actually covers this block.

    The failure it exists to catch is silent: a band that clips one shaded
    face still produces a confident-looking region, just a smaller one, and
    the pose fitted to it is wrong in a way the picture does not show.  So
    this reports where the block's pixels really sit in hue, saturation and
    value, and how close they come to the edges of the band being used.
    """
    from .blocks import ColourSpec

    spec = block.colour_spec
    image = perception._normalized_rgb(rgb)
    hue, saturation, value = perception._hsv_channels(image)
    labels, count = colour_labels(rgb, spec)
    regions = component_regions(labels, count, min_area)
    rows = []
    for region in regions[:4]:
        mask = region['mask']
        # Pixels just outside the region that the saturation/value floors
        # accept: if the band is clipping the block, they are sitting there.
        grown = ndimage.binary_dilation(mask, iterations=6) & ~mask
        grown &= (saturation >= spec.min_saturation) & (value >= spec.min_value)
        inside_hue = hue[mask]
        margins = []
        for low, high in spec.hue_ranges:
            within = (inside_hue >= low) & (inside_hue < high)
            if within.any():
                margins.append({'band_deg': [low, high],
                                'closest_to_low_deg': float(inside_hue[within].min() - low),
                                'closest_to_high_deg': float(high - inside_hue[within].max())})
        rows.append({
            'bbox': region['bbox'], 'area_px': region['area'],
            'hue_deg': {'p5': float(np.percentile(inside_hue, 5)),
                        'median': float(np.median(inside_hue)),
                        'p95': float(np.percentile(inside_hue, 95))},
            'saturation': {'p5': float(np.percentile(saturation[mask], 5)),
                           'median': float(np.median(saturation[mask]))},
            'value': {'p5': float(np.percentile(value[mask], 5)),
                      'median': float(np.median(value[mask]))},
            'band_margins_deg': margins,
            'neighbouring_saturated_px': int(grown.sum()),
            'neighbouring_hue_median_deg': (float(np.median(hue[grown]))
                                            if grown.any() else None),
        })
    return {'block_id': block.id, 'colour_spec': spec.describe(),
            'regions': rows,
            'how_to_read': (
                'band_margins_deg says how far this block\'s pixels stop short of '
                'the band edges. A margin near zero means the band is cutting the '
                'block, and neighbouring_hue_median_deg says where the cut-off '
                'pixels went. Widen hue_deg to cover them, but never so far that '
                'it overlaps another block\'s band.')}
