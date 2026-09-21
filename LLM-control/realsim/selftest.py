"""Rendered ground truth: what the estimator recovers when the answer is known.

This is a *synthetic* check and is labelled as one everywhere it appears.  It
renders blocks at poses MuJoCo was told to put them in, through the camera
the real calibration exports, and compares the estimate with those poses.
What it proves is that the geometry chain is right: the projection
convention, the table frame, the plane assumption, the yaw convention and its
symmetry, and the refusal paths.  What it cannot prove is anything about the
real camera's colours, lighting, blur or the accuracy of the calibration
itself -- a rendered frame shares the estimator's own model of the world.

Real accuracy needs blocks whose position was measured by something other
than this camera.  ``REALSIM_README.md`` says how; nothing here substitutes
for it.
"""
from pathlib import Path
import json
import math
import time

import mujoco
import numpy as np

from . import estimate, geometry, overlay, scene


def placements_grid(count, x_range=(0.18, 0.34), y_range=(-0.12, 0.12)):
    """Ground-truth poses spread over the workspace, corners included.

    An estimator that is only ever tested at the centre of the image is
    tested where the lens, the table fit and the viewing angle are all at
    their best.
    """
    rows = int(round(math.sqrt(max(count, 1))))
    columns = int(math.ceil(count / rows))
    xs = np.linspace(x_range[0], x_range[1], rows)
    ys = np.linspace(y_range[0], y_range[1], columns)
    poses = []
    for index, (x, y) in enumerate((x, y) for x in xs for y in ys):
        # A different yaw at every site, spread over the cube's symmetry, so
        # the angles tested are not all near zero.
        poses.append((float(x), float(y),
                      math.radians((index * 97) % 90 + 0.5)))
    return poses[:count]


def _visible(calibration, position):
    uv, depth = geometry.project_points(np.asarray(position), calibration)
    return bool(depth > 0 and 0 <= uv[0] < calibration['width'] and
                0 <= uv[1] < calibration['height'])


def run(catalogue, directory, placements=9, fit_settings=None, placement='calibrated',
        lifts=True, save_images=True):
    """Render, estimate, compare.  Returns the full report and writes it."""
    from nexus_vision.cameras import CameraSuite

    from .mirror import MirrorSimulation

    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    block = catalogue.blocks[0]
    other = catalogue.blocks[1] if len(catalogue.blocks) > 1 else None
    settings = fit_settings or estimate.FitSettings()
    cameras = CameraSuite(modality='rgb', placement=placement)
    simulation = MirrorSimulation(catalogue, directory / 'sim', cameras=cameras)
    rows, lift_rows, pair_rows = [], [], []
    try:
        for index, (x, y, yaw) in enumerate(placements_grid(placements), start=1):
            for entry in catalogue.blocks:
                simulation.park(entry.id)
            simulation.place(block.id, [x, y, block.half_extent[2]],
                             geometry.yaw_quaternion(yaw))
            mujoco.mj_forward(simulation.model, simulation.data)
            rgb, _, calibration = simulation.capture_cameras()[
                cameras.environment_camera]
            truth = simulation.data.qpos[
                simulation.slots[block.id].qpos_addr:
                simulation.slots[block.id].qpos_addr + 7].copy()
            started = time.perf_counter()
            candidates = estimate.estimate_block(rgb, calibration, block,
                                                 settings=settings)
            elapsed = time.perf_counter() - started
            row = {'index': index, 'truth_position_m': truth[:3].tolist(),
                   'truth_yaw_deg': math.degrees(geometry.canonical_yaw(
                       geometry.quaternion_yaw(truth[3:]), block.symmetry_step_rad)),
                   'in_frame': _visible(calibration, truth[:3]),
                   'estimate_s': round(elapsed, 3)}
            best = next((entry for entry in candidates if entry['valid']), None)
            if best is None:
                row.update({'detected': False,
                            'reasons': candidates[0]['reasons'] if candidates
                                       else ['no colour region of this block']})
            else:
                position = np.asarray(best['position'])
                row.update({
                    'detected': True,
                    'estimated_position_m': position.tolist(),
                    'estimated_yaw_deg': best['yaw_deg'],
                    'position_error_mm': float(
                        1000 * np.linalg.norm(position[:2] - truth[:2])),
                    'height_error_mm': float(1000 * (position[2] - truth[2])),
                    'yaw_error_deg': math.degrees(geometry.yaw_difference(
                        best['yaw_rad'], geometry.quaternion_yaw(truth[3:]),
                        block.symmetry_step_rad)),
                    'iou': best['quality']['iou'],
                    'area_ratio': best['quality']['area_ratio'],
                    'height_check': best['quality'].get('height_check')})
                if save_images and index <= 4:
                    picture = overlay.annotate(
                        rgb, calibration,
                        [dict(best, id=block.id, state='detected')],
                        header=[f'synthetic placement {index}',
                                f"truth {1000 * truth[0]:.0f}, {1000 * truth[1]:.0f} mm "
                                f"yaw {row['truth_yaw_deg']:.1f} deg"])
                    row['overlay'] = str(overlay.save(
                        picture, directory / 'frames' / f'placement-{index:02d}.png'))
            rows.append(row)

        if lifts:
            for elevation in (0.005, 0.010, 0.020, 0.040, 0.080):
                for entry in catalogue.blocks:
                    simulation.park(entry.id)
                simulation.place(block.id,
                                 [0.26, 0.0, elevation + block.half_extent[2]],
                                 geometry.yaw_quaternion(math.radians(20)))
                mujoco.mj_forward(simulation.model, simulation.data)
                rgb, _, calibration = simulation.capture_cameras()[
                    cameras.environment_camera]
                candidates = estimate.estimate_block(rgb, calibration, block,
                                                     settings=settings)
                accepted = [entry for entry in candidates if entry['valid']]
                check = candidates[0]['quality'].get('height_check') if candidates else None
                lift_rows.append({
                    'lifted_by_mm': 1000 * elevation,
                    'refused': not accepted,
                    'best_elevation_mm': (None if not check else
                                          1000 * check['best_elevation_m']),
                    'height_margin': None if not check else check['margin'],
                    'conclusion': None if not check else check['conclusion'],
                    'reasons': candidates[0]['reasons'] if candidates else
                               ['no colour region at all'],
                    'note': ('a block held above the table must not be reported as a '
                             'pose on the table')})

        if other is not None:
            for entry in catalogue.blocks:
                simulation.park(entry.id)
            # Two blocks only, whatever the catalogue's size: the point is
            # that two colours in one frame do not claim each other's pixels.
            truths = {block.id: (0.23, -0.08, math.radians(15)),
                      other.id: (0.29, 0.07, math.radians(62))}
            pair = [entry for entry in catalogue.blocks if entry.id in truths]
            for entry in pair:
                x, y, yaw = truths[entry.id]
                simulation.place(entry.id, [x, y, entry.half_extent[2]],
                                 geometry.yaw_quaternion(yaw))
            mujoco.mj_forward(simulation.model, simulation.data)
            rgb, _, calibration = simulation.capture_cameras()[
                cameras.environment_camera]
            for entry in pair:
                candidates = estimate.estimate_block(rgb, calibration, entry,
                                                     settings=settings)
                best = next((item for item in candidates if item['valid']), None)
                x, y, yaw = truths[entry.id]
                pair_rows.append({
                    'id': entry.id, 'detected': best is not None,
                    'position_error_mm': (None if best is None else float(
                        1000 * np.linalg.norm(np.asarray(best['position'][:2]) - [x, y]))),
                    'yaw_error_deg': (None if best is None else math.degrees(
                        geometry.yaw_difference(best['yaw_rad'], yaw,
                                                entry.symmetry_step_rad)))})
            if save_images:
                entries = []
                for entry in pair:
                    candidates = estimate.estimate_block(rgb, calibration, entry,
                                                         settings=settings)
                    best = next((item for item in candidates if item['valid']), None)
                    if best:
                        entries.append(dict(best, id=entry.id, state='detected'))
                overlay.save(overlay.annotate(rgb, calibration, entries,
                                              header=['synthetic two-block scene']),
                             directory / 'frames' / 'two-blocks.png')
    finally:
        simulation.close()

    detected = [row for row in rows if row.get('detected')]
    position_errors = [row['position_error_mm'] for row in detected]
    yaw_errors = [row['yaw_error_deg'] for row in detected]
    tolerances = catalogue.tolerances or {}
    position_limit = float(tolerances.get('position_mm', 5.0))
    yaw_limit = float(tolerances.get('yaw_deg', 10.0))
    summary = {
        'kind': 'SYNTHETIC: rendered frames, not photographs of the real table',
        'camera': placement,
        'placements': len(rows), 'detected': len(detected),
        'in_frame': sum(1 for row in rows if row['in_frame']),
        'position_error_mm': _statistics(position_errors),
        'yaw_error_deg': _statistics(yaw_errors),
        'height_error_mm': _statistics([abs(row['height_error_mm'])
                                        for row in detected]),
        'estimate_s': _statistics([row['estimate_s'] for row in rows]),
        'tolerances': {'position_mm': position_limit, 'yaw_deg': yaw_limit,
                       'source': str(tolerances.get('note', 'catalogue defaults'))},
        'lift_refusals': lift_rows,
        'lift_detection_floor_mm': _floor(lift_rows),
        'two_block_scene': pair_rows,
        'what_this_does_not_show': (
            'rendered images share the estimator\'s own model of the world: they say '
            'nothing about the real camera\'s colour, lighting or blur, and nothing '
            'about whether the calibration puts the table where the table is'),
    }
    summary['passed'] = bool(
        detected and len(detected) == sum(1 for row in rows if row['in_frame']) and
        summary['position_error_mm']['max'] <= position_limit and
        summary['yaw_error_deg']['max'] <= yaw_limit and
        all(row['refused'] for row in lift_rows if row['lifted_by_mm'] >= 40))
    report = {'schema': 'realsim/selftest/1', 'written_at': scene.timestamp(),
              'catalogue': catalogue.describe(), 'summary': summary,
              'placements': rows}
    (directory / 'selftest.json').write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    return report


def _floor(lift_rows):
    """The smallest lift this viewpoint actually refused, stated as measured.

    It is a property of the block size, the camera angle and the image scale,
    not a constant, so it is measured here rather than claimed.
    """
    refused = [row['lifted_by_mm'] for row in lift_rows if row['refused']]
    missed = [row['lifted_by_mm'] for row in lift_rows if not row['refused']]
    return {'smallest_refused_mm': min(refused) if refused else None,
            'largest_missed_mm': max(missed) if missed else None,
            'note': ('a block lifted by less than this is reported as a pose on the '
                     'table; the plane assumption cannot be checked below it from '
                     'this viewpoint')}


def _statistics(values):
    if not values:
        return {'count': 0}
    array = np.asarray(values, dtype=float)
    return {'count': int(array.size), 'median': float(np.median(array)),
            'mean': float(array.mean()), 'max': float(array.max()),
            'p90': float(np.percentile(array, 90))}
