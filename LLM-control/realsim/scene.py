"""The scene state: the one artefact that crosses from real to simulated.

Two virtual environments split this project -- OpenCV and the real camera on
one side, MuJoCo and ``so101_nexus`` on the other -- so the mapping is not a
function call but a file, exactly as ``rgbcal sim-camera`` already hands the
simulation its camera.  This module owns that file's shape, in one place, so
the two sides cannot drift apart.

Frames, written ``T_a_b`` for "takes coordinates in ``b``, returns them in
``a``", the same convention as :mod:`rgbcal.transforms`:

``table``
    The measured table.  ``z = 0`` is the table surface, the origin is the
    point of the table below the robot base origin, ``x`` is the base's x
    projected onto the table.  **This is the simulation world**, which is why
    a pose needs no conversion to be mirrored.
``base``
    The robot base the hand-eye calibration solved for.  ``T_base_table``
    comes from the measured plane; poses are reported in it as well, because
    that is the frame an arm command is written in.
``camera``
    OpenCV axes (x right, y down, z forward), the frame the intrinsics and
    the extrinsic are expressed in.
``object``
    The block's own frame: axis-aligned box, origin at its geometric centre,
    which is also the origin of the MuJoCo free-joint body that mirrors it.

The chain a block's pose travels is
``T_table_object = T_table_camera x T_camera_object``, and the base-frame
copy is ``T_base_object = T_base_table x T_table_object``.  Units are metres
and radians throughout; quaternions are ``[w, x, y, z]``, MuJoCo's order.
"""
from datetime import datetime, timezone
from pathlib import Path
import json
import math

SCHEMA = 'realsim/scene/1'
FRAMES = {
    'reference': 'table',
    'table': ('z = 0 is the measured table surface; origin on the table below the '
              'robot base origin; x is the base x axis projected onto the table. '
              'This frame is the MuJoCo world of the mirror.'),
    'base': 'robot base frame of the hand-eye extrinsic',
    'object': ('axis-aligned box, origin at its geometric centre, which is the '
               'origin of the mirroring MuJoCo free-joint body'),
    'convention': ('T_a_b maps coordinates in b to a; T_table_object = '
                   'T_table_camera x T_camera_object'),
    'units': 'metres, radians; quaternions are [w, x, y, z] (MuJoCo order)',
}


def timestamp():
    return datetime.now(timezone.utc).astimezone().isoformat(timespec='milliseconds')


def _pose(position, yaw, symmetry_step):
    from . import geometry

    return {'position_m': [float(value) for value in position],
            'quaternion_wxyz': geometry.yaw_quaternion(yaw).tolist(),
            'yaw_rad': float(yaw), 'yaw_deg': math.degrees(float(yaw)),
            'symmetry_step_deg': math.degrees(symmetry_step),
            'yaw_note': ('folded into [0, symmetry_step); the block model cannot '
                         'distinguish poses a whole step apart')}


def object_record(track, now, settings, to_base=None):
    """One block's entry: its state, its pose, and when that pose was measured."""
    block = track.block
    state, age = track.state(now, settings)
    entry = {'id': block.id, 'block': block.describe(), 'state': state,
             'observations': track.observations, 'misses_since_seen': track.misses}
    if track.last_valid is None:
        entry.update({'pose': None, 'reason': 'never detected in this session'})
        return entry
    candidate = track.last_valid
    entry['observed_at'] = candidate.get('observed_at', '')
    entry['age_s'] = None if age is None else round(float(age), 3)
    entry['fresh'] = state == 'detected'
    entry['source_image'] = candidate.get('source_image')
    entry['frame_index'] = candidate.get('frame_index')
    entry['pose'] = _pose(candidate['position'], candidate['yaw_rad'],
                          block.symmetry_step_rad)
    entry['pose_filtered'] = _pose(track.filtered_position, track.filtered_yaw,
                                   block.symmetry_step_rad)
    entry['filter'] = {'position_alpha': settings.position_alpha,
                       'yaw_alpha': settings.yaw_alpha,
                       'reset_on_this_frame': candidate.get('filter_reset', False),
                       'note': 'pose is the raw single-frame fit; pose_filtered is '
                               'smoothed. Both are logged so the filter can be undone'}
    if to_base is not None:
        entry['pose_base'] = {
            'position_m': to_base(candidate['position']).tolist(),
            'note': ('the same pose expressed in the robot base frame via '
                     'T_base_table; the yaw is about the table normal, not the '
                     'base z axis')}
    # Image-space evidence for the pose: where the colour region was, and the
    # model outline that was fitted to it.  Kept in the record so an overlay
    # can be redrawn later from the log alone, without re-running the fit.
    entry['pixel'] = candidate.get('pixel')
    entry['bbox'] = candidate.get('bbox')
    entry['outline_uv'] = candidate.get('outline_uv')
    entry['measured'] = candidate['measured']
    entry['assumed'] = candidate['assumed']
    entry['quality'] = candidate['quality']
    entry['method'] = candidate['method']
    entry['association'] = candidate.get('association')
    jitter = track.jitter()
    if jitter:
        entry['still_jitter'] = jitter
    if state != 'detected':
        entry['warning'] = (f'this pose was measured {entry["age_s"]} s ago and is '
                            'not a current measurement')
    return entry


def build(tracker, now, frame_info, calibration_info, rejected=(), extra=None):
    """Assemble the scene record for one frame."""
    settings = tracker.settings
    objects = [object_record(track, now, settings, frame_info.get('to_base'))
               for track in tracker.tracks.values()]
    record = {
        'schema': SCHEMA,
        'captured_at': frame_info.get('captured_at', timestamp()),
        'written_at': timestamp(),
        'frame_index': frame_info.get('frame_index'),
        'frames': FRAMES,
        'reference_frame': 'table',
        'calibration': calibration_info,
        'camera': frame_info.get('camera'),
        'images': frame_info.get('images', {}),
        'catalogue': tracker.catalogue.describe(),
        'objects': objects,
        'rejected': [{'block_id': entry.get('block_id'),
                      'reasons': entry.get('reasons', []),
                      'pixel': entry.get('pixel'), 'bbox': entry.get('bbox'),
                      'quality': entry.get('quality')} for entry in rejected],
        'summary': {
            'detected': sum(1 for entry in objects if entry['state'] == 'detected'),
            'stale': sum(1 for entry in objects if entry['state'] == 'stale'),
            'lost': sum(1 for entry in objects if entry['state'] == 'lost'),
            'blocks': len(objects)},
        'usable_for_simulation': all(entry['state'] != 'lost' for entry in objects),
        'note': ('positions and yaw are camera measurements under the stated plane '
                 'assumption; centre height, roll and pitch are consequences of the '
                 'measured table and the catalogue block size, not observations'),
    }
    if extra:
        record.update(extra)
    return record


def write(record, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(record, indent=2, ensure_ascii=False) + '\n')
    return path


def append(record, path):
    """Append one frame to a JSONL log; one line is one complete scene state."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('a', encoding='utf-8') as stream:
        stream.write(json.dumps(record, ensure_ascii=False) + '\n')
    return path


def read(path):
    """Load a scene record, or the last one of a JSONL log."""
    path = Path(path)
    text = path.read_text().strip()
    if not text:
        raise ValueError(f'{path} is empty')
    if path.suffix == '.jsonl':
        record = json.loads(text.splitlines()[-1])
    else:
        record = json.loads(text)
    if record.get('schema') != SCHEMA:
        raise ValueError(f'{path} is not a {SCHEMA} record')
    return record


def read_all(path):
    """Every frame of a JSONL log, in order, for replay."""
    records = []
    for line in Path(path).read_text().splitlines():
        if line.strip():
            record = json.loads(line)
            if record.get('schema') != SCHEMA:
                raise ValueError(f'{path} holds a record that is not {SCHEMA}')
            records.append(record)
    return records


def placements(record, use_filtered=True, allow_stale=False):
    """Block poses a simulation should apply, with the ones it must not.

    Returns ``(placements, skipped)``.  A stale or lost pose is skipped unless
    the caller explicitly opts in, because putting a five-second-old
    observation into a scene and calling it the current state is the mistake
    this whole schema exists to prevent.
    """
    chosen, skipped = [], []
    for entry in record.get('objects', []):
        if entry.get('pose') is None:
            skipped.append({'id': entry['id'], 'reason': 'never detected'})
            continue
        if entry['state'] == 'lost' or (entry['state'] == 'stale' and not allow_stale):
            skipped.append({'id': entry['id'],
                            'reason': f"state is {entry['state']} "
                                      f"(age {entry.get('age_s')} s)"})
            continue
        pose = entry['pose_filtered'] if use_filtered else entry['pose']
        chosen.append({'id': entry['id'],
                       'position_m': list(pose['position_m']),
                       'quaternion_wxyz': list(pose['quaternion_wxyz']),
                       'yaw_deg': pose['yaw_deg'],
                       'size_m': entry['block']['size_m'],
                       'colour': entry['block']['colour'],
                       'state': entry['state'], 'age_s': entry.get('age_s'),
                       'observed_at': entry.get('observed_at'),
                       'source': 'filtered' if use_filtered else 'raw'})
    return chosen, skipped


def contact_check(records, first, second, catalogue, tolerance_mm=1.0):
    """Two blocks pushed flush together: their centres are one edge apart.

    The cheapest independent check in this project, and the only one here
    that does not come out of the same calibration chain it is testing.  Two
    cubes in face-to-face contact have a centre separation equal to the mean
    of their edge lengths -- a fact about the blocks, not about the camera --
    so the difference is a real error, in millimetres, with no ruler and no
    ground-truth fixture.

    The separation is split along the contacting face's normal and across it.
    The normal component is what must equal the edge; a large lateral
    component means the blocks are offset along the face, so they were not
    placed as this check assumes and the result says so rather than counting
    the offset as error.
    """
    import numpy as np

    from . import geometry

    expected = 1000 * (catalogue.by_id(first).size_m[0] +
                       catalogue.by_id(second).size_m[0]) / 2
    rows = []
    for record in records:
        entries = {entry['id']: entry for entry in record.get('objects', [])}
        pair = [entries.get(first), entries.get(second)]
        if any(entry is None or entry.get('pose') is None or
               entry['state'] != 'detected' for entry in pair):
            continue
        poses = [entry['pose'] for entry in pair]
        separation = (np.asarray(poses[1]['position_m'][:2]) -
                      np.asarray(poses[0]['position_m'][:2]))
        yaw = math.radians(poses[0]['yaw_deg'])
        normals = [np.array([math.cos(yaw + quarter * math.pi / 2),
                             math.sin(yaw + quarter * math.pi / 2)])
                   for quarter in range(4)]
        normal = max(normals, key=lambda vector: float(vector @ separation))
        tangent = np.array([-normal[1], normal[0]])
        rows.append({'frame_index': record.get('frame_index'),
                     'observed_at': record.get('captured_at'),
                     'separation_mm': 1000 * float(np.linalg.norm(separation)),
                     'along_contact_normal_mm': 1000 * float(normal @ separation),
                     'lateral_mm': 1000 * float(tangent @ separation),
                     'relative_yaw_deg': math.degrees(geometry.yaw_difference(
                         math.radians(poses[0]['yaw_deg']),
                         math.radians(poses[1]['yaw_deg']),
                         math.radians(poses[0]['symmetry_step_deg'])))})
    if not rows:
        return {'frames': 0, 'reason': 'the two blocks were never both detected'}
    along = np.array([row['along_contact_normal_mm'] for row in rows])
    lateral = np.array([row['lateral_mm'] for row in rows])
    error = float(along.mean() - expected)
    flush = bool(abs(lateral.mean()) < 3.0)
    return {
        'blocks': [first, second], 'frames': len(rows),
        'expected_mm': expected,
        'expected_from': 'the mean of the two catalogue edge lengths; two blocks '
                         'in face-to-face contact are exactly that far apart',
        'measured_mm': float(along.mean()),
        'measurement_std_mm': float(along.std()),
        'error_mm': error, 'error_percent': 100 * error / expected,
        'lateral_mm': float(lateral.mean()),
        'relative_yaw_deg': float(np.mean([row['relative_yaw_deg'] for row in rows])),
        'blocks_were_flush': flush,
        'tolerance_mm': float(tolerance_mm),
        'passed': bool(flush and abs(error) <= tolerance_mm),
        'note': ('a failure here is a real error in metres and cannot be explained '
                 'away by repeatability: a stable estimate can be stably wrong. '
                 'If the blocks were not flush (lateral is large) the placement, '
                 'not the pipeline, is what failed'),
        'rows': rows,
    }
