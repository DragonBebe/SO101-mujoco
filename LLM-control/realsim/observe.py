"""The real side: a calibrated frame in, a scene state out.

Everything that touches the physical setup is here, and all of it is
read-only.  The camera is opened, optionally the servo bus is read, and
nothing is ever commanded: mapping the world into a simulation is not a
reason to move a real arm, and an arm that moved between the frame and the
reading would make both wrong anyway.

The frame this produces is the one the calibration describes -- undistorted
with the measured ``K``/``D`` into ``K_new``, never the raw frame paired with
``K_new`` and never the undistorted frame paired with ``K``.  That pairing is
``rgbcal.realcam``'s rule and it is reused rather than restated, which is
also why a capture whose resolution, device or focus does not match the
intrinsics is refused here instead of quietly producing plausible numbers.
"""
from pathlib import Path
import hashlib
import json
import time

import cv2
import numpy as np

from rgbcal import realcam
from rgbcal.capture import check_compatible, save_frame, timestamp

from . import overlay, scene, track


def digest(path):
    """Short content hash, so a log line names the exact calibration used."""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()[:12]


def calibration_report(calibration, settings, new_matrix):
    """Which calibration produced this frame, and whether it may drive motion."""
    status = realcam.motion_status(calibration, settings)
    return {
        'sources': calibration.sources,
        'versions': {key: digest(path) for key, path in calibration.sources.items()
                     if Path(path).exists()},
        'imaging': settings.as_dict(),
        'intrinsics_resolution': {'width': calibration.size[0],
                                  'height': calibration.size[1]},
        'undistorted_K': new_matrix.tolist(),
        'distortion': 'removed: frames are undistorted into K_new before use',
        'table_plane': calibration.table.get('plane'),
        'table_fit_residual_rms_mm': calibration.table.get('fit_residual_rms_mm'),
        'T_base_table': calibration.T_base_table.tolist(),
        'T_table_camera_opencv': calibration.T_table_camera.tolist(),
        'usable_for_motion': status['usable_for_motion'],
        'usable_for_motion_reasons': status['reasons'],
        'motion_note': ('this mapping reads the camera only. usable_for_motion '
                        'describes the calibration, and is not made true by a good '
                        'looking overlay'),
    }


class RealObserver:
    """Turns calibrated frames into scene records, live or from a file."""

    def __init__(self, settings, catalogue, calibration=None, tracker=None,
                 directory=None, fit_settings=None, track_settings=None):
        self.settings = settings
        self.catalogue = catalogue
        self.calibration = calibration or realcam.load()
        # The same refusal rgbcal makes when it opens the camera, made here as
        # well: an offline frame taken at another resolution or focus is not
        # described by these intrinsics either, and every pose built on them
        # would be wrong without looking wrong.
        check_compatible(self.calibration.intrinsics, settings)
        self.new_matrix, self.maps = self.calibration.undistortion()
        self.nexus_calibration = self.calibration.nexus_calibration(
            self.new_matrix, frame='table')
        self.tracker = tracker or track.Tracker(catalogue, track_settings,
                                                fit_settings)
        self.directory = Path(directory) if directory else None
        # Raw frames are evidence and are never overwritten, so a second run
        # into the same session continues the numbering rather than colliding
        # with the first one's frame-0001.
        self.frame_index = (len(list((self.directory / 'frames').glob('*.png')))
                            if self.directory and (self.directory / 'frames').exists()
                            else 0)
        self.report = calibration_report(self.calibration, settings, self.new_matrix)

    def add_warnings(self, record):
        """Conditions that do not make a pose wrong but do limit what it means.

        Kept separate from the rejection reasons in ``estimate``: those say a
        fit failed, these say a fit succeeded somewhere the calibration was
        never checked, or with a block size nobody measured.  Both belong in
        the record; conflating them would either hide a real caveat or throw
        away a usable pose.
        """
        region = self.calibration.table.get('covered_region_base_m')
        warnings = []
        for block in self.catalogue.blocks:
            if not block.size_measured:
                warnings.append(
                    f'{block.id}: its size was never measured ({block.size_source}). '
                    'Every position reported for it scales with that number')
        for entry in record['objects']:
            notes = []
            if entry.get('pose_base') and _outside(region,
                                                   entry['pose_base']['position_m']):
                notes.append(
                    'outside the region the table plane was fitted over '
                    f'(base x {region["x"][0]:.3f}..{region["x"][1]:.3f} m, '
                    f'y {region["y"][0]:.3f}..{region["y"][1]:.3f} m): the support '
                    'height here is extrapolated')
            if entry.get('block', {}).get('size_measured') is False:
                notes.append('block size never measured; position scales with it')
            if notes:
                entry['warnings'] = notes
                warnings.extend(f"{entry['id']}: {note}" for note in notes)
        if not self.report['usable_for_motion']:
            warnings.append('this calibration is not cleared to drive the real arm: '
                            + '; '.join(self.report['usable_for_motion_reasons']))
        record['warnings'] = warnings
        return warnings

    def undistort(self, raw_bgr):
        return cv2.remap(raw_bgr, *self.maps, cv2.INTER_LINEAR)

    def camera_block(self):
        camera = dict(self.nexus_calibration)
        camera['frame'] = 'table'
        camera['axes'] = ('rotation maps OpenGL camera axes into the table frame, '
                          'the convention nexus_vision.perception uses')
        camera['T_table_camera_opencv'] = self.calibration.T_table_camera.tolist()
        return camera

    def process(self, raw_bgr, captured_at=None, support_heights=None,
                robot=None, save_images=True, label='frame'):
        """One frame: undistort, estimate, track, and build the scene record."""
        started = time.perf_counter()
        captured_at = captured_at or timestamp()
        self.frame_index += 1
        undistorted = self.undistort(raw_bgr)
        rgb = cv2.cvtColor(undistorted, cv2.COLOR_BGR2RGB)
        now = time.monotonic()
        detections, rejected = self.tracker.update(
            rgb, self.nexus_calibration, now, support_heights)
        for candidate in detections.values():
            candidate['observed_at'] = captured_at
            candidate['frame_index'] = self.frame_index
        images = {}
        if save_images and self.directory:
            stem = f'{label}-{self.frame_index:04d}'
            images['raw'] = str(save_frame(
                raw_bgr, self.directory / 'frames', stem,
                {'stage': 'realsim-observe', 'frame_index': self.frame_index},
                configuration=self.settings.as_dict()))
        elapsed = time.perf_counter() - started
        record = scene.build(
            self.tracker, now,
            {'captured_at': captured_at, 'frame_index': self.frame_index,
             'camera': self.camera_block(), 'images': images,
             'to_base': lambda point: realcam.table_to_base(self.calibration, point)},
            self.report, rejected,
            extra={'robot': robot,
                   'timing': {'estimate_s': round(elapsed, 3),
                              'note': 'time from frame in hand to record built; it '
                                      'excludes capture and disk'}})
        self.add_warnings(record)
        if save_images and self.directory:
            picture = overlay.annotate(
                rgb, self.nexus_calibration, record['objects'],
                header=[f"frame {self.frame_index}  {captured_at}",
                        f"detected {record['summary']['detected']}/"
                        f"{record['summary']['blocks']}  "
                        f"{elapsed * 1000:.0f} ms"])
            record['images']['overlay'] = str(overlay.save(
                picture, self.directory / 'overlay' / f'{label}-{self.frame_index:04d}.png'))
            record['images']['undistorted_note'] = (
                'the undistorted frame is reproducible from the raw one and the '
                'recorded calibration; only the raw frame is archived')
        return record


def _outside(region, position):
    """True when a base-frame point lies outside the table fit's coverage."""
    if not region:
        return False
    for axis, key in ((0, 'x'), (1, 'y')):
        low, high = region.get(key, (None, None))
        if low is None or not low <= position[axis] <= high:
            return True
    return False


def read_raw(path):
    """An archived raw frame and when it was taken, from its own sidecar.

    An offline frame must not be stamped with the time it was re-processed:
    the whole state machine downstream is about how old an observation is.
    """
    path = Path(path)
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f'cannot read {path}')
    sidecar = path.with_suffix('.json')
    captured_at = None
    if sidecar.exists():
        captured_at = json.loads(sidecar.read_text()).get('saved_at')
    return image, captured_at


def read_arm(port='/dev/ttyACM0', extrinsics=None, lerobot_calibration=None):
    """Joint angles in the solved convention, read-only.

    Reuses the hand-eye solution's signs and zero offsets rather than a fresh
    guess, because those are the convention the extrinsic was fitted with;
    mixing conventions is exactly what puts a mirrored arm in a pose the real
    one is not in.  The jaw is different: its reading is reported as a
    fraction of the calibrated travel, and the note says plainly that no one
    has checked that fraction against the model's jaw.
    """
    from rgbcal import robot as robot_module
    from rgbcal.pointcheck import solved_arm

    calibration = realcam.load(extrinsics=extrinsics)
    tcp = solved_arm(calibration.extrinsics)
    with robot_module.ServoBus(port) as bus:
        ticks = bus.read_positions()
    pose, angles = tcp(ticks)
    record = {'read_at': timestamp(), 'servo_ticks': {str(k): v for k, v in ticks.items()},
              'joint_angles_rad': angles,
              'tcp_base_m': pose[:3, 3].tolist(),
              'source': 'servo bus, read-only; joint convention from '
                        + Path(calibration.sources['extrinsics']).name,
              'convention': calibration.extrinsics.get('joint_convention')}
    path = Path(lerobot_calibration or (Path.home() /
                '.cache/huggingface/lerobot/calibration/robots/so101_follower/follower.json'))
    if path.exists():
        data = json.loads(path.read_text())
        entry = data.get('gripper')
        if entry:
            low, high = float(entry['range_min']), float(entry['range_max'])
            value = float(ticks.get(6, low))
            record['gripper_ticks'] = value
            record['gripper_fraction'] = float(np.clip((value - low) / max(high - low, 1),
                                                       0.0, 1.0))
            record['gripper_note'] = (
                'fraction of the lerobot calibrated jaw travel. Mapping it onto the '
                'simulated jaw angle is a linear guess that nobody has verified '
                'against the real jaw opening')
    return record
