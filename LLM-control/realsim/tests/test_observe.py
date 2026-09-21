"""The real side's guards: calibration pairing, provenance and coverage.

Needs OpenCV and the measured calibration, so it is skipped elsewhere.
"""
import json

import numpy as np
import pytest

pytest.importorskip('cv2')

from rgbcal import realcam                                        # noqa: E402
from realsim import blocks, observe                               # noqa: E402

have_calibration = all(realcam.DEFAULTS[key].exists()
                       for key in ('intrinsics', 'extrinsics', 'table'))
needs_calibration = pytest.mark.skipif(
    not have_calibration, reason='no measured calibration on this machine')


def test_coverage_test_flags_only_points_outside_the_fitted_region():
    region = {'x': [0.12, 0.45], 'y': [-0.13, 0.17]}
    assert not observe._outside(region, [0.25, 0.0, 0.0])
    assert observe._outside(region, [0.25, 0.30, 0.0])
    assert observe._outside(region, [0.50, 0.0, 0.0])
    # No region recorded means nothing can be claimed about coverage.
    assert not observe._outside(None, [9.0, 9.0, 0.0])


def test_an_archived_frame_brings_its_own_capture_time(tmp_path):
    import cv2

    image = np.zeros((8, 8, 3), dtype=np.uint8)
    path = tmp_path / 'frame.png'
    cv2.imwrite(str(path), image)
    loaded, captured_at = observe.read_raw(path)
    assert loaded.shape == image.shape and captured_at is None
    path.with_suffix('.json').write_text(json.dumps({'saved_at': '2026-09-18T17:00:00+02:00'}))
    _, captured_at = observe.read_raw(path)
    assert captured_at == '2026-09-18T17:00:00+02:00'


@needs_calibration
def test_a_capture_that_does_not_match_the_intrinsics_is_refused():
    from rgbcal.capture import CameraSettings

    calibration = realcam.load()
    new_matrix, _ = calibration.undistortion()
    wrong = CameraSettings(device='/dev/video0', width=1280, height=720)
    report = observe.calibration_report(calibration, wrong, new_matrix)
    assert not report['usable_for_motion']
    assert any('1280x720' in reason for reason in report['usable_for_motion_reasons'])
    with pytest.raises(RuntimeError, match='does not match'):
        observe.RealObserver(wrong, blocks.load(), calibration)


@needs_calibration
def test_every_record_names_the_calibration_files_it_used():
    from rgbcal.cli import _profile
    from rgbcal.capture import CameraSettings

    settings = CameraSettings(**_profile('c920'))
    calibration = realcam.load()
    new_matrix, _ = calibration.undistortion()
    report = observe.calibration_report(calibration, settings, new_matrix)
    assert set(report['versions']) == {'intrinsics', 'extrinsics', 'table'}
    assert all(len(value) == 12 for value in report['versions'].values())
    # The table frame is the simulation world; its origin sits on the table.
    assert report['T_base_table'][2][3] == pytest.approx(
        calibration.table['plane']['offset_m'] *
        calibration.table['plane']['normal_base'][2], abs=1e-9)


@needs_calibration
def test_the_warnings_name_unmeasured_sizes_and_the_motion_status(tmp_path):
    from rgbcal.cli import _profile
    from rgbcal.capture import CameraSettings

    settings = CameraSettings(**_profile('c920'))
    # A block whose size nobody measured: the shipped catalogue's sizes are
    # filled in, so the warning has to be provoked deliberately.
    unmeasured = blocks.Block(id='mystery', colour='red', size_m=(0.03,) * 3,
                              size_source='nominal, from a label', size_measured=False,
                              mass_kg=0.01, mass_source='default',
                              friction=(1.0, 0.005, 0.0001), friction_source='default')
    catalogue = blocks.Catalogue(blocks=[unmeasured])
    observer = observe.RealObserver(settings, catalogue, realcam.load())
    record = {'objects': [{'id': block.id, 'block': block.describe()}
                          for block in catalogue.blocks]}
    warnings = observer.add_warnings(record)
    assert any('never measured' in warning for warning in warnings)
    assert any('not cleared to drive the real arm' in warning for warning in warnings)
