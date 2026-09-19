"""The simulation export carries exactly what the real RGB pipeline uses."""
import numpy as np
import pytest

from rgbcal import realcam, simcam

have_calibration = all(realcam.DEFAULTS[key].exists()
                       for key in ('intrinsics', 'extrinsics', 'table'))


@pytest.mark.skipif(not have_calibration, reason='no measured calibration on this machine')
def test_export_matches_the_real_pipelines_undistorted_table_frame_camera(tmp_path):
    calibration = realcam.load()
    new_matrix, _ = calibration.undistortion()
    expected = calibration.nexus_calibration(new_matrix, frame='table')
    output, record = simcam.export(tmp_path / 'sim_camera.json')
    assert output.exists() and record['schema'] == simcam.SCHEMA
    np.testing.assert_allclose(record['K'], new_matrix)
    for key in ('fx', 'fy', 'cx', 'cy'):
        assert record[key] == pytest.approx(expected[key])
    np.testing.assert_allclose(record['position'], expected['position'])
    np.testing.assert_allclose(record['rotation_opengl'], expected['rotation'])
    # Above the table, looking down, and never usable to move the real arm.
    assert record['position'][2] > 0.1 and record['look_down_deg'] > 5
    assert record['usable_for_motion'] is False
