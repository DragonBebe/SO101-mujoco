"""Camera preview must not change policy frames or simulation physics."""
import importlib
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from nexus_vision.simulation import VisualSimulation


def test_depth_display_uses_fixed_metric_scale_and_marks_invalid():
    assert importlib.util.find_spec('nexus_vision.camera_viewer') is not None
    from nexus_vision.camera_viewer import colorize_depth
    depth = np.array([[0.1, 0.5, 0.9, np.nan, np.inf, 0, -1]], dtype=float)
    rgb = colorize_depth(depth)
    assert rgb.shape == (1, 7, 3) and rgb.dtype == np.uint8
    assert np.all(rgb[0, 3:] == 0)
    assert not np.array_equal(rgb[0, 0], rgb[0, 1])
    assert not np.array_equal(rgb[0, 1], rgb[0, 2])
    # The same distance keeps its color when other pixels/ranges change.
    np.testing.assert_array_equal(rgb[0, 1], colorize_depth(np.array([[0.5, 8.0]]))[0, 0])


def test_preview_capture_does_not_invalidate_localization_or_write_frames(tmp_path):
    with VisualSimulation('Touch', tmp_path, seed=4) as sim:
        assert hasattr(sim, 'capture_cameras')
        frame = sim.frame_id
        files = set(tmp_path.iterdir())
        qpos = sim.data.qpos.copy()
        time = sim.data.time
        old_frames = sim.frames
        cameras = sim.capture_cameras()
        assert set(cameras) == {'overhead', 'wrist'}
        for rgb, depth, calibration in cameras.values():
            assert rgb.shape == (480, 640, 3)
            assert depth.shape == (480, 640)
            assert calibration['width'] == 640
        assert sim.frame_id == frame and sim.frames is old_frames
        assert set(tmp_path.iterdir()) == files
        np.testing.assert_array_equal(sim.data.qpos, qpos)
        assert sim.data.time == time
        assert sim.execute({'action':'localize','pixel':[50,50],'frame_id':frame})['point_id']


def test_live_window_updates_during_motion_and_close_keeps_control_alive(tmp_path, monkeypatch):
    import os
    import pytest
    if os.environ.get('MUJOCO_GL') != 'glfw' or not os.environ.get('DISPLAY'):
        pytest.skip('Live window integration requires MUJOCO_GL=glfw and a desktop')
    import glfw
    from OpenGL import GL
    with VisualSimulation('Touch', tmp_path, camera_viewer=True) as sim:
        panel = sim.camera_viewer
        times = []
        original_show = panel.show

        def record_show(cameras, simulation_time):
            original_show(cameras, simulation_time)
            times.append(simulation_time)
        monkeypatch.setattr(panel, 'show', record_show)
        start = sim.data.time
        # Make sure the first in-motion refresh is due; no new observation yet.
        import time
        time.sleep(0.11)
        result = sim.execute({'action':'wait','seconds':1})
        assert any(start < t < result['time'] for t in times)
        previous = glfw.get_current_context()
        try:
            glfw.make_context_current(panel.window)
            GL.glReadBuffer(GL.GL_FRONT)
            pixels = GL.glReadPixels(0, 0, 960, 836, GL.GL_RGB, GL.GL_UNSIGNED_BYTE)
            assert np.frombuffer(pixels, dtype=np.uint8).std() > 20
        finally:
            glfw.make_context_current(previous)
        frame = sim.frame_id
        glfw.set_window_should_close(panel.window, True)
        sim.update_camera_viewer()
        assert panel.window is None
        assert sim.frame_id == frame
        assert sim.execute({'action':'wait','seconds':0.1})['time'] > result['time']


def test_camera_only_realtime_paces_motion(tmp_path):
    import os
    import time
    import pytest
    if os.environ.get('MUJOCO_GL') != 'glfw' or not os.environ.get('DISPLAY'):
        pytest.skip('Live window integration requires GLFW and a desktop')
    with VisualSimulation('Touch', tmp_path, camera_viewer=True, realtime=True) as sim:
        start = time.monotonic()
        sim.execute({'action':'wait','seconds':0.6})
        assert time.monotonic() - start >= 0.55


def test_worker_draws_new_world_even_when_simulation_time_is_unchanged(monkeypatch):
    from queue import Queue
    from threading import Event, Thread
    from nexus_vision import camera_viewer
    frames, ready, drawn = Queue(), Queue(), Queue()
    stopped = Event()

    class Window:
        def poll(self): return True
        def due(self, simulation_time): return False
        def show(self, cameras, simulation_time): drawn.put(cameras)
        def close(self): pass

    # Replace only the desktop boundary; exercise the worker's real delivery loop.
    monkeypatch.setattr(camera_viewer, 'CameraViewer', Window)
    worker = Thread(target=camera_viewer._camera_process, args=(frames, ready, stopped))
    worker.start()
    try:
        assert ready.get(timeout=2) is None
        frames.put(('world-one', .25))
        assert drawn.get(timeout=2) == 'world-one'
        frames.put(('world-two', .25))
        assert drawn.get(timeout=2) == 'world-two'
    finally:
        stopped.set()
        worker.join(timeout=2)
