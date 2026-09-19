"""Live camera dashboard. Display only: never creates policy observations."""
import time

import numpy as np
from PIL import Image, ImageDraw

#: Layout used when no configuration is supplied: the original RGB-D pair.
DEFAULT_VIEW = {'modality': 'rgbd', 'cameras': ('overhead', 'wrist'),
                'environment_camera': 'overhead'}
TILE = (480, 360)
COLUMNS = 2


def colorize_depth(depth):
    """Fixed optical-depth scale: 0 m blue, 1 m red; invalid pixels black."""
    valid = np.isfinite(depth) & (depth > 0)
    value = np.clip(np.where(valid, depth, 0), 0, 1)
    # Piecewise linear blue -> cyan -> yellow -> red map.
    anchors = np.array([[30, 70, 255], [0, 220, 220], [255, 230, 30], [255, 40, 30]])
    rgb = np.stack([np.interp(value, [0, 1/3, 2/3, 1], anchors[:, c])
                    for c in range(3)], axis=-1).astype(np.uint8)
    rgb[~valid] = 0
    return rgb


def panel_cells(view):
    """Tiles to draw, as ``(camera name, 'rgb' | 'depth')`` in reading order.

    RGB-D keeps one camera per row with its depth beside it. RGB-only has no
    depth tile to draw, so the two cameras share a single row instead of
    leaving half the window blank or filled with a stale depth image.
    """
    names = tuple(view.get('cameras', DEFAULT_VIEW['cameras']))
    if view.get('modality', DEFAULT_VIEW['modality']) == 'rgbd':
        return [(name, kind) for name in names for kind in ('rgb', 'depth')]
    return [(name, 'rgb') for name in names]


def panel_size(view):
    """Pixel size of the composed panel, and therefore of the window."""
    cells = panel_cells(view)
    rows = -(-len(cells) // COLUMNS)
    legend = 20 if any(kind == 'depth' for _, kind in cells) else 10
    return COLUMNS * TILE[0], 40 + rows * (TILE[1] + 28) + legend


def fit_tile(width, height):
    """Largest size of that aspect inside ``TILE``, and its centring offset.

    Cameras need not share an aspect (the calibrated C920 is 16:9, the wrist
    camera 4:3); stretching both to one tile would distort the geometry.
    """
    scale = min(TILE[0] / width, TILE[1] / height)
    size = (max(1, round(width * scale)), max(1, round(height * scale)))
    return size, ((TILE[0] - size[0]) // 2, (TILE[1] - size[1]) // 2)


def compose_panel(cameras, simulation_time, view=DEFAULT_VIEW):
    cells = panel_cells(view)
    width, height = panel_size(view)
    panel = Image.new('RGB', (width, height), (20, 24, 32))
    draw = ImageDraw.Draw(panel)
    modality = view.get('modality', DEFAULT_VIEW['modality'])
    environment = view.get('environment_camera', DEFAULT_VIEW['environment_camera'])
    draw.text((12, 10), f'Live cameras | mode {modality.upper()} | environment camera '
                        f'{environment.upper()} (world-fixed) + WRIST (on the arm) | '
                        f'Simulation {simulation_time:.2f} s | preview only',
              fill='white')
    depth_shown = False
    for index, (name, kind) in enumerate(cells):
        rgb, depth, _ = cameras[name]
        column, row = index % COLUMNS, index // COLUMNS
        x, y = column * TILE[0], 40 + row * (TILE[1] + 28)
        if kind == 'rgb':
            label, array = f'{name.upper()} / RGB', rgb
        else:
            label, array = f'{name.upper()} / DEPTH (optical Z, metres)', colorize_depth(depth)
            depth_shown = True
        draw.text((x + 12, y), label, fill='white')
        tile, offset = fit_tile(array.shape[1], array.shape[0])
        panel.paste(Image.fromarray(array).resize(tile, Image.Resampling.NEAREST),
                    (x + offset[0], y + 22 + offset[1]))
    if depth_shown:
        ramp = np.linspace(0.00001, 1, 320)[None, :]
        panel.paste(Image.fromarray(colorize_depth(ramp)).resize((320, 12)), (110, height - 18))
        draw.text((12, height - 18), 'Depth: 0 m', fill='white')
        draw.text((440, height - 18), '1 m+     Black: invalid / no surface', fill='white')
    return np.asarray(panel)


class CameraViewer:
    """Owns one GLFW window without terminating MuJoCo's GLFW resources."""
    def __init__(self, view=DEFAULT_VIEW):
        import glfw
        from OpenGL import GL
        self.glfw, self.gl = glfw, GL
        self.view = view
        self.window = None
        self.last_update = -float('inf')
        self.last_simulation_time = None
        if not glfw.init():
            raise RuntimeError('Camera viewer requires a working desktop display (GLFW)')
        # No glfw.terminate(): the MuJoCo viewer/renderers share this library.
        glfw.default_window_hints()
        width, height = panel_size(view)
        modality = view.get('modality', DEFAULT_VIEW['modality'])
        title = ('SO101 Cameras - RGB + Depth' if modality == 'rgbd'
                 else 'SO101 Cameras - RGB only (no depth)')
        self.window = glfw.create_window(width, height, title, None, None)
        if not self.window:
            raise RuntimeError('Could not create camera viewer window')
        self._redraw = True
        glfw.set_window_refresh_callback(self.window, self._request_redraw)
        glfw.set_framebuffer_size_callback(self.window, self._request_redraw)

    def _request_redraw(self, *_):
        self._redraw = True

    def poll(self):
        if self.window:
            self.glfw.poll_events()
            if self.glfw.window_should_close(self.window):
                self.close()
        return self.window is not None

    def due(self, simulation_time, force=False):
        if not self.poll():
            return False
        return force or ((self._redraw or simulation_time != self.last_simulation_time)
                         and time.monotonic() - self.last_update >= 0.1)

    def show(self, cameras, simulation_time):
        if not self.window:
            return
        glfw, gl = self.glfw, self.gl
        panel = compose_panel(cameras, simulation_time, self.view)
        previous = glfw.get_current_context()
        try:
            glfw.make_context_current(self.window)
            width, height = glfw.get_framebuffer_size(self.window)
            if width <= 0 or height <= 0:
                return
            gl.glViewport(0, 0, width, height)
            gl.glClearColor(0.08, 0.09, 0.12, 1)
            gl.glClear(gl.GL_COLOR_BUFFER_BIT)
            gl.glMatrixMode(gl.GL_PROJECTION)
            gl.glLoadIdentity()
            gl.glMatrixMode(gl.GL_MODELVIEW)
            gl.glLoadIdentity()
            gl.glPixelStorei(gl.GL_UNPACK_ALIGNMENT, 1)
            gl.glRasterPos2f(-1, -1)
            gl.glPixelZoom(width / panel.shape[1], height / panel.shape[0])
            gl.glDrawPixels(panel.shape[1], panel.shape[0], gl.GL_RGB,
                            gl.GL_UNSIGNED_BYTE, np.ascontiguousarray(panel[::-1]))
            glfw.swap_buffers(self.window)
        finally:
            glfw.make_context_current(previous)
        self._redraw = False
        self.last_update = time.monotonic()
        self.last_simulation_time = simulation_time

    def close(self):
        if self.window:
            self.glfw.destroy_window(self.window)
            self.window = None


def _camera_process(frames, ready, stopped, view=DEFAULT_VIEW):
    """Own the dashboard GLFW event loop separately from MuJoCo's viewer."""
    from queue import Empty
    panel = None
    try:
        panel = CameraViewer(view)
        ready.put(None)
        latest = None
        while not stopped.is_set() and panel.poll():
            received = False
            try:
                latest = frames.get(timeout=.05)
                received = True
            except Empty:
                pass
            if latest is not None and (received or panel.due(latest[1])):
                panel.show(*latest)
    except Exception as exc:
        ready.put(str(exc))
    finally:
        stopped.set()
        if panel:
            panel.close()


class ProcessCameraViewer:
    """Bounded latest-frame transport; the physics thread never waits on a GUI."""
    def __init__(self, view=DEFAULT_VIEW):
        import multiprocessing
        context = multiprocessing.get_context('spawn')
        self.view = view
        self.frames = context.Queue(maxsize=1)
        self.ready = context.Queue(maxsize=1)
        self.stopped = context.Event()
        self.last_update = -float('inf')
        self.last_simulation_time = None
        self.pending = None
        self.process = context.Process(target=_camera_process,
                                       args=(self.frames, self.ready, self.stopped, view),
                                       daemon=True)
        self.process.start()
        try:
            error = self.ready.get(timeout=15)
            if error is not None:
                raise RuntimeError(error)
        except Exception as exc:
            self.close()
            raise RuntimeError(f'Camera window failed to start: {exc}') from exc

    def poll(self):
        from queue import Full
        running = not self.stopped.is_set() and self.process.is_alive()
        if running and self.pending is not None:
            try:
                self.frames.put_nowait(self.pending)
            except Full:
                pass
            else:
                self.last_simulation_time = self.pending[1]
                self.last_update = time.monotonic()
                self.pending = None
        return running

    def due(self, simulation_time, force=False):
        return self.poll() and (force or (simulation_time != self.last_simulation_time
                                          and time.monotonic() - self.last_update >= .1))

    def show(self, cameras, simulation_time):
        # Keep the newest unsent frame for the next poll. In particular a reset
        # can change the image without changing time; that frame must not vanish.
        self.pending = (cameras, simulation_time)
        self.poll()

    def close(self):
        self.stopped.set()
        self.process.join(timeout=2)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=2)
        for queue in (self.frames, self.ready):
            queue.cancel_join_thread()
            queue.close()
