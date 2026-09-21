"""Write the captured scene as a standalone MJCF anyone can open.

The mirror in :mod:`realsim.mirror` is a live object inside this process.
This is the other thing people want: one file on disk that MuJoCo's own
viewer, ``simulate``, or any other tool can load, showing the blocks where
the camera saw them.

Two details make it standalone rather than nearly standalone:

The robot
    is embedded from the existing vendored XML, with its calibrated base transform.  The vendored model
    declares ``meshdir="assets"``, which MuJoCo resolves against the *main*
    file, so a scene written anywhere else would fail to find a single mesh.
    Declaring ``<compiler meshdir="...">`` with an absolute path **after**
    the embedded model overrides it, which is what keeps the file portable across
    directories.

The camera
    is written with the measured intrinsics, not a field of view.  MuJoCo
    cameras take ``resolution``/``focalpixel``/``principalpixel``, which
    expresses an off-centre principal point exactly, so switching to this
    camera in a viewer shows the scene from the real camera's viewpoint
    rather than an approximation of it.

The blocks carry their pose on the body, so the model's rest state already
is the captured scene -- no keyframe has to be loaded first.  A keyframe is
written as well, holding the arm pose the record carried if it had one.
"""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

import numpy as np

from . import scene as scene_module

SCHEMA = 'realsim/mjcf/1'
#: Arbitrary physical pixel pitch for the exported camera; see _camera_element.
PIXEL_PITCH_M = 1e-5


def _camera_element(camera, name='calibrated'):
    """The measured camera as MJCF, in the world (= table) frame.

    A MuJoCo camera looks along its own -z with +y up, which is exactly the
    OpenGL convention the calibration export already uses, so its rotation
    columns are the camera's x and y axes and go straight into ``xyaxes``.
    """
    position = np.asarray(camera['position'], dtype=float)
    rotation = np.asarray(camera['rotation'], dtype=float)
    width, height = int(camera['width']), int(camera['height'])
    # MuJoCo's principalpixel runs opposite to the image axes on BOTH axes:
    # positive x moves the principal point left and positive y moves it down
    # in the rendered image.  Measured, not assumed -- the four sign
    # combinations were rendered and the intrinsics read back from the GL
    # frustum, and only this one reproduces the calibrated cx/cy exactly.
    # tests/test_export.py holds it there.
    principal_x = (width - 1) / 2.0 - float(camera['cx'])
    principal_y = float(camera['cy']) - (height - 1) / 2.0
    axes = ' '.join(f'{value:.9f}' for value in
                    list(rotation[:, 0]) + list(rotation[:, 1]))
    # MuJoCo's physical camera model needs a sensor size before it will accept
    # a focal length.  Focal and principal are given here in pixels, so the
    # sensor's physical size only sets the unit both are converted through and
    # cancels out of the projection; PIXEL_PITCH_M just makes it a round number.
    sensor_x, sensor_y = width * PIXEL_PITCH_M, height * PIXEL_PITCH_M
    return (f'    <camera name="{name}" mode="fixed"\n'
            f'            pos="{position[0]:.9f} {position[1]:.9f} {position[2]:.9f}"\n'
            f'            xyaxes="{axes}"\n'
            f'            resolution="{width} {height}"\n'
            f'            sensorsize="{sensor_x:.9f} {sensor_y:.9f}"\n'
            f'            focalpixel="{camera["fx"]:.6f} {camera["fy"]:.6f}"\n'
            f'            principalpixel="{principal_x:.6f} {principal_y:.6f}"/>\n')


def _block_body(entry, block):
    """One captured block: a free body with its own colour and size."""
    position = entry['position_m']
    quaternion = entry['quaternion_wxyz']
    half = block.half_extent
    red, green, blue = block.colour_spec.representative_rgb()
    friction = ' '.join(str(value) for value in block.friction)
    return (
        f'    <body name="{block.id}" pos="{position[0]:.9f} {position[1]:.9f} '
        f'{position[2]:.9f}" quat="{quaternion[0]:.9f} {quaternion[1]:.9f} '
        f'{quaternion[2]:.9f} {quaternion[3]:.9f}">\n'
        f'      <freejoint name="{block.id}_joint"/>\n'
        f'      <geom name="{block.id}_geom" type="box" '
        f'size="{half[0]:.9f} {half[1]:.9f} {half[2]:.9f}" '
        f'rgba="{red:.4f} {green:.4f} {blue:.4f} 1" mass="{block.mass_kg}" '
        f'friction="{friction}"/>\n'
        f'    </body>\n')


#: The simulation's own home pose, used only to give the exported scene a
#: sensible-looking arm when the record carries no joint readings.
REST_POSE_DEG = (70.0, -85.0, 85.0, 30.0, 0.0, 70.0)


def _keyframe(record, placed):
    """A loadable state for the arm, named for whether it was observed.

    The blocks already sit at their captured poses on their bodies, so the
    model's rest state *is* the captured scene.  The arm is the part that may
    not have been observed at all -- the servo bus is only read when asked --
    and an arm posed for the picture must never be mistaken for a measurement,
    so the keyframe's own name says which it is.
    """
    from nexus_vision.simulation import JOINT_NAMES

    arm = (record.get('robot') or {}).get('joint_angles_rad')
    if arm:
        name = 'arm_as_read'
        joints = [float(arm.get(joint, 0.0)) for joint in JOINT_NAMES]
        gripper = (record.get('robot') or {}).get('gripper_angle_rad')
        joints.append(0.0 if gripper is None else float(gripper))
        if gripper is None:
            name += '_gripper_NOT_observed'
    else:
        name = 'arm_rest_NOT_observed'
        joints = [math.radians(value) for value in REST_POSE_DEG]
    values = ' '.join(f'{value:.6f}' for value in joints)
    for entry in placed:
        values += ' ' + ' '.join(f'{value:.9f}' for value in entry['position_m'])
        values += ' ' + ' '.join(f'{value:.9f}' for value in entry['quaternion_wxyz'])
    return f'\n  <keyframe>\n    <key name="{name}" qpos="{values}"/>\n  </keyframe>\n'


def _provenance(record, placed, skipped, paths):
    """The header comment: where every number in this file came from."""
    lines = [
        'Captured tabletop scene, written by realsim (run_realsim.sh export).',
        '',
        f'schema        {SCHEMA}',
        f'captured at   {record.get("captured_at")}',
        f'frame index   {record.get("frame_index")}',
        'world frame   the measured table: z = 0 is the table surface, the origin',
        '              is the point of the table below the robot base origin, and x',
        '              is the base x axis projected onto the table.',
        'units         metres, radians. Quaternions are [w, x, y, z].',
        '',
        'Blocks placed:',
    ]
    for entry in placed:
        lines.append(
            f'  {entry["id"]:<12} x {1000 * entry["position_m"][0]:7.1f}  '
            f'y {1000 * entry["position_m"][1]:7.1f}  '
            f'z {1000 * entry["position_m"][2]:6.1f} mm   '
            f'yaw {entry["yaw_deg"]:6.1f} deg   ({entry["source"]})')
    for entry in skipped:
        lines.append(f'  {entry["id"]:<12} NOT PLACED: {entry["reason"]}')
    lines += [
        '',
        'What was measured and what was assumed:',
        '  measured by the camera   x, y and yaw of each block',
        '  assumed, not measured    the centre height (table plane + half the known',
        '                           block edge), and zero roll and pitch',
        '',
        'Calibration used:',
    ]
    for key, value in (record.get('calibration', {}).get('sources') or {}).items():
        lines.append(f'  {key:<12} {value}')
    for key, value in (record.get('calibration', {}).get('versions') or {}).items():
        lines.append(f'  {key:<12} sha256:{value}')
    lines += [
        '',
        'Accuracy: the block positions are as accurate as the calibration behind',
        'them, and nothing in this file re-checks that. Run',
        '"run_realsim.sh drift" and "run_realsim.sh scale-check" for the current',
        'state of both. Mass and friction below are simulation defaults that have',
        'never been identified from the real blocks.',
        '',
        f'Robot model embedded from {paths["robot"]}',
        '',
        'The blocks carry their captured pose on their bodies, so the scene opens',
        'already in the captured state. The arm does not: unless the record carried',
        'servo readings, the keyframe below holds the simulation\'s own home pose and',
        'is named to say so. Load it from the keyframe control in your viewer.',
    ]
    return '\n'.join(f'  {line}'.rstrip() for line in lines)


def _robot_elements(robot, record):
    """Embed the existing model with a calibrated base, without editing assets."""
    root = ET.parse(robot).getroot()
    if record.get('calibration', {}).get('T_base_table') is not None:
        import mujoco
        from .arm_sync import align_base
        model = mujoco.MjModel.from_xml_path(str(robot))
        data = mujoco.MjData(model)
        align_base(model, data, record)
        base = root.find("./worldbody/body[@name='base']")
        if base is None:
            raise ValueError('SO101 model has no named base body')
        base.set('pos', ' '.join(f'{x:.12g}' for x in model.body('base').pos))
        base.set('quat', ' '.join(f'{x:.12g}' for x in model.body('base').quat))
    return '\n'.join(ET.tostring(child, encoding='unicode') for child in root)


def mjcf(record, catalogue, camera=None, use_filtered=True, allow_stale=False,
         model_name='realsim_capture'):
    """Return the MJCF text for one scene record, and what went into it."""
    from so101_nexus.mujoco.pick_env import _SO101_DIR, _SO101_XML

    placed, skipped = scene_module.placements(record, use_filtered, allow_stale)
    robot = Path(_SO101_XML).resolve()
    meshdir = (Path(_SO101_DIR) / 'assets').resolve()
    bodies = ''.join(_block_body(entry, catalogue.by_id(entry['id']))
                     for entry in placed)
    camera_xml = _camera_element(camera) if camera else ''
    header = _provenance(record, placed, skipped,
                         {'robot': robot, 'meshdir': meshdir})
    keyframe = _keyframe(record, placed)
    robot_xml = _robot_elements(robot, record)
    text = f'''<mujoco model="{model_name}">
  <!--
{header}
  -->

  {robot_xml}
  <!-- After the embedded robot on purpose: its compiler sets meshdir="assets",
       which MuJoCo resolves against THIS file's directory. An absolute
       meshdir here is what lets the scene live anywhere. -->
  <compiler angle="radian" meshdir="{meshdir}" autolimits="true"/>

  <option timestep="0.002" integrator="implicitfast"/>

  <visual>
    <global offwidth="1920" offheight="1080"/>
    <quality shadowsize="4096"/>
  </visual>

  <worldbody>
    <light name="overhead" pos="0.3 0 1.2" dir="0 0 -1" directional="true"/>
    <light name="fill" pos="0.9 -0.5 0.8" dir="-0.6 0.35 -0.7" diffuse="0.3 0.3 0.3"/>
    <geom name="table" type="plane" size="0 0 0.01" rgba="0.78 0.76 0.74 1"
          pos="0 0 0" contype="1" conaffinity="1"/>

{bodies}{camera_xml}  </worldbody>
{keyframe}</mujoco>
'''
    return text, {'placed': placed, 'skipped': skipped,
                  'robot': str(robot), 'meshdir': str(meshdir),
                  'camera': bool(camera)}


def write(record, catalogue, path, camera=None, **options):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    text, detail = mjcf(record, catalogue, camera, **options)
    path.write_text(text, encoding='utf-8')
    detail['path'] = str(path)
    return detail
