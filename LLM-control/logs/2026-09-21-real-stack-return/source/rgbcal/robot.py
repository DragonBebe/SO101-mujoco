"""The real SO101: reading its joints, and forward kinematics from them.

Two things that are easy to confuse are kept apart here.

*Measured* is what the servos report: raw encoder ticks.  Nothing else
about the arm is measured directly.

*Modelled* is everything downstream: the tick-to-radian scale, each
joint's sign and zero, and the pose of the gripper frame.  Those come from
a joint model plus the URDF-derived MuJoCo model, and a joint model that
has not been checked against observation is explicitly marked unverified.
Hand-eye calibration will not accept an unverified model silently: a wrong
sign or zero produces a confident extrinsic that is wrong everywhere.

The gripper frame is the model's ``gripperframe`` site, the same frame the
simulation calls the TCP.  It is a model quantity, not a measured tool
offset, and this module never claims otherwise.
"""
from dataclasses import dataclass, asdict
from pathlib import Path
import json
import os
import threading

from .bus_access import PortOwnership, serialized

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
MODEL_XML = (ROOT / 'third_party/so101-nexus/src/so101_nexus/assets/'
             'SO101_menagerie/so101.xml')
#: Feetech STS3215 control table entries this module touches.
REGISTERS = {'Homing_Offset': (31, 2), 'Min_Position_Limit': (9, 2),
             'Max_Position_Limit': (11, 2), 'Torque_Enable': (40, 1),
             'Goal_Position': (42, 2), 'Lock': (55, 1),
             'Present_Position': (56, 2)}
#: A hold that moves any joint further than this was not a hold.
HOLD_TOLERANCE_TICKS = 50
#: Feetech STS3215 resolution.  lerobot converts with ``360 / (resolution - 1)``
#: degrees per tick, so the divisor is 4095, not 4096.
TICKS_PER_TURN = 4095
#: The usual reason the bus will not open, and the one-time fix for it.
PORT_HELP = ('Check that the arm is plugged in and powered, and that your user may '
             'use the port: sudo usermod -aG dialout $USER, then log out and back '
             'in (or, for this session only, sudo chmod a+rw /dev/ttyACM0).')
#: Servo IDs as the SO101 follower is wired, matching the model's joint order.
JOINT_IDS = {'shoulder_pan': 1, 'shoulder_lift': 2, 'elbow_flex': 3,
             'wrist_flex': 4, 'wrist_roll': 5, 'gripper': 6}
#: The five joints that move the gripper frame; the sixth only opens the jaw.
ARM_JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex',
              'wrist_roll')
#: Joints whose zero offset hand-eye data can actually determine.
#:
#: The first joint turns the whole arm, so an error in its zero is exactly a
#: rotation of the camera about the base z axis: the two are one unknown, not
#: two.  The last joint spins the board about its mount, which the unknown
#: board mounting already describes.  Fitting either one anyway does not make
#: the calibration better, it makes it unidentifiable -- the solver would then
#: report a camera pose wrong by a rotation it silently traded away.  They are
#: pinned to the servo-calibrated zero instead, and any error there is absorbed,
#: consistently, by the camera pose and the board mount.
IDENTIFIABLE_OFFSETS = ('shoulder_lift', 'elbow_flex', 'wrist_flex')


#: Feetech status-packet alarm bits.
ALARM_BITS = {1: 'input voltage', 2: 'angle sensor', 4: 'overheat',
              8: 'over-current', 32: 'overload'}


def alarm_text(error):
    names = [name for bit, name in ALARM_BITS.items() if error & bit]
    return 'alarm: ' + (', '.join(names) if names else f'code {error}')


def decode_sign_magnitude(value, sign_bit=11):
    """Feetech stores some 2-byte values as sign + magnitude, not two's complement."""
    magnitude = value & (2 ** sign_bit - 1)
    return -magnitude if value & (1 << sign_bit) else magnitude


@dataclass
class JointModel:
    """Ticks to radians for one joint.

    ``q = sign * (ticks - zero_ticks) * 2 * pi / 4095``

    ``verified`` says whether this mapping has been confirmed against
    camera observation, not merely copied from a configuration file.
    """

    name: str
    servo_id: int
    zero_ticks: float = 2047.5
    sign: int = 1
    verified: bool = False
    source: str = 'nominal centre of travel, unverified'

    def radians(self, ticks):
        return self.sign * (np.asarray(ticks, dtype=float) - self.zero_ticks) \
            * 2 * np.pi / TICKS_PER_TURN

    def ticks(self, radians):
        return self.zero_ticks + self.sign * np.asarray(radians, dtype=float) \
            * TICKS_PER_TURN / (2 * np.pi)


def joint_models_from_lerobot(path):
    """Starting guess from a saved lerobot calibration, marked unverified.

    Read from lerobot's own conversion rather than guessed:
    ``write_calibration`` writes ``Homing_Offset`` into the servo, so
    ``Present_Position`` already has it applied, and the DEGREES conversion
    measures from the *midpoint of the calibrated range*, not from tick 2048.

    What this still cannot say is whether lerobot's zero pose is the zero
    pose of the kinematic model, or whether each joint turns the way the
    model's axis does.  Those are physical facts about this arm, and they
    are established by observation in :mod:`rgbcal.handeye`, not here.
    """
    data = json.loads(Path(path).read_text())
    models = {}
    for name, entry in data.items():
        midpoint = (float(entry['range_min']) + float(entry['range_max'])) / 2
        models[name] = JointModel(
            name=name, servo_id=int(entry.get('id', JOINT_IDS.get(name, 0))),
            zero_ticks=midpoint,
            # lerobot's DEGREES conversion ignores drive_mode; it is recorded
            # here only so a non-zero one is visible rather than silent.
            sign=-1 if int(entry.get('drive_mode', 0)) else 1,
            verified=False,
            source=(f'lerobot calibration {Path(path).name}: zero = midpoint of '
                    f'[{entry["range_min"]}, {entry["range_max"]}], '
                    'drive_mode ' + str(entry.get('drive_mode', 0)) + '; '
                    'unverified against the kinematic model'))
    return models


def default_joint_models():
    return {name: JointModel(name=name, servo_id=servo_id)
            for name, servo_id in JOINT_IDS.items()}


class ServoBus:
    """Access to the Feetech bus: reads, plus holding the arm where it is.

    Deliberately offers no motion command.  The only writes are ``hold`` and
    ``release``: ``hold`` stiffens the arm at the position it is already in,
    so a hand no longer has to support it while an image is taken, and
    ``release`` makes it limp again.  Neither can move the arm to a new
    place, because the only goal ever written is the present position.
    """

    def __init__(self, port='/dev/ttyACM0', baudrate=1_000_000):
        import scservo_sdk as sdk

        self._io_lock = threading.RLock()
        self._publisher = None
        self._closed = False
        self._ownership = PortOwnership(port)
        self.sdk = sdk
        self.port_handler = sdk.PortHandler(port)
        try:
            if not self.port_handler.openPort():
                raise RuntimeError(f'cannot open {port}. {PORT_HELP}')
            if not self.port_handler.setBaudRate(baudrate):
                raise RuntimeError(f'cannot set baudrate {baudrate} on {port}')
            self._ownership.attach(self.port_handler.ser)
            self.packet_handler = sdk.PacketHandler(0)
            if os.environ.get('SO101_FEEDBACK_PATH'):
                from realsim.arm_feedback import FeedbackPublisher
                self._publisher = FeedbackPublisher(
                    self, os.environ['SO101_FEEDBACK_PATH'],
                    rate=float(os.environ.get('SO101_FEEDBACK_HZ', '30')),
                    log=os.environ.get('SO101_FEEDBACK_LOG')).start()
        except Exception:
            self._ownership.close()
            if self.port_handler.ser is not None:
                self.port_handler.closePort()
            raise

    @serialized
    def _read(self, servo_id, register):
        address, size = REGISTERS[register]
        reader = (self.packet_handler.read1ByteTxRx if size == 1
                  else self.packet_handler.read2ByteTxRx)
        value, result, error = reader(self.port_handler, servo_id, address)
        if result != self.sdk.COMM_SUCCESS:
            raise RuntimeError(
                f'servo {servo_id}: {register} read failed '
                f'({self.packet_handler.getTxRxResult(result)}). Is the arm powered?')
        if error:
            raise RuntimeError(f'servo {servo_id}: {register} returned error {error}')
        return value

    @serialized
    def read_positions(self, ids=None):
        ids = ids or list(JOINT_IDS.values())
        return {servo_id: int(self._read(servo_id, 'Present_Position'))
                for servo_id in ids}

    @serialized
    def read_state(self, ids=None):
        """Positions plus the registers needed to interpret them."""
        ids = ids or list(JOINT_IDS.values())
        state = {}
        for servo_id in ids:
            state[servo_id] = {
                'present_position': int(self._read(servo_id, 'Present_Position')),
                'homing_offset': decode_sign_magnitude(
                    self._read(servo_id, 'Homing_Offset')),
                'min_position_limit': int(self._read(servo_id, 'Min_Position_Limit')),
                'max_position_limit': int(self._read(servo_id, 'Max_Position_Limit')),
                'torque_enabled': bool(self._read(servo_id, 'Torque_Enable')),
            }
        return state

    @serialized
    def _write(self, servo_id, register, value, tolerate_alarm=False):
        """Write a register.  Returns the servo's alarm byte.

        A Feetech status packet carries the servo's alarm flags (overload,
        overheat, ...) whatever instruction it answers, so a non-zero byte
        does not by itself mean this write was refused.  For writes that make
        the arm safer -- turning torque off -- an alarm must never stop the
        operation, so the caller can ask for it to be tolerated and confirm
        by reading the register back instead.
        """
        address, size = REGISTERS[register]
        writer = (self.packet_handler.write1ByteTxRx if size == 1
                  else self.packet_handler.write2ByteTxRx)
        result, error = writer(self.port_handler, servo_id, address, int(value))
        if result != self.sdk.COMM_SUCCESS:
            raise RuntimeError(f'servo {servo_id}: writing {register} failed '
                               f'({self.packet_handler.getTxRxResult(result)})')
        if error and not tolerate_alarm:
            raise RuntimeError(f'servo {servo_id}: {alarm_text(error)} while writing '
                               f'{register}')
        return error

    @serialized
    def alarms(self, ids=None):
        """Current alarm flags per servo, read with a harmless register read."""
        ids = ids or list(JOINT_IDS.values())
        found = {}
        for servo_id in ids:
            address, _ = REGISTERS['Torque_Enable']
            _, result, error = self.packet_handler.read1ByteTxRx(
                self.port_handler, servo_id, address)
            if result == self.sdk.COMM_SUCCESS and error:
                found[servo_id] = alarm_text(error)
        return found

    def torque_enabled(self, ids=None):
        ids = ids or [JOINT_IDS[name] for name in ARM_JOINTS]
        return {servo_id: bool(self._read(servo_id, 'Torque_Enable')) for servo_id in ids}

    def hold(self, ids=None):
        """Stiffen the arm exactly where it is.

        Every goal is written *before* any torque is enabled: a goal left
        over from an earlier teleoperation session would otherwise make the
        arm jump to it the instant the motor engages.  The order and the
        registers follow lerobot's own ``enable_torque``.  If any joint then
        moves further than a hold should allow, torque is dropped again and
        the call fails -- that means the goal was not what it should be.
        """
        ids = ids or [JOINT_IDS[name] for name in ARM_JOINTS]
        alarms = self.alarms(ids)
        if alarms:
            raise RuntimeError(f'not locking: servo alarms {alarms}. Let the servo '
                               'rest with torque off before trying again')
        before = self.read_positions(ids)
        for servo_id in ids:
            self._write(servo_id, 'Goal_Position', before[servo_id])
        for servo_id in ids:
            if self._read(servo_id, 'Goal_Position') != before[servo_id]:
                raise RuntimeError(f'servo {servo_id}: goal did not read back; '
                                   'torque left off')
        for servo_id in ids:
            self._write(servo_id, 'Torque_Enable', 1)
            self._write(servo_id, 'Lock', 1)
        after = self.read_positions(ids)
        moved = {servo_id: after[servo_id] - before[servo_id] for servo_id in ids}
        if max(abs(value) for value in moved.values()) > HOLD_TOLERANCE_TICKS:
            self.release(ids)
            raise RuntimeError(f'arm moved {moved} ticks when torque engaged; '
                               'torque released for safety')
        return {'before': before, 'after': after, 'moved_ticks': moved}

    def release(self, ids=None):
        """Make the arm limp again.  Whoever calls this must be supporting it.

        Never gives up on an alarm: turning torque off is the safe direction,
        so every servo is tried, alarms are collected, and the result is
        confirmed by reading ``Torque_Enable`` back.
        """
        ids = ids or [JOINT_IDS[name] for name in ARM_JOINTS]
        alarms = {}
        for servo_id in ids:
            error = self._write(servo_id, 'Torque_Enable', 0, tolerate_alarm=True)
            self._write(servo_id, 'Lock', 0, tolerate_alarm=True)
            if error:
                alarms[servo_id] = alarm_text(error)
        still_on = [servo_id for servo_id in ids
                    if self._read_tolerant(servo_id, 'Torque_Enable')]
        if still_on:
            raise RuntimeError(f'torque still on for servos {still_on} after release')
        return alarms

    @serialized
    def _read_tolerant(self, servo_id, register):
        address, size = REGISTERS[register]
        reader = (self.packet_handler.read1ByteTxRx if size == 1
                  else self.packet_handler.read2ByteTxRx)
        value, result, _ = reader(self.port_handler, servo_id, address)
        if result != self.sdk.COMM_SUCCESS:
            raise RuntimeError(f'servo {servo_id}: reading {register} failed')
        return value

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            if self._publisher:
                self._publisher.close()
        finally:
            with self._io_lock:
                self._ownership.close()
                self.port_handler.closePort()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class Kinematics:
    """Forward kinematics of the gripper frame from joint angles.

    Uses the project's own SO101 MuJoCo model, so the frame conventions
    match the simulation the control loop already uses.  ``T_base_gripper``
    maps gripper-frame coordinates into base-frame coordinates.
    """

    def __init__(self, model_path=MODEL_XML):
        import mujoco

        self.mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.joint_order = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
                            for i in range(self.model.njnt)]
        self.qadr = {name: self.model.jnt_qposadr[index]
                     for index, name in enumerate(self.joint_order)}
        self.gripper_site = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, 'gripperframe')
        self.base_site = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, 'baseframe')
        if self.gripper_site < 0 or self.base_site < 0:
            raise RuntimeError('model is missing the baseframe/gripperframe sites')

    def T_base_gripper(self, angles):
        """4x4 transform of the gripper frame in base coordinates.

        ``angles`` maps joint name to radians; missing joints stay at zero.
        """
        self.data.qpos[:] = 0
        for name, value in angles.items():
            if name in self.qadr:
                self.data.qpos[self.qadr[name]] = float(value)
        self.mujoco.mj_forward(self.model, self.data)
        transform = np.eye(4)
        transform[:3, :3] = self.data.site_xmat[self.gripper_site].reshape(3, 3)
        transform[:3, 3] = self.data.site_xpos[self.gripper_site]
        base = np.eye(4)
        base[:3, :3] = self.data.site_xmat[self.base_site].reshape(3, 3)
        base[:3, 3] = self.data.site_xpos[self.base_site]
        # Express the gripper in the base site's frame rather than the world's,
        # so the result is independent of where the model puts the robot.
        return np.linalg.inv(base) @ transform

    def joint_axis_in_base(self, joint, angles):
        """Unit axis of ``joint`` in base coordinates at this configuration."""
        index = self.joint_order.index(joint)
        self.data.qpos[:] = 0
        for name, value in angles.items():
            if name in self.qadr:
                self.data.qpos[self.qadr[name]] = float(value)
        self.mujoco.mj_forward(self.model, self.data)
        return np.array(self.data.xaxis[index], dtype=float)


def angles_from_ticks(ticks, models):
    """Joint angles in radians from a servo-id -> ticks mapping."""
    angles = {}
    for name, model in models.items():
        if model.servo_id in ticks:
            angles[name] = float(model.radians(ticks[model.servo_id]))
    return angles


def models_to_dict(models):
    return {name: asdict(model) for name, model in models.items()}


def models_from_dict(data):
    return {name: JointModel(**entry) for name, entry in data.items()}
