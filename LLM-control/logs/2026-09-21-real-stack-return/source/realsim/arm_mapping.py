"""Explicit feedback mapping, using the calibration that produced the scene."""
from pathlib import Path
import hashlib
import json
import math
import numpy as np
from rgbcal.robot import ARM_JOINTS, JOINT_IDS, TICKS_PER_TURN


def finite_number(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


def rigid_transform(value):
    t = np.asarray(value, dtype=float)
    if (t.shape != (4, 4) or not np.isfinite(t).all()
            or not np.allclose(t[3], [0, 0, 0, 1])
            or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(t[:3, :3]), 1)):
        raise ValueError('expected a finite rigid T_base_table transform')
    return t


class ArmMapping:
    def __init__(self, entries, provenance, expected_registers):
        self.entries = entries
        self.provenance = provenance
        self.expected_registers = expected_registers
        self.gripper = None

    @classmethod
    def from_scene(cls, record):
        calibration = record['calibration']
        path = Path(calibration['sources']['extrinsics'])
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        expected = calibration.get('versions', {}).get('extrinsics')
        if not expected or not digest.startswith(expected):
            raise ValueError('scene extrinsics hash missing or changed; re-observe scene')
        extrinsics = json.loads(path.read_text())
        manifest_path = Path(extrinsics['source_session']) / 'poses.json'
        manifest = json.loads(manifest_path.read_text())
        convention = extrinsics['joint_convention']
        if not convention.get('solved') or convention['ticks_per_turn'] != TICKS_PER_TURN:
            raise ValueError('joint convention is not solved or encoder resolution differs')
        entries = {}
        corrections = record.get('model_joint_offsets', {})
        if not isinstance(corrections, dict) or set(corrections) - set(ARM_JOINTS):
            raise ValueError('model_joint_offsets must name known arm joints')
        for name in ARM_JOINTS:
            original = manifest['joint_models'][name]
            servo_id = original['servo_id']
            if servo_id != JOINT_IDS[name]:
                raise ValueError(f'{name}: motor ID conflicts with ServoBus')
            sign = convention['signs'][name]
            if sign not in (-1, 1):
                raise ValueError(f'{name}: invalid sign')
            if name in convention['zero_offsets_rad']:
                offset = convention['zero_offsets_rad'][name]
                zero_status = 'handeye_fitted'
            elif name in convention['offsets_pinned_to_servo_zero']:
                offset = 0.
                # Gauge fixing is not physical zero calibration. In particular,
                # wrist roll is inseparable from the board mounting transform.
                zero_status = 'unverified_pinned'
            else:
                raise ValueError(f'{name}: no zero offset or pinned-zero evidence')
            zero = original['zero_ticks']
            if not all(finite_number(v) for v in (offset, zero)):
                raise ValueError(f'{name}: invalid calibration')
            correction = corrections.get(name)
            model_offset = 0.
            if name in corrections:
                if (not isinstance(correction, dict)
                        or not finite_number(correction.get('radians'))
                        or abs(correction['radians']) > math.pi
                        or correction.get('status') not in ('visual_estimate', 'measured')
                        or not isinstance(correction.get('source'), str)
                        or not correction['source'].strip()):
                    raise ValueError(f'{name}: model offset needs radians, status and source')
                model_offset = correction['radians']
                zero_status = correction['status']
            entries[name] = {'servo_id': servo_id, 'joint': name, 'unit': 'ticks',
                             'zero_ticks': zero, 'sign': sign,
                             'calibration_offset_rad': offset,
                             'model_offset_rad': model_offset,
                             'zero_status': zero_status,
                             'offset_rad': offset + model_offset,
                             'radians_per_tick': 2 * math.pi / TICKS_PER_TURN,
                             'raw_range': [0, 4095]}
        return cls(entries, {'extrinsics': str(path), 'extrinsics_sha256': digest,
                            'manifest': str(manifest_path),
                            'model_joint_offsets': corrections,
                            'manifest_sha256': hashlib.sha256(manifest_path.read_bytes()).hexdigest()},
                   manifest['servo_state_at_start'])

    def set_gripper(self, config):
        ticks, radians = config['ticks'], config['radians']
        if (len(ticks) != 2 or len(radians) != 2
                or not all(finite_number(v) for v in ticks + radians)
                or ticks[0] >= ticks[1] or not 0 <= ticks[0] < ticks[1] <= 4095
                or radians[0] == radians[1] or not config.get('source')):
            raise ValueError('gripper needs two ordered measured ticks, angles and source')
        self.gripper = dict(config)

    def check_registers(self, registers):
        if not isinstance(registers, dict):
            raise ValueError('calibration registers must be a mapping')
        for servo_id, expected in self.expected_registers.items():
            actual = registers.get(str(servo_id), registers.get(int(servo_id)))
            if not isinstance(actual, dict):
                raise ValueError(f'missing servo {servo_id} calibration registers')
            for key in ('homing_offset', 'min_position_limit', 'max_position_limit'):
                if actual.get(key) != expected[key]:
                    raise ValueError(f'servo {servo_id}: {key} differs from hand-eye capture')

    def convert(self, ticks):
        if not isinstance(ticks, dict):
            raise ValueError('ticks must be a mapping keyed by motor ID')
        values = {}
        for name, servo_id in JOINT_IDS.items():
            value = ticks.get(str(servo_id), ticks.get(servo_id))
            if not finite_number(value) or not 0 <= value <= 4095 or value != int(value):
                raise ValueError(f'{name}: missing or invalid Present_Position ticks')
            values[name] = value
        q = {name: e['sign'] * (values[name] - e['zero_ticks']) * e['radians_per_tick']
             + e['offset_rad'] for name, e in self.entries.items()}
        grip = {'ticks': values['gripper'], 'status': 'uncalibrated'}
        if self.gripper:
            a, b = self.gripper['ticks']; x, y = self.gripper['radians']
            if not a <= values['gripper'] <= b:
                raise ValueError('gripper feedback outside measured calibration interval')
            q['gripper'] = x + (values['gripper'] - a) / (b - a) * (y - x)
            grip.update(status='calibrated', radians=q['gripper'], source=self.gripper['source'])
        return q, grip
