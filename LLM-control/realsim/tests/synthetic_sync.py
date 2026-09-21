"""Offline end-to-end probe. Generates SYNTHETIC ticks; never opens hardware.
Run from the repository root with .venv-loop/bin/python and PYTHONPATH=LLM-control.
"""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
from realsim.arm_feedback import atomic_json, clock_id
from realsim.arm_mapping import ArmMapping


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[3]
    directory = Path(args.output).resolve(); directory.mkdir(parents=True, exist_ok=True)
    scene = root / 'LLM-control/calib/realsim-model/scene.json'
    mapping = ArmMapping.from_scene(json.loads(scene.read_text()))
    launcher = ['bash', str(root / 'LLM-control/run_realsim.sh')]
    xml = directory / 'scene.xml'
    subprocess.run(launcher + ['export', '--scene', str(scene), '--output', str(xml)], check=True,
                   stdout=subprocess.DEVNULL)
    state, raw, audit = [directory / x for x in ('latest.json', 'synthetic-feedback.jsonl', 'sync-audit.jsonl')]
    gripper = directory / 'synthetic-gripper.json'
    gripper.write_text(json.dumps({'ticks': [2020, 3000], 'radians': [0, 1.2],
                                  'source': 'SYNTHETIC TEST ONLY; not a real calibration'}))
    common = ['--scene', str(scene), '--xml', str(xml), '--gripper-map', str(gripper)]
    console = open(directory / 'sync-console.jsonl', 'x')
    child = subprocess.Popen(launcher + ['sync', *common, '--state', str(state),
                                        '--record', str(audit), '--duration', '7'], stdout=console)
    try:
        time.sleep(.7)
        started = time.monotonic(); sequence = 0
        with open(raw, 'x') as log:
            while time.monotonic() - started < 5.5:
                elapsed = time.monotonic() - started
                if 2 < elapsed < 4.4:
                    time.sleep(.01); continue  # missing producer, then recovery
                sequence += 1
                ticks = {'1': 2101, '2': 2087, '3': 1984, '4': 1956, '5': 2048, '6': 2510}
                motor = str(min(5, int(elapsed * 2) % 5 + 1))
                ticks[motor] += int(100 * math.sin(elapsed * 3))
                stamp = time.monotonic()
                row = {'schema': 'realsim/arm-feedback/1', 'valid': True,
                       'source': 'SYNTHETIC TEST ONLY', 'clock_id': clock_id(),
                       'stream_id': 'synthetic-probe', 'sequence': sequence,
                       'sampled_at': time.time(), 'sampled_monotonic': stamp,
                       'received_at': time.time(), 'received_monotonic': time.monotonic(),
                       'registers': mapping.expected_registers,
                       'ticks': ticks}
                atomic_json(state, row); log.write(json.dumps(row) + '\n'); log.flush()
                time.sleep(max(0, 1 / 30 - (time.monotonic() - stamp)))
        child.wait(timeout=12)
        assert child.returncode == 0
    finally:
        if child.poll() is None: child.terminate(); child.wait()
        console.close()
    with open(directory / 'replay-console.jsonl', 'x') as console:
        subprocess.run(launcher + ['arm-replay', *common, '--log', str(raw),
                                  '--record', str(directory / 'replay-audit.jsonl')],
                       stdout=console, check=True, timeout=15)
    events = [json.loads(line) for line in audit.read_text().splitlines()]
    valid = [e for e in events if e.get('valid')]
    states = {e['status_event']['status'] for e in events if 'status_event' in e}
    assert {'live', 'stale', 'disconnected'} <= states, states
    for filename in ('sync-audit.jsonl', 'replay-audit.jsonl'):
        recorded = [json.loads(line) for line in (directory / filename).read_text().splitlines()]
        transitions = [e['status_event']['status'] for e in recorded if 'status_event' in e]
        cursor = 0
        for wanted in ('live', 'stale', 'disconnected', 'live'):
            cursor = transitions.index(wanted, cursor) + 1
        assert recorded[-1]['status_event']['error'] == 'mirror stopped'
    assert len(valid) > 50
    console_rows = [json.loads(line) for line in (directory / 'sync-console.jsonl').read_text().splitlines()]
    report = {'synthetic_only': True, 'valid_applied_samples': len(valid),
              'status_transitions_observed': sorted(states),
              'processing_ms_mean': 1000 * sum(e['processing_s'] for e in valid) / len(valid),
              'processing_ms_max': 1000 * max(e['processing_s'] for e in valid),
              'sample_age_ms_mean': 1000 * sum(e['status']['age_s'] for e in valid) / len(valid),
              'sample_age_ms_max': 1000 * max(e['status']['age_s'] for e in valid),
              'summary': console_rows[-1],
              'note': 'Headless refresh; no display scanout or hardware motion latency measured.'}
    (directory / 'synthetic-summary.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__': main()
