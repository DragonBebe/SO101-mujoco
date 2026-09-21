"""Offline replay of measured ticks; does not connect to real hardware.
Run from LLM-control with PYTHONPATH=. MUJOCO_GL=egl .venv-loop/bin/python <this file>.
The jaw angle remains uncalibrated; its default zero is NOT a measurement.
"""
from pathlib import Path
import json
import numpy as np
import mujoco
from PIL import Image, ImageDraw
from realsim.arm_mapping import ArmMapping
from realsim.arm_sync import ArmMirror

out = Path(__file__).resolve().parent
session = out.parent
scene = json.loads(Path('calib/realsim-model/scene.json').read_text())
old_scene = json.loads((out / 'scene-before.json').read_text())
old = ArmMapping.from_scene(old_scene)
new = ArmMapping.from_scene(scene)
sim = ArmMirror('calib/realsim-model/scene.xml', scene, new)
renderer = mujoco.Renderer(sim.model, 540, 960)
rows = [json.loads(line) for line in (session / 'actions.jsonl').read_text().splitlines()]
report = {'source': 'recorded real Present_Position; offline model rendering, not live visual validation',
          'zero_status': 'visual_estimate', 'gripper': 'uncalibrated; model default 0, not measured',
          'frames': []}
try:
    for frame in [1, 30, 65]:
        row = next(row for row in rows if row.get('frame') == frame and 'ticks' in row)
        before, _ = old.convert(row['ticks']); after, _ = new.convert(row['ticks'])
        panel = Image.new('RGB', (2880, 580), 'white')
        photo = Image.open(session / f'{frame:04d}-env.png').resize((960, 540))
        panel.paste(photo, (0, 40))
        for i, (label, angles) in enumerate([('before', before), ('after', after)], 1):
            sim.apply(angles)
            renderer.update_scene(sim.data, camera='calibrated')
            im = Image.fromarray(renderer.render())
            im.save(out / f'{frame:04d}-{label}.png')
            panel.paste(im, (i*960, 40))
        draw = ImageDraw.Draw(panel)
        for i, label in enumerate(['REAL recorded photo', 'SIM original nominal wrist zero; jaw NOT calibrated',
                                   'SIM -90 deg VISUAL ESTIMATE; jaw NOT calibrated']):
            draw.text((i*960+10, 12), label, fill='black')
        panel.save(out / f'{frame:04d}-comparison.jpg')
        report['frames'].append({'frame': frame, 'ticks': row['ticks'],
            'before_deg': {k:float(np.degrees(v)) for k,v in before.items()},
            'after_deg': {k:float(np.degrees(v)) for k,v in after.items()}})
finally:
    renderer.close()
(out / 'comparison.json').write_text(json.dumps(report, indent=2)+'\n')
