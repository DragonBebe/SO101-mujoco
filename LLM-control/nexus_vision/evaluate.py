"""Terminal-only evaluator. Never used to construct policy observations."""
import json
from pathlib import Path
import numpy as np


def evaluate(simulation):
    """Save upstream metrics after control has finished; no object XYZ is exported."""
    info = simulation.env._get_info()
    metrics = {}
    for key, value in info.items():
        if key == 'privileged_state' or key in ('target_index','target_object','task_potential'):
            continue
        if isinstance(value, (bool, np.bool_)):
            metrics[key] = bool(value)
        elif isinstance(value, (int, float, np.integer, np.floating)):
            metrics[key] = float(value)
    result = {'task':simulation.task, 'seed':simulation.seed, 'success':bool(info.get('success')),
              'metrics':metrics, 'policy_input':'RGB + optical-depth + camera calibration + robot proprioception',
              'upstream':'so101-nexus 0.5.4', 'simulation_time':float(simulation.data.time)}
    (Path(simulation.directory)/'evaluation.json').write_text(json.dumps(result,indent=2)+'\n')
    return result
