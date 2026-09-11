"""The launcher must recover on the next invocation after setup fails."""
import os
from pathlib import Path
import shutil
import subprocess


def test_failed_dependency_install_is_retried(tmp_path):
    root = Path(__file__).resolve().parents[1]
    shutil.copy(root/'run_vision.sh',tmp_path)
    (tmp_path/'requirements-vision.lock').write_text('dummy==1\n')
    binary = tmp_path/'bin'
    binary.mkdir()
    uv = binary/'uv'
    uv.write_text('''#!/usr/bin/env bash
set -eu
if [[ "$1" == venv ]]; then
    mkdir -p .venv-vision/bin
    printf '#!/usr/bin/env bash\\nexit 0\\n' > .venv-vision/bin/python
    chmod +x .venv-vision/bin/python
else
    echo attempt >> install-attempts
    if [[ ! -f install-failed-once ]]; then
        touch install-failed-once
        exit 7
    fi
fi
''')
    uv.chmod(0o755)
    env = {**os.environ,'PATH':str(binary)+os.pathsep+os.environ['PATH']}
    def run():return subprocess.run(['bash',str(tmp_path/'run_vision.sh'),'--help'],env=env,capture_output=True)
    assert run().returncode == 7
    assert run().returncode == 0
    assert (tmp_path/'install-attempts').read_text().splitlines() == ['attempt','attempt']
