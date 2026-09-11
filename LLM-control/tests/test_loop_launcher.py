import os
from pathlib import Path
import shutil
import subprocess


def test_launcher_uses_explicit_interpreter_from_another_directory(tmp_path):
    root = Path(__file__).resolve().parents[1]
    shutil.copy(root / 'run_loop.sh', tmp_path)
    interpreter = tmp_path / 'python with spaces'
    interpreter.write_text('#!/bin/sh\nprintf "%s\\n" "$PWD" "$@" "$PYTHONPATH"\n')
    interpreter.chmod(0o755)
    result = subprocess.run(['bash', str(tmp_path / 'run_loop.sh'), 'command', '{"action":"observe"}'],
                            cwd='/', env={**os.environ, 'SO101_PYTHON': str(interpreter)},
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    lines = result.stdout.splitlines()
    assert lines[:4] == [str(tmp_path), 'loop.py', 'command', '{"action":"observe"}']
    assert lines[4].split(os.pathsep)[0] == str(tmp_path / 'third_party/so101-nexus/src')


def test_launcher_reports_missing_environment_without_installing(tmp_path):
    root = Path(__file__).resolve().parents[1]
    shutil.copy(root / 'run_loop.sh', tmp_path)
    env = dict(os.environ)
    env.pop('SO101_PYTHON', None)
    result = subprocess.run(['bash', str(tmp_path / 'run_loop.sh'), 'status'], env=env,
                            capture_output=True, text=True)
    assert result.returncode == 1
    assert 'setup_loop.sh' in result.stderr
    assert not (tmp_path / '.venv-loop').exists()
