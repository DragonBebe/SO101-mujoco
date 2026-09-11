#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if [[ ! -x .venv-vision/bin/python ]]; then
    uv venv --python 3.12 .venv-vision
fi
if ! cmp -s requirements-vision.lock .venv-vision/.installed-requirements; then
    uv pip install --python .venv-vision/bin/python -r requirements-vision.lock
    cp -- requirements-vision.lock .venv-vision/.installed-requirements
fi
exec .venv-vision/bin/python vision.py "$@"
