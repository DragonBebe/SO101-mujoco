#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if [[ ! -x .venv-loop/bin/python ]]; then
    if command -v uv >/dev/null 2>&1; then
        uv venv --python "${SO101_BASE_PYTHON:-3.12}" .venv-loop
    else
        "${SO101_BASE_PYTHON:-python3}" -m venv .venv-loop
    fi
fi
if command -v uv >/dev/null 2>&1; then
    uv pip install --python .venv-loop/bin/python -r requirements-loop.txt
else
    .venv-loop/bin/python -m pip install -r requirements-loop.txt
fi
echo 'Ready: bash LLM-control/run_loop.sh serve --viewer --realtime --camera-viewer'
