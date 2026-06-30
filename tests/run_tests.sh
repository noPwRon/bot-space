#!/usr/bin/env bash
# Run all tests in this project using the project virtual environment.
# Usage: bash tests/run_tests.sh
# Optional flags are passed through to pytest, e.g.:
#   bash tests/run_tests.sh -v
#   bash tests/run_tests.sh tests/test_kinematics.py

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
VENV="$PROJECT_ROOT/.venv/bin/activate"

if [ ! -f "$VENV" ]; then
    echo "ERROR: virtual environment not found at $VENV"
    exit 1
fi

source "$VENV"
cd "$PROJECT_ROOT"
pytest tests/ "$@"
