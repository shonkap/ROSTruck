#!/bin/bash
# Build + unit tests + lint for the whole workspace. Run inside the sim (or dev) container:
#   ./scripts/run_tests.sh
# Add --smoke to also run the ~10 min simulator end-to-end test.
set -e
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
cd "$REPO/ros2_ws"
colcon build --symlink-install 2>&1 | grep -E "Summary|Failed|ERROR" || true
source install/setup.bash
echo "== unit tests"
# -p no:anyio: the anthropic SDK ships an anyio pytest plugin that breaks Ubuntu 22.04's pytest
python3 -m pytest -q -p no:anyio -p no:cacheprovider src/*/test
echo "== lint (pyflakes)"
python3 -m pyflakes src/*/fc_bridge src/*/truck_* src/*/launch "$REPO"/scripts/*.py && echo "lint OK"
echo "== shell syntax"
for f in "$REPO"/scripts/*.sh "$REPO"/sim/*.sh "$REPO"/docker/*.sh; do bash -n "$f"; done && echo "shell OK"
if [ "${1:-}" = "--smoke" ]; then
    echo "== simulator smoke test"
    python3 "$REPO/scripts/sim_smoke_test.py"
fi
