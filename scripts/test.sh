#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"
source scripts/env.sh
mkdir -p test_artifacts
for script in scripts/*.sh; do bash -n "$script"; done
python3 -m compileall -q scripts src/uav_dog_nav2/scripts
python3 -m unittest discover -s tests -v 2>&1 | tee test_artifacts/unit_tests.log
python3 scripts/doctor.py --source-only
