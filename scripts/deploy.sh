#!/usr/bin/env bash
# First deployment: stop on the first failure; never start an incomplete simulator.
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/check_build_path.sh"
cd "$root"
bash scripts/install_dependencies.sh
bash scripts/setup_px4.sh
bash scripts/build.sh
source scripts/env.sh
python3 scripts/doctor.py
bash scripts/test.sh
echo 'Deployment verified. Start: CLICK_NAV_GUI=true bash scripts/start_click_navigation.sh'
