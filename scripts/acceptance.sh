#!/usr/bin/env bash
# Real SITL movement tests. Run only after the shared demo is ready.
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"
source scripts/env.sh
mode="${1:-both}"
if [[ "$mode" != dog && "$mode" != drone && "$mode" != both ]]; then
    echo 'Usage: bash scripts/acceptance.sh [dog|drone|both]' >&2; exit 2
fi
if [[ "$mode" == dog || "$mode" == both ]]; then
    python3 scripts/test_click_navigation_http.py --url http://localhost:8765/dog \
        --goals='-1,-6;-1,-3;-4,-6' --output test_artifacts/dog_acceptance.json
    python3 scripts/audit_click_navigation.py --platform dog \
        --acceptance test_artifacts/dog_acceptance.json --output test_artifacts/dog_audit.json
fi
if [[ "$mode" == drone || "$mode" == both ]]; then
    python3 scripts/test_click_navigation_http.py --url http://localhost:8765/drone \
        --goals='0,-4,3;0,-15,2' --output test_artifacts/drone_acceptance.json
    python3 scripts/audit_click_navigation.py --platform drone \
        --acceptance test_artifacts/drone_acceptance.json --output test_artifacts/drone_audit.json
fi
