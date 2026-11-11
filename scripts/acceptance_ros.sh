#!/usr/bin/env bash
# Run after the HTTP acceptance suite: the drone must already be airborne.
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"
source scripts/env.sh
for platform in dog drone; do
    python3 scripts/test_click_navigation_ros.py --url "http://localhost:8765/$platform" \
        --prefix "/$platform" --output "test_artifacts/${platform}_ros_acceptance.json"
done
