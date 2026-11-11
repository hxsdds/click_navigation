#!/usr/bin/env python3
"""Read-only portability/dependency checks; never starts or moves a robot."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source-only', action='store_true')
    args = parser.parse_args()
    checks = []
    def check(name, passed, detail=''):
        checks.append(bool(passed))
        print(f'{"OK" if passed else "FAIL"}: {name}' + (f' — {detail}' if detail else ''))
    meta = json.loads((ROOT/'config/static_navigation_map.json').read_text())
    for key in ('known_grid', 'ground_grid', 'world', 'octomap'):
        f = ROOT/meta[key]['file']
        check(key, f.is_file() and hashlib.sha256(f.read_bytes()).hexdigest() == meta[key]['sha256'])
    packages = sorted(ROOT.glob('src/**/package.xml'))
    check('source packages', len(packages) >= 8, str(len(packages)))
    check('ROS-independent planner', (ROOT/'scripts/plan_3d.py').is_file())
    if not args.source_only:
        deps = Path(os.environ.get('PX4_DEPS_DIR', ROOT/'_deps')).resolve()
        check('workspace built', (ROOT/'install/local_setup.bash').is_file())
        for command in ('ros2', 'gz'):
            check(command, shutil.which(command) is not None)
        for name in ('rclpy', 'numpy', 'px4_msgs.msg', 'control_input_msgs.msg', 'nav2_msgs.action'):
            try:
                __import__(name)
                check(name, True)
            except ImportError as exc:
                check(name, False, str(exc))
        for rel in ('px4_runtime_v1.17.0/bin/px4', 'px4_runtime_v1.17.0/bin/px4-alias.sh',
                    'px4_runtime_v1.17.0/bin/px4-commander', 'agent_system_build/MicroXRCEAgent',
                    'px4_runtime_v1.17.0/models/x500/model.sdf', 'px4_runtime_v1.17.0/models/x500_base/model.sdf'):
            check(rel, (deps/rel).is_file())
        for rel in ('px4_runtime_v1.17.0/bin/px4','agent_system_build/MicroXRCEAgent'):
            binary = deps/rel
            if binary.is_file() and shutil.which('ldd'):
                result = subprocess.run(['ldd',str(binary)], capture_output=True,text=True)
                check('dynamic libraries '+rel, result.returncode == 0 and 'not found' not in result.stdout,
                      result.stdout.strip() if 'not found' in result.stdout else '')
    return 0 if all(checks) else 1

if __name__ == '__main__':
    raise SystemExit(main())
