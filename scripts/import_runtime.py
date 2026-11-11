#!/usr/bin/env python3
"""Copy a verified local PX4/Agent cache; no links to the source workspace."""
import argparse
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--from', dest='source', required=True, type=Path)
    parser.add_argument('--to', dest='destination', type=Path, default=ROOT/'_deps')
    args = parser.parse_args()
    source, destination = args.source.resolve(), args.destination.resolve()
    if source == destination or source in destination.parents or destination in source.parents:
        raise ValueError('Source and destination must be separate directories')
    runtime = source/'px4_runtime_v1.17.0'
    agent = source/'agent_system_build'
    required = [runtime/'bin/px4', runtime/'bin/px4-alias.sh', runtime/'bin/px4-commander',
                runtime/'rootfs/etc/init.d-posix/rcS',
                runtime/'models/x500/model.sdf',runtime/'models/x500_base/model.sdf',
                agent/'MicroXRCEAgent',agent/'libmicroxrcedds_agent.so.2.4.3']
    for path in required:
        if not path.is_file():
            raise FileNotFoundError(path)
    target = destination/'px4_runtime_v1.17.0'
    (target/'bin').mkdir(parents=True,exist_ok=True)
    for path in (runtime/'bin').iterdir():
        dest = target/'bin'/path.name
        if path.is_symlink():
            # PX4 is a multicall executable. Keep links inside the new bin,
            # rather than resolving each into a 55 MB duplicate or losing it.
            if path.resolve() != (runtime/'bin/px4').resolve():
                raise ValueError(f'Unsupported external binary link: {path}')
            if dest.is_symlink() and dest.readlink() == Path('px4'):
                continue
            if dest.exists() or dest.is_symlink():
                raise FileExistsError(dest)
            dest.symlink_to('px4')
        elif path.is_file():
            shutil.copy2(path,dest)
    for name in ('rootfs/etc','models/x500','models/x500_base'):
        shutil.copytree(runtime/name,target/name,symlinks=False,dirs_exist_ok=True)
    (target/'plugins').mkdir(exist_ok=True)
    for path in (runtime/'plugins').glob('*.so'):
        shutil.copy2(path,target/'plugins'/path.name)
    shutil.copy2(runtime/'server.config',target/'server.config')
    agent_target = destination/'agent_system_build'
    agent_target.mkdir(parents=True,exist_ok=True)
    for name in ('MicroXRCEAgent','libmicroxrcedds_agent.so',
                 'libmicroxrcedds_agent.so.2.4','libmicroxrcedds_agent.so.2.4.3'):
        shutil.copy2(agent/name,agent_target/name,follow_symlinks=True)
    # Do not copy gz_env.sh: it embeds the old absolute path. env.sh computes paths.
    record = {'method':'local runtime import','px4_ref':'v1.17.0','agent_ref':'v2.4.3',
              'independent_files':True,'runtime_source':str(source)}
    (destination/'runtime_import.json').write_text(json.dumps(record,indent=2)+'\n')
    print(f'Runtime copied to {destination}. Run: source scripts/env.sh; python3 scripts/doctor.py')

if __name__ == '__main__':
    main()
