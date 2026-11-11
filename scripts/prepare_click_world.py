#!/usr/bin/env python3
"""Use canonical audited geometry with the Go2-compatible 1 ms physics tick."""
import argparse,hashlib,json
from pathlib import Path
import xml.etree.ElementTree as ET

p=argparse.ArgumentParser();p.add_argument('source');p.add_argument('output');args=p.parse_args()
src=Path(args.source);dst=Path(args.output)
tree=ET.parse(src);world=tree.getroot().find('world');physics=world.find('physics')
physics.find('max_step_size').text='0.001'
physics.find('real_time_update_rate').text='1000'
dst.parent.mkdir(parents=True,exist_ok=True);tree.write(dst,encoding='utf-8',xml_declaration=True)
# Prove no model/collision/sensor/world-coordinate geometry was changed.
original=ET.parse(src).getroot().find('world')
for element in [world,original]:element.remove(element.find('physics'))
assert ET.tostring(world)==ET.tostring(original)
dst.with_suffix('.manifest.json').write_text(json.dumps({'source':str(src),'source_sha256':hashlib.sha256(src.read_bytes()).hexdigest(),
    'physics_step_seconds':.001,'canonical_geometry_unchanged':True},indent=2))
