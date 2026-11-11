#!/usr/bin/env python3
"""Independent saved click-task map/SDF audit, never used by the controller."""
import argparse
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
from audit_world_clearance import surfaces


def distances(points,t,g):
    q=(points-t[:3,3])@t[:3,:3]
    box=g.find('box/size')
    if box is not None:
        d=np.abs(q)-np.array([float(v) for v in box.text.split()])/2
        return np.linalg.norm(np.maximum(d,0),axis=1)+np.minimum(d.max(axis=1),0)
    cylinder=g.find('cylinder')
    if cylinder is not None:
        d=np.stack([np.linalg.norm(q[:,:2],axis=1)-float(cylinder.findtext('radius')),
                    np.abs(q[:,2])-float(cylinder.findtext('length'))/2],axis=1)
        return np.linalg.norm(np.maximum(d,0),axis=1)+np.minimum(d.max(axis=1),0)
    sphere=g.find('sphere/radius')
    if sphere is not None:return np.linalg.norm(q,axis=1)-float(sphere.text)
    return np.full(len(points),np.inf)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--acceptance',required=True)
    parser.add_argument('--platform',choices=['dog','drone'],required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--known-grid',default='maps/chemical_plant_v4_static11_known_0p3.bin')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    sys.path.insert(0,str(root/'src/uav_dog_nav2/scripts'))
    from click_nav_core import GridMap
    grid=GridMap(root/args.known_grid,args.platform)
    events=json.loads(Path(args.acceptance).read_text())
    tasks=[e['task'] for e in events if e['test'] in ('goal_finished','cancel_motion')]
    shapes=[(name,t,g) for kind,name,t,g in surfaces(ET.parse(root/'worlds/chemical_plant_v4.world').getroot().find('world'))
            if kind=='collision' and g is not None]
    rows=[]
    for task in tasks:
        files=list((root/'test_artifacts/click_navigation').glob('*/'+task['id']+'.jsonl'))
        if len(files)!=1:raise ValueError('expected exactly one trace for '+task['id'])
        samples=[json.loads(line) for line in files[0].read_text().splitlines()]
        points=[s['pose'][:3] if args.platform=='dog' else s['truth'] for s in samples if s.get('pose')]
        if args.platform=='drone':points=[p for p in points if p and p[2]>=.8]
        if not points and task['state']=='succeeded':raise ValueError('no physical samples')
        dense=points[:1]
        for a,b in zip(points[:-1],points[1:]):
            count=max(1,math.ceil(math.dist(a,b)/.03))
            dense.extend([[x+(y-x)*j/count for x,y in zip(a,b)] for j in range(1,count+1)])
        # Launch column is independently checked down to 15 cm before arming;
        # the airborne sphere intersects the floor while lifting off.
        low_takeoff=lambda p: args.platform=='drone' and task.get('takeoff') and p[2]<1.2 and math.dist(p[:2],task['takeoff_xy'])<=.25
        unsafe=sum(not grid.safe(p,physical=True) and not low_takeoff(p) for p in dense)
        clearance=[(name,float(distances(np.array(dense),t,g).min())) for name,t,g in shapes] if dense else []
        minimum=min((d for _,d in clearance),default=None)
        non_ground=min((d for name,d in clearance if name.split('/')[1]!='ground'),default=None)
        radius=.35 if args.platform=='dog' else .65
        final_truth=samples[-1].get('truth') if samples else None
        final=task['final_pose'][:2] if args.platform=='dog' else final_truth
        error=math.dist(final,task['target'][:len(final)]) if final else None
        passed=unsafe==0 and (non_ground is None or non_ground>=radius)
        passed &= task['peak_tilt_rad']<=.25
        if task['state']=='succeeded':passed &= error is not None and error<(.35 if args.platform=='dog' else .5)
        row={'task_id':task['id'],'platform':args.platform,'state':task['state'],'goal':task['target'],
             'samples':len(points),'unsafe_map_samples':unsafe,'minimum_sdf_clearance_m':minimum,
             'checked_points_including_3cm_interpolation':len(dense),'minimum_non_ground_sdf_clearance_m':non_ground,
             'physical_final_error_m':error,'peak_tilt_rad':task['peak_tilt_rad'],'pass':bool(passed)}
        rows.append(row);print(json.dumps(row),flush=True)
    unsupported=[name for name,_,g in shapes if not any(g.find(kind) is not None for kind in ('box','cylinder','sphere'))]
    Path(args.output).write_text(json.dumps({'map_id':grid.map_id,'unsupported_collision_shapes':unsupported,
        'dog_z_is_nominal_planning_slice':args.platform=='dog','interpolation_spacing_m':.03,'results':rows},indent=2))
    return 0 if rows and all(r['pass'] for r in rows) else 1


if __name__=='__main__':raise SystemExit(main())
