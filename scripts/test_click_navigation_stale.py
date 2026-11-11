#!/usr/bin/env python3
"""Pause only the running simulation to verify stale-pose fail-stop handling."""
import argparse,json,subprocess,time,urllib.request
from pathlib import Path

p=argparse.ArgumentParser();p.add_argument('--url',default='http://localhost:8765');p.add_argument('--output',required=True)
args=p.parse_args()
def status():
    with urllib.request.urlopen(args.url+'/api/status',timeout=5) as r:return json.load(r)
def pause(value):
    return subprocess.run(['gz','service','-s','/world/chemical_plant_v4/control',
        '--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','3000',
        '--req','pause: '+('true' if value else 'false')],check=True,capture_output=True,text=True).stdout
s=status();assert s['ready']
body={'x':s['pose'][0],'y':s['pose'][1],'frame':'map','map_id':s['map_id'],'execute':True}
request=urllib.request.Request(args.url+'/api/goal',json.dumps(body).encode(),{'Content-Type':'application/json'})
with urllib.request.urlopen(request,timeout=5) as r:taskid=json.load(r)['task_id']
deadline=time.monotonic()+15
while time.monotonic()<deadline:
    s=status()
    if s['task']['state']=='executing':break
    time.sleep(.05)
assert s['task']['state']=='executing',s
try:
    receipt=pause(True);deadline=time.monotonic()+10
    while time.monotonic()<deadline:
        s=status()
        if s['task']['state']=='failed':break
        time.sleep(.2)
    assert s['task']['state']=='failed' and not s['ready'],s
    Path(args.output).write_text(json.dumps({'test':'simulation_pose_stale_fail_stop',
        'pause_receipt':receipt,'task':s['task'],'ready_reason':s['ready_reason'],'pass':True},indent=2))
    print('PASS: stale physical pose stops task and blocks unconfirmed restart',flush=True)
finally:pause(False)
