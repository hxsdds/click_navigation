#!/usr/bin/env python3
"""Exercise the real running click server and robot; never starts/teleports it."""
import argparse
import json
import math
from pathlib import Path
import time
import urllib.request
import urllib.error


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--url",default="http://localhost:8765")
    p.add_argument("--output",required=True)
    p.add_argument("--goals",default="14.5,-4;-8,-19;-32,2.5")
    p.add_argument("--skip-cancel",action="store_true")
    args=p.parse_args()
    out=Path(args.output);out.parent.mkdir(parents=True,exist_ok=True)
    events=[]
    def record(name,**fields):
        data={"test":name,"time":time.time(),**fields};events.append(data)
        out.write_text(json.dumps(events,indent=2),encoding="utf-8")
        print(json.dumps(data),flush=True)
    def get():
        with urllib.request.urlopen(args.url+"/api/status",timeout=5) as r:return json.load(r)
    def post(path,data,fail=False):
        request=urllib.request.Request(args.url+"/api/"+path,json.dumps(data).encode(),{"Content-Type":"application/json"})
        try:
            with urllib.request.urlopen(request,timeout=5) as r: result=json.load(r)
            if fail: raise AssertionError("request was not rejected")
            return result
        except urllib.error.HTTPError as exc:
            if not fail: raise
            return json.loads(exc.read())
    def wait(predicate,timeout):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            try:s=get()
            except (OSError,urllib.error.URLError):
                time.sleep(.3);continue
            if predicate(s):return s
            time.sleep(.3)
        raise AssertionError("timed out waiting for task")
    wait(lambda s:s["ready"],120)
    state=get();mapid=state["map_id"]
    def goal(x,y,execute=False,**extra):
        return {"x":x,"y":y,"frame":"map","map_id":mapid,"execute":execute,**extra}
    with urllib.request.urlopen(args.url+'/api/map') as r:mapview=json.load(r)
    negative=[('outside',100,0)]
    for label,predicate in [('unknown',lambda i:mapview['states'][i]==0),
                            ('occupied',lambda i:mapview['states'][i]==2),
                            ('clearance',lambda i:mapview['states'][i]==1 and mapview['blocked'][i])]:
        i=next(i for i in range(len(mapview['states'])) if predicate(i))
        negative.append((label,mapview['origin'][0]+(i%mapview['width']+.5)*mapview['cell'],
                        mapview['origin'][1]+(i//mapview['width']+.5)*mapview['cell']))
    for name,x,y in negative:
        record("reject_"+name,result=post("goal",goal(x,y),True))
    record("reject_frame",result=post("goal",goal(14.5,-4,frame="odom"),True))
    record("reject_map_version",result=post("goal",goal(14.5,-4,map_id="wrong"),True))
    record("reject_nonfinite",result=post("goal",goal(float("nan"),-4),True))
    if state['platform']=='drone':
        for z in (0,1.,9.4,float('nan')):
            record('reject_height',z=str(z),result=post('goal',goal(14.5,-4,z=z),True))
    xy=get()["pose"][:2] if state["platform"]=="dog" else [14.5,-4.]
    id1=post("goal",goal(*xy))["task_id"]
    s=wait(lambda s:s["task"]["state"]!="planning",30)
    assert s["task"]["state"]=="preview",s
    record("reject_busy",result=post("goal",goal(14.5,-4),True))
    record("reject_wrong_execute_id",result=post("execute",{"task_id":"wrong"},True))
    post("cancel",{"task_id":id1})
    s=wait(lambda s:s["task"]["state"]=="cancelled",5)
    record("cancel_preview",task=s["task"])
    if not args.skip_cancel:
        taskid=post("goal",goal(14.5,-4,True))["task_id"]
        wait(lambda s:s["task"]["state"]=="executing",30)
        if state["platform"]=="drone":
            airborne=wait(lambda s:s["pose"][2]>=2.5 or s["task"]["state"] in ("failed","rejected"),45)
            assert airborne["task"]["state"]=="executing",airborne
        time.sleep(4)
        record("reject_busy_while_moving",result=post("goal",goal(-8,-19),True))
        post("cancel",{"task_id":taskid})
        s=wait(lambda s:s["task"]["state"] in ("cancelled","failed"),10)
        record("cancel_motion",task=s["task"])
        assert s["task"]["state"]=="cancelled",s
        wait(lambda s:s["ready"],30)
    for coord in args.goals.split(";"):
        values=list(map(float,coord.split(',')));x,y=values[:2]
        extra={'z':values[2]} if len(values)==3 else {}
        before=get()["pose"]
        taskid=post("goal",goal(x,y,True,**extra))["task_id"]
        record("goal_started",task_id=taskid,start=before,target=values)
        s=wait(lambda s:s["task"]["state"] in ("succeeded","failed","rejected"),1400)
        record("goal_finished",task=s["task"],ready=s["ready"],ready_reason=s["ready_reason"])
        assert s["task"]["state"]=="succeeded",s
        assert s["task"]["error_m"] < (.35 if s["platform"]=="dog" else .5)
    record("suite_passed",platform=get()["platform"])


if __name__=="__main__":main()
