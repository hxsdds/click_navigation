#!/usr/bin/env python3
"""Synthetic UI-only fixture: no ROS, Gazebo, vehicle or actuator access.

Open localhost:8789, request camera follow, then cancel while camera waits.
The fixture checks that cancellation arrives before the delayed camera reply.
"""
import json, threading, time
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
root=Path(__file__).resolve().parents[1]
html=(root/'src/uav_dog_nav2/scripts/click_navigation.html').read_text().replace(
    '<h1>厂区点击导航</h1>','<h1>界面隔离测试 · 不控制机器人</h1>')
mapdata={'map_id':'synthetic_ui_fixture','frame':'map','cell':.3,'origin':[-52,-40,0],
    'width':346,'height':266,'z':.5,'states':[1]*(346*266),'blocked':[0]*(346*266)}
state={'platform':'dog','map_id':mapdata['map_id'],'ready':True,'ready_reason':'',
    'pose':[-4,-6,.5,0],'history':[],'task':{'id':'synthetic_ui_task','state':'executing',
    'target':[14.5,-4,.5],'path':[[-4,-6,.5],[14.5,-4,.5]],'length_m':18.6,'error_m':None,'reason':''}}
events=[];camera_done=threading.Event();cancelled=threading.Event()
class Handler(BaseHTTPRequestHandler):
    def log_message(self,*args):pass
    def reply(self,payload,mime='application/json'):
        data=payload.encode() if isinstance(payload,str) else json.dumps(payload).encode()
        self.send_response(200);self.send_header('Content-Type',mime);self.send_header('Content-Length',str(len(data)));self.end_headers()
        try:self.wfile.write(data)
        except (BrokenPipeError,ConnectionResetError):pass
    def do_GET(self):
        if self.path=='/':self.reply(html,'text/html; charset=utf-8')
        elif self.path=='/api/map':self.reply(mapdata)
        elif self.path=='/api/status':self.reply(state)
        else:self.send_error(404)
    def do_POST(self):
        self.rfile.read(int(self.headers.get('Content-Length','0')))
        if self.path=='/api/camera':
            events.append({'event':'camera_requested','time':time.monotonic()})
            time.sleep(8)
            events.append({'event':'camera_finished','time':time.monotonic()});camera_done.set();self.reply({'ok':True})
        elif self.path=='/api/cancel':
            events.append({'event':'cancel_received','time':time.monotonic()})
            state['task'].update(state='cancelled',reason='cancel requested',error_m=18.6)
            cancelled.set();self.reply({'ok':True})
        else:self.send_error(409)
server=ThreadingHTTPServer(('127.0.0.1',8789),Handler);server.daemon_threads=True
threading.Thread(target=server.serve_forever,daemon=True).start()
print('UI-only fixture http://localhost:8789 (no robot control)',flush=True)
try:
    until=time.monotonic()+120
    while time.monotonic()<until and not (camera_done.is_set() and cancelled.is_set()):time.sleep(.1)
    times={e['event']:e['time'] for e in events}
    passed=all(k in times for k in ['camera_requested','cancel_received','camera_finished']) and times['camera_requested']<times['cancel_received']<times['camera_finished']
    output={'test':'synthetic_ui_cancel_during_camera_wait','pass':passed,'events':events,'controls_real_robots':False}
    (root/'test_artifacts/click_navigation/ui_camera_cancel_fixture_20261109.json').write_text(json.dumps(output,indent=2))
    assert passed,output
    print('PASS: UI cancellation remains available during the camera request',flush=True)
    time.sleep(5)
finally:server.shutdown();server.server_close()
