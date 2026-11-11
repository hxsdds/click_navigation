#!/usr/bin/env python3
"""Isolated ROS-domain regression of cancellation with a slow/paused clock.

Synthetic telemetry only; this is not a Gazebo locomotion acceptance test.
"""
import json, os, signal, subprocess, time, urllib.request
from pathlib import Path
os.environ['ROS_DOMAIN_ID']='232'
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray
from control_input_msgs.msg import Inputs

root=Path(__file__).resolve().parents[1]
folder=root/os.environ.get('CLICK_NAV_TEST_ARTIFACT_DIR','test_artifacts/click_navigation');folder.mkdir(parents=True,exist_ok=True)
log=open(folder/'cancel_sim_clock.log','w')
child=subprocess.Popen(['python3','-u',str(root/'src/uav_dog_nav2/scripts/click_navigation.py'),
    '--root',str(root),'--port','8779','--ros-args','-p','use_sim_time:=true'],stdout=log,stderr=subprocess.STDOUT)
rclpy.init();node=Node('cancel_clock_injector')
odom=node.create_publisher(Odometry,'/odom',10)
trot=node.create_publisher(Float64MultiArray,'/trot_state',10)
clock=node.create_publisher(Clock,'/clock',10)
commands=[];sim=10.
node.create_subscription(Inputs,'/control_input',lambda m:commands.append([m.ly,m.rx,m.command]),10)
def feed():
    m=Odometry();m.pose.pose.position.x=-4.;m.pose.pose.position.y=-6.;m.pose.pose.orientation.w=1.;odom.publish(m)
    m=Float64MultiArray();m.data=[0.,0.,0.,0.,5.];trot.publish(m)
    m=Clock();m.clock.sec=int(sim);m.clock.nanosec=int((sim-int(sim))*1e9);clock.publish(m)
    rclpy.spin_once(node,timeout_sec=.02);time.sleep(.02)
def status():
    with urllib.request.urlopen('http://localhost:8779/api/status',timeout=2) as r:return json.load(r)
def post(path,data):
    req=urllib.request.Request('http://localhost:8779/api/'+path,json.dumps(data).encode(),{'Content-Type':'application/json'})
    with urllib.request.urlopen(req,timeout=3) as r:return json.load(r)
try:
    until=time.monotonic()+45;s={}
    while time.monotonic()<until:
        feed()
        try:s=status()
        except OSError:continue
        if s['ready']:break
    assert s.get('ready'),s
    task=post('goal',{'x':-1.,'y':-6.,'frame':'map','map_id':s['map_id'],'execute':True})['task_id']
    until=time.monotonic()+10
    while time.monotonic()<until:
        feed();s=status()
        if s['task']['state']=='executing':break
    assert s['task']['state']=='executing',s
    commands.clear();post('cancel',{'task_id':task})
    until=time.monotonic()+2.5
    while time.monotonic()<until:feed()
    frozen=status()
    assert frozen['task']['state']=='cancelling',frozen
    sim+=2.1
    until=time.monotonic()+3
    while time.monotonic()<until:
        feed();s=status()
        if s['task']['state']=='cancelled':break
    assert s['task']['state']=='cancelled' and s['ready'],s
    assert commands and all(c==[0.,0.,0] for c in commands),commands
    result={'test':'synthetic_cancel_requires_simulation_settling','pass':True,
        'frozen_clock_state':frozen['task']['state'],'after_2p1_sim_seconds':s['task'],
        'zero_velocity_without_fsm_switch':True}
    (folder/'cancel_sim_clock.json').write_text(json.dumps(result,indent=2))
    print('PASS: cancellation waits for physics time and preserves trot FSM',flush=True)
finally:
    child.send_signal(signal.SIGINT)
    try:child.wait(timeout=10)
    except subprocess.TimeoutExpired:child.kill();child.wait()
    log.close();node.destroy_node();rclpy.shutdown()
