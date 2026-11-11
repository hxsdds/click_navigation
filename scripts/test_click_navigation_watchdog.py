#!/usr/bin/env python3
"""Isolated ROS domain fault injection; does not move either Gazebo robot."""
import json,os,signal,subprocess,time,urllib.request
from pathlib import Path
os.environ['ROS_DOMAIN_ID']='231'
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray
from control_input_msgs.msg import Inputs

root=Path(__file__).resolve().parents[1]
folder=root/os.environ.get('CLICK_NAV_TEST_ARTIFACT_DIR','test_artifacts/click_navigation');folder.mkdir(parents=True,exist_ok=True)
output=folder/'watchdog_clock_pause.json'
log=open(folder/'watchdog_clock_pause.log','w')
child=subprocess.Popen(['python3','-u',str(root/'src/uav_dog_nav2/scripts/click_navigation.py'),
    '--root',str(root),'--port','8778','--ros-args','-p','use_sim_time:=true'],stdout=log,stderr=subprocess.STDOUT)
rclpy.init();node=Node('click_watchdog_injector')
odom=node.create_publisher(Odometry,'/odom',10)
trot=node.create_publisher(Float64MultiArray,'/trot_state',10)
clock=node.create_publisher(Clock,'/clock',10)
controls=[]
node.create_subscription(Inputs,'/control_input',lambda m:controls.append([time.monotonic(),m.ly,m.rx]),10)
def feed():
    msg=Odometry();msg.header.frame_id='map';msg.pose.pose.position.x=-4.;msg.pose.pose.position.y=-6.;msg.pose.pose.orientation.w=1.;odom.publish(msg)
    state=Float64MultiArray();state.data=[0.,0.,0.,0.,5.];trot.publish(state)
    stamp=Clock();t=time.monotonic();stamp.clock.sec=int(t);stamp.clock.nanosec=int(t%1*1e9);clock.publish(stamp)
    rclpy.spin_once(node,timeout_sec=.02)
def status():
    with urllib.request.urlopen('http://localhost:8778/api/status',timeout=2) as r:return json.load(r)
try:
    deadline=time.monotonic()+30
    while time.monotonic()<deadline:
        feed()
        try:
            s=status()
            if s['ready']:break
        except OSError:pass
        time.sleep(.02)
    assert s['ready'],s
    request=urllib.request.Request('http://localhost:8778/api/goal',json.dumps({'x':-1,'y':-6,'frame':'map','map_id':s['map_id'],'execute':True}).encode(),{'Content-Type':'application/json'})
    with urllib.request.urlopen(request,timeout=3) as r:task_id=json.load(r)['task_id']
    deadline=time.monotonic()+5
    while time.monotonic()<deadline:
        feed();s=status()
        if s['task']['state']=='executing':break
    assert s['task']['state']=='executing',s
    # Freeze /clock and both telemetry streams. The control timer must keep
    # running on a steady clock and publish zero, even with frozen sim time.
    deadline=time.monotonic()+8
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.1);s=status()
        if s['task']['state']=='failed':break
    assert s['task']['state']=='failed' and not s['ready'],s
    assert controls and controls[-1][1:]==[0.,0.],controls[-5:]
    output.write_text(json.dumps({'test':'isolated_ros_clock_and_telemetry_pause','pass':True,
        'task':s['task'],'ready_reason':s['ready_reason'],'last_commands':controls[-10:]},indent=2))
    print('PASS: watchdog runs with frozen simulation clock and sends zero motion',flush=True)
finally:
    child.send_signal(signal.SIGINT)
    try:child.wait(timeout=10)
    except subprocess.TimeoutExpired:child.kill();child.wait()
    log.close();node.destroy_node();rclpy.shutdown()
