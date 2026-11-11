#!/usr/bin/env python3
"""Real ROS Action and RViz point/service integration against an idle server."""
import argparse,json,time,urllib.request
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PointStamped,PoseStamped
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger


def main():
    p=argparse.ArgumentParser();p.add_argument('--url',default='http://localhost:8765')
    p.add_argument('--prefix',default='');p.add_argument('--output',required=True)
    args=p.parse_args();records=[]
    def status():
        with urllib.request.urlopen(args.url+'/api/status',timeout=5) as r:return json.load(r)
    rclpy.init();node=Node('click_navigation_ros_acceptance')
    def wait_future(f,seconds=15):
        rclpy.spin_until_future_complete(node,f,timeout_sec=seconds)
        assert f.done(),'ROS request timeout'
        return f.result()
    client=ActionClient(node,NavigateToPose,args.prefix+'/click_navigation/navigate_to_pose')
    assert client.wait_for_server(timeout_sec=20)
    def action():
        pose=status()['pose'];goal=NavigateToPose.Goal();goal.pose.header.frame_id='map'
        goal.pose.pose.position.x,goal.pose.pose.position.y=pose[:2]
        if status()['platform']=='drone':goal.pose.pose.position.z=pose[2]
        goal.pose.pose.orientation.w=1.
        handle=wait_future(client.send_goal_async(goal))
        assert handle.accepted,'Action rejected'
        return handle
    handle=action();result=wait_future(handle.get_result_async(),45)
    assert result.status==4,result
    records.append({'test':'action_near_current_succeeded','status':result.status,'task':status()['task']})
    handle=action();cancel=wait_future(handle.cancel_goal_async())
    assert cancel.goals_canceling
    result=wait_future(handle.get_result_async(),15)
    assert result.status==5,result
    records.append({'test':'action_cancelled','status':result.status,'task':status()['task']})
    pub=node.create_publisher(PointStamped,args.prefix+'/clicked_point',10)
    execute=node.create_client(Trigger,args.prefix+'/click_navigation/execute')
    assert execute.wait_for_service(timeout_sec=10)
    previous=status()['task']['id'];pose=status()['pose']
    msg=PointStamped();msg.header.frame_id='map';msg.point.x,msg.point.y=pose[:2]
    deadline=time.monotonic()+10
    while time.monotonic()<deadline:
        pub.publish(msg);rclpy.spin_once(node,timeout_sec=.2)
        task=status()['task']
        if task['id']!=previous and task['state']=='preview':break
    assert task['id']!=previous and task['state']=='preview',task
    response=wait_future(execute.call_async(Trigger.Request()))
    assert response.success,response.message
    deadline=time.monotonic()+45
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.1);task=status()['task']
        if task['state'] in ['succeeded','failed','rejected']:break
    assert task['state']=='succeeded',task
    records.append({'test':'rviz_point_preview_and_execute_service','task':task})
    if status()['platform']=='drone':
        posepub=node.create_publisher(PoseStamped,args.prefix+'/goal_pose',10)
        msg=PoseStamped();msg.header.frame_id='map';pose=status()['pose']
        msg.pose.position.x,msg.pose.position.y=pose[:2];msg.pose.position.z=5.5;msg.pose.orientation.w=1.
        previous=task['id'];deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            posepub.publish(msg);rclpy.spin_once(node,timeout_sec=.2);task=status()['task']
            if task['id']!=previous and task['state']=='preview':break
        assert task['id']!=previous and task['state']=='preview' and task['target'][2]==5.5,task
        cancelservice=node.create_client(Trigger,args.prefix+'/click_navigation/cancel')
        assert cancelservice.wait_for_service(timeout_sec=5)
        assert wait_future(cancelservice.call_async(Trigger.Request())).success
        records.append({'test':'pose_goal_preserves_explicit_z_preview_cancel','task':task})
    Path(args.output).write_text(json.dumps(records,indent=2))
    print(json.dumps({'tests':len(records),'pass':True}),flush=True)
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
