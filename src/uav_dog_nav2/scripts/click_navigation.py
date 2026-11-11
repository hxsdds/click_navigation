#!/usr/bin/env python3
"""Persistent simulation-only click navigation with HTTP, RViz and ROS Action."""
import argparse
import collections
import concurrent.futures
import copy
import fcntl
import json
import math
import os
import signal
import subprocess
from pathlib import Path
import sys
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit, parse_qs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default=str(Path(__file__).resolve().parents[3]))
    parser.add_argument("--platform", choices=["dog", "drone"], default="dog")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--known-grid", default=None,help='explicit test map; default validates config/static_navigation_map.json')
    parser.add_argument("--origin-x", type=float, default=-4.)
    parser.add_argument("--origin-y", type=float, default=-6.)
    parser.add_argument("--origin-z", type=float, default=-.013)
    parser.add_argument("--require-ground-truth", action="store_true")
    parser.add_argument("--shared-world", action="store_true")
    parser.add_argument("--dog-speed", type=float, default=.30)
    parser.add_argument("--dog-yaw-speed", type=float, default=.10)
    parser.add_argument("--drone-speed", type=float, default=.6)
    parser.add_argument("--auto-follow", action="store_true")
    args, ros_args = parser.parse_known_args()
    if not (0<args.dog_speed<=.30 and 0<args.dog_yaw_speed<=.10):
        parser.error("dog speed must be in (0,0.30], yaw speed in (0,0.10]")
    if not 0<args.drone_speed<=1.:
        parser.error("drone trajectory speed must be in (0,1.0]")
    root = Path(args.root).resolve()
    sys.path.insert(0, str(root/"scripts"))
    from click_nav_core import GridMap, DogMotion, FlightTrajectory, TERMINAL, segment_distance, target_coordinates, resolve_navigation_grid
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from rclpy.action import ActionServer, GoalResponse, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from control_input_msgs.msg import Inputs
    from geometry_msgs.msg import PointStamped, PoseStamped
    from nav_msgs.msg import Odometry, OccupancyGrid, Path as RosPath
    from nav2_msgs.action import NavigateToPose
    from std_msgs.msg import String, Float64MultiArray
    from std_srvs.srv import Trigger

    # Hold an OS lock for the process lifetime; never allow two navigation
    # owners in the same ROS domain, even on different HTTP ports/platforms.
    lockfile = open(f"/tmp/uav_dog_click_nav_{os.environ.get('ROS_DOMAIN_ID','0')}_{args.platform}.lock", "w")
    fcntl.flock(lockfile, fcntl.LOCK_EX | fcntl.LOCK_NB)
    grid_file,map_metadata=resolve_navigation_grid(root,args.known_grid)
    grid = GridMap(grid_file, args.platform)
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)

    class Navigation(Node):
        def __init__(self):
            super().__init__("click_navigation_"+args.platform)
            self.lock = threading.RLock()
            self.pool = concurrent.futures.ThreadPoolExecutor(max_workers=1)
            self.future = None
            self.cancel_event = threading.Event()
            self.pose = None
            self.pose_seen = self.fsm_seen = self.status_seen = self.truth_seen = 0.
            self.fsm = None
            self.tilt = self.peak_tilt = 0.
            self.yaw_rate=0.
            self.truth = self.status = None
            self.task = None
            self.other_pose,self.other_seen = None,0.
            self.mission_lock = open(f"/tmp/uav_dog_click_mission_{os.environ.get('ROS_DOMAIN_ID','0')}.lock","w")
            self.owns_mission = False
            self.history = collections.deque(maxlen=30)
            self.results = collections.OrderedDict()
            self.motion = None
            self.stop_until = 0.
            self.stop_sim_started = 0.
            self.stop_pose = None
            self.stop_samples = collections.deque(maxlen=30)
            self.stopped_state = "cancelled"
            self.blocked_reason = ""
            self.logdir = root/"test_artifacts"/"click_navigation"/time.strftime("%Y%m%d_%H%M%S")
            self.logdir.mkdir(parents=True, exist_ok=True)
            self.trace_file = None
            self.last_trace = 0.
            self.init_until = 0.
            self.init_command = 0
            self.init_stage = "wait"
            self.init_started = time.monotonic()
            self.stream_enabled = False
            self.stream_target = None
            self.last_command = 0.
            self.ever_offboard = False
            self.reached_at = None
            self.mission_start = 0.
            self.waypoint_index = 0
            self.flight_path = []
            self.flight_progress_pose = None
            self.flight_progress_at = 0.
            self.heartbeat_stop = threading.Event()
            self.stream_count=0;self.stream_max_gap=0.;self.last_stream_sent=0.
            prefix = "/"+args.platform if args.shared_world else ""
            self.pub_status = self.create_publisher(String, prefix+"/click_navigation/status", 10)
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pub_path = self.create_publisher(RosPath, prefix+"/click_navigation/path", latched)
            self.pub_pose = self.create_publisher(PoseStamped, prefix+"/click_navigation/pose", 10)
            self.pub_map = self.create_publisher(OccupancyGrid, prefix+"/click_navigation/map", latched)
            view = grid.view(.5 if args.platform=="dog" else 6.)
            map_msg = OccupancyGrid();map_msg.header.frame_id="map"
            map_msg.info.resolution=.3;map_msg.info.width=view["width"];map_msg.info.height=view["height"]
            map_msg.info.origin.position.x=-52.;map_msg.info.origin.position.y=-40.
            map_msg.info.origin.orientation.w=1.
            map_msg.data=[-1 if v==0 else 100 if v==2 else 0 for v in view["states"]]
            self.pub_map.publish(map_msg)
            self.create_subscription(PointStamped, prefix+"/clicked_point", self.clicked, 10)
            self.create_subscription(PoseStamped, prefix+"/goal_pose", self.goal_pose, 10)
            self.create_service(Trigger, prefix+"/click_navigation/execute", self.execute_service)
            self.create_service(Trigger, prefix+"/click_navigation/cancel", self.cancel_service)
            if args.platform == "dog":
                self.dog_pub = self.create_publisher(Inputs, "/control_input", 10)
                self.create_subscription(Odometry, "/odom", self.odom, 10)
                self.create_subscription(Float64MultiArray, "/trot_state", self.trot, 10)
            else:
                from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
                self.PX4Status, self.PX4Command = VehicleStatus, VehicleCommand
                self.OffboardMode, self.Setpoint = OffboardControlMode, TrajectorySetpoint
                self.px4_origin = [args.origin_x,args.origin_y,args.origin_z]
                self.offboard_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
                self.setpoint_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)
                self.command_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
                qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.VOLATILE)
                self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self.local_position, qos)
                self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", self.vehicle_status, qos)
                self.create_subscription(Odometry, "/px4/ground_truth/odometry", self.ground_truth, qos)
                self.heartbeat = threading.Thread(target=self.stream, daemon=True)
                self.heartbeat.start()
            self.action_ids = {}
            if args.shared_world:
                other_topic="/px4/ground_truth/odometry" if args.platform=="dog" else "/odom"
                other_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
                self.create_subscription(Odometry,other_topic,self.other_robot,other_qos)
            self.action = ActionServer(self, NavigateToPose, prefix+"/click_navigation/navigate_to_pose",
                execute_callback=self.action_execute, goal_callback=self.action_goal,
                cancel_callback=self.action_cancel, callback_group=ReentrantCallbackGroup())
            # Watchdogs must still run if Gazebo /clock pauses or disappears.
            self.wall_clock=Clock(clock_type=ClockType.STEADY_TIME)
            self.create_timer(.05, self.tick, clock=self.wall_clock)
            self.create_timer(.25, self.publish_status, clock=self.wall_clock)

        def other_robot(self,msg):
            p=msg.pose.pose.position
            with self.lock:
                self.other_pose=[p.x,p.y,p.z if args.platform=="dog" else .5]
                self.other_seen=time.monotonic()

        def odom(self, msg):
            p,q = msg.pose.pose.position,msg.pose.pose.orientation
            vals = [p.x,p.y,q.x,q.y,q.z,q.w]
            if not all(math.isfinite(v) for v in vals): return
            with self.lock:
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                roll = math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
                pitch = math.asin(max(-1., min(1.,2*(q.w*q.y-q.z*q.x))))
                self.pose = [p.x,p.y,.5,yaw]  # ground planning slice, not base height
                self.odom_tilt = max(abs(roll),abs(pitch))
                self.pose_seen = time.monotonic()

        def trot(self,msg):
            if len(msg.data)<5 or not all(math.isfinite(v) for v in msg.data[:5]): return
            with self.lock:
                self.fsm = int(round(msg.data[4]))
                self.tilt = max(abs(msg.data[2]),abs(msg.data[3]))
                self.yaw_rate=msg.data[1]
                self.fsm_seen = time.monotonic()

        def local_position(self,msg):
            if not (msg.xy_valid and msg.z_valid and all(math.isfinite(v) for v in [msg.x,msg.y,msg.z])): return
            with self.lock:
                self.pose = [args.origin_x+msg.y,args.origin_y+msg.x,args.origin_z-msg.z,0.]
                self.pose_seen = time.monotonic()

        def vehicle_status(self,msg):
            with self.lock:
                self.status,self.status_seen = msg,time.monotonic()

        def ground_truth(self,msg):
            p = msg.pose.pose.position
            if not all(math.isfinite(v) for v in [p.x,p.y,p.z]): return
            with self.lock:
                self.truth,self.truth_seen = [p.x,p.y,p.z],time.monotonic()

        def ready_reason(self):
            now = time.monotonic()
            if self.blocked_reason: return self.blocked_reason
            if self.pose is None or now-self.pose_seen>1.5: return "position unavailable or stale"
            if args.shared_world and (self.other_pose is None or now-self.other_seen>1.5): return "other robot position unavailable or stale"
            if args.platform == "dog":
                if now-self.fsm_seen>1.5: return "gait telemetry stale"
                if self.init_stage!="done": return "initializing trot"
                if self.fsm!=5: return "initializing trot" if self.init_stage!="done" else "gait left trot"
                if max(self.tilt,getattr(self,"odom_tilt",0))>.25: return "tilt exceeded 0.25 rad"
                if self.count_publishers("/control_input")>1: return "another control_input publisher is active"
            else:
                if self.status is None or now-self.status_seen>1.5: return "PX4 status stale"
                if args.require_ground_truth and (self.truth is None or now-self.truth_seen>1.5): return "ground truth stale"
                if args.require_ground_truth and math.dist(self.truth,self.pose[:3])>.75: return "localization disagrees with physical pose"
                if self.count_publishers("/fmu/in/trajectory_setpoint")>1: return "another Offboard publisher is active"
            return ""

        def busy(self):
            return bool(self.task and self.task["state"] not in TERMINAL)

        def snapshot(self):
            with self.lock:
                reason = self.ready_reason()
                return {"platform":args.platform,"map_id":grid.map_id,"frame":"map",
                        "ground_projection_id":grid.ground_projection_id,"static_map_only":True,
                        "map_metadata":map_metadata,
                        "ready":not bool(reason),"ready_reason":reason,
                        "pose":copy.deepcopy(self.pose),"fsm":self.fsm,"tilt_rad":self.tilt,
                        "other_pose":copy.deepcopy(self.other_pose),"shared_world":args.shared_world,
                        "nav_state":self.status.nav_state if self.status else None,
                        "task":copy.deepcopy(self.task),"history":list(self.history),
                        "target_height":self.task['target'][2] if self.task else (.5 if args.platform=='dog' else 6.)}

        def submit(self,data):
            with self.lock:
                if self.busy() or (self.future and not self.future.done()): raise ValueError("busy; cancel the previous task first")
                reason = self.ready_reason()
                if reason: raise ValueError(reason)
                if data.get("frame","map")!="map": raise ValueError("only map frame is accepted; transform coordinates before submitting")
                if data.get("map_id")!=grid.map_id: raise ValueError("map version mismatch; reload the map")
                target = target_coordinates(data,args.platform)
                grid.check(target,"goal")
                start = list(self.pose[:3])
                takeoff = args.platform=="drone" and start[2]<.8
                if takeoff:
                    if args.shared_world and math.dist(self.pose[:2],self.other_pose[:2])<1.3:
                        raise ValueError("takeoff column too close to the other robot")
                    start[2] = 2.7
                    grid.check_takeoff(start[:2])
                grid.check(start,"start")
                self.cancel_event = threading.Event()
                self.task = {"id":uuid.uuid4().hex,"state":"planning","target":target,
                    "platform":args.platform,
                    "start":start,"path":[],"map_id":grid.map_id,"reason":"",
                    "physical_start":list(self.pose[:3]),
                    "created":time.time(),"auto_execute":bool(data.get("execute",False)),
                    "takeoff":takeoff,"error_m":None,"peak_tilt_rad":0.,
                    "takeoff_xy":start[:2] if takeoff else None,
                    "static_map_only":True,"height_limits_m":[1.2,9.3] if args.platform=='drone' else None}
                self.task["nav_settings"]={"dog_vx_max":args.dog_speed,"dog_yaw_max":args.dog_yaw_speed,
                    "planning_radius_m":grid.radius,"target_height_m":target[2],
                    "px4_map_origin":[args.origin_x,args.origin_y,args.origin_z],
                    "require_ground_truth":args.require_ground_truth,"shared_world":args.shared_world}
                self.task["nav_settings"]["drone_trajectory_speed_m_s"]=args.drone_speed
                self.task["nav_settings"]["vertical_speed_limit_m_s"]=.4
                self.task["max_localization_error_m"]=0.
                task_id = self.task["id"]
                self.peak_tilt = 0.
                obstacles=[]
                if args.shared_world:
                    centre=list(self.other_pose)
                    if args.platform=="dog": centre[2]=max(.5,centre[2])
                    obstacles=[(centre,1.3)]
                self.task["obstacles"]=obstacles
                self.future = self.pool.submit(grid.plan,start,target,self.cancel_event,25,obstacles)
                self.future.add_done_callback(lambda f: self.planned(task_id,f))
                self.record("submitted")
                return task_id

        def planned(self,task_id,future):
            with self.lock:
                if not self.task or self.task["id"]!=task_id or self.task["state"]!="planning": return
                try:
                    self.task["path"] = future.result()
                    self.task["length_m"] = sum(math.dist(a,b) for a,b in zip(self.task["path"][:-1],self.task["path"][1:]))
                    self.task["state"] = "preview"
                    heights=[p[2] for p in self.task['path']]
                    self.task['height_range_m']=[min(heights),max(heights)]
                    self.publish_path()
                    self.record("planned")
                    if self.task["auto_execute"]: self.execute(task_id)
                except Exception as exc:
                    self.finish("rejected",str(exc))

        def execute(self,task_id):
            with self.lock:
                if not self.task or self.task["id"]!=task_id or self.task["state"]!="preview": raise ValueError("no matching preview task")
                reason = self.ready_reason()
                if reason: raise ValueError(reason)
                expected = list(self.pose[:3])
                if self.task["takeoff"]: expected[2]=2.7
                if math.dist(expected,self.task["start"])>.25:
                    self.finish("rejected","robot moved after preview; select and plan again")
                    raise ValueError(self.task["reason"])
                if args.shared_world and self.task.get("obstacles"):
                    other=list(self.other_pose)
                    if args.platform=="dog": other[2]=max(.5,other[2])
                    if math.dist(other,self.task["obstacles"][0][0])>.25:
                        raise ValueError("other robot moved after preview; plan again")
                try: fcntl.flock(self.mission_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
                except BlockingIOError: raise ValueError("other platform is navigating; wait for it to stop")
                self.owns_mission=True
                now = time.monotonic()
                self.task["state"] = "executing"
                self.task["started"] = time.time()
                self.trace_file = open(self.logdir/(task_id+".jsonl"),"a",encoding="utf-8")
                if args.platform=="dog":
                    self.motion = DogMotion(self.task["path"],self.task["target"],self.get_clock().now().nanoseconds/1e9,
                                            args.dog_speed,args.dog_yaw_speed)
                else:
                    self.flight_path = copy.deepcopy(self.task["path"])
                    self.flight_path[-1] = self.task["target"]
                    self.waypoint_index = 0
                    self.mission_start,self.flight_progress_at = now,now
                    self.flight_progress_pose = list(self.pose[:3])
                    self.ever_offboard = False
                    self.reached_at = None
                    self.stream_target = list(self.pose[:3])
                    self.trajectory=FlightTrajectory(self.stream_target,args.drone_speed)
                    self.setpoint_sim_time=self.get_clock().now().nanoseconds/1e9
                    self.stream_enabled = True
                    self.stream_count=0;self.stream_max_gap=0.;self.last_stream_sent=0.
                self.record("executing")
                if args.auto_follow:
                    threading.Thread(target=self.follow_for_task,args=(task_id,),daemon=True).start()

        def focus_camera(self):
            model="go2" if args.platform=="dog" else "x500_0"
            with self.lock:
                source=self.truth if args.platform=="drone" and self.truth else self.pose
                point=list(source[:3]) if source else None
            requests=[("/gui/follow","gz.msgs.StringMsg",f'data: "{model}"')]
            if point:
                # Changing follow offset alone preserves the previous view
                # direction. Set a matching look-at pose as well, so switching
                # vehicles cannot leave the new target outside the frame.
                yaw=math.pi/4;pitch=math.atan2(4,math.sqrt(18))
                sy,cy=math.sin(yaw/2),math.cos(yaw/2)
                sp,cp=math.sin(pitch/2),math.cos(pitch/2)
                x,y,z=point
                pose=f'pose {{ position {{ x: {x-3} y: {y-3} z: {z+4} }} orientation {{ x: {-sy*sp} y: {cy*sp} z: {sy*cp} w: {cy*cp} }} }}'
                requests.extend([("/gui/follow/offset","gz.msgs.Vector3d","x: -3 y: -3 z: 4"),
                                 ("/gui/move_to/pose","gz.msgs.GUICamera",pose)])
            else:
                requests.insert(0,("/gui/move_to","gz.msgs.StringMsg",f'data: "{model}"'))
            for service,request_type,request in requests:
                try:
                    result=subprocess.run(["gz","service","-s",service,"--reqtype",request_type,
                        "--reptype","gz.msgs.Boolean","--timeout","1500","--req",request],
                        check=True,capture_output=True,text=True,timeout=4)
                except (OSError,subprocess.SubprocessError) as exc:
                    raise ValueError("Gazebo camera request failed; open its GUI first") from exc
                if "true" not in result.stdout.lower():
                    raise ValueError("Gazebo camera is unavailable; open its GUI first")
            return {"ok":True,"model":model}

        def follow_for_task(self,task_id):
            try:self.focus_camera()
            except Exception as exc:
                self.get_logger().warning(f"Camera follow unavailable: {exc}")
                with self.lock:
                    if self.task and self.task["id"]==task_id:self.task["camera_warning"]=str(exc)

        def cancel(self,task_id=None):
            with self.lock:
                if not self.busy(): return False
                if task_id and task_id!=self.task["id"]: raise ValueError("task ID mismatch")
                self.cancel_event.set()
                if self.task["state"] in ("planning","preview"):
                    self.finish("cancelled","cancelled before motion")
                elif self.task["state"] not in ("stopping","cancelling"):
                    self.begin_stop("cancelled","cancel requested")
                return True

        def begin_stop(self,state,reason):
            self.stopped_state = state
            self.task["stop_trigger_reason"] = reason
            self.task["reason"] = reason
            self.task["state"] = "cancelling" if state=="cancelled" else "stopping"
            # Motion settling follows physics time; telemetry watchdogs and
            # the bounded stop deadline must continue to use steady time.
            self.stop_until = time.monotonic()+ (3. if args.platform=="drone" else 15.)
            self.stop_sim_started = self.get_clock().now().nanoseconds/1e9
            self.stop_pose = list(self.pose[:3]) if self.pose else None
            self.stop_samples.clear()
            if args.platform=="dog":
                # A normal cancellation uses the same controlled zero-velocity
                # stance as a completed route. Switching FSM mid-stride and
                # restarting trot changes foot references unnecessarily.
                self.send_dog(command=2 if state=="failed" and self.fsm==5 else 0)
            else:
                self.stream_target = list(self.pose[:3]) if self.pose else self.stream_target
                self.hold()
            self.record("stopping")

        def finish(self,state,reason=""):
            self.task["state"],self.task["reason"] = state,reason
            self.task["finished"] = time.time()
            self.task["final_pose"] = list(self.pose) if self.pose else None
            self.task["peak_tilt_rad"] = self.peak_tilt
            if args.platform=="drone":
                self.task["heartbeat_count"]=self.stream_count
                self.task["heartbeat_max_wall_gap_s"]=self.stream_max_gap
            if self.pose:
                self.task["error_m"] = math.dist(self.pose[:2 if args.platform=="dog" else 3],
                                                self.task["target"][:2 if args.platform=="dog" else 3])
            self.record("finished")
            (self.logdir/(self.task["id"]+".json")).write_text(json.dumps(self.task,indent=2),encoding="utf-8")
            self.history.append({k:self.task.get(k) for k in ["id","state","target","error_m","reason"]})
            self.results[self.task["id"]] = copy.deepcopy(self.task)
            while len(self.results)>30: self.results.popitem(last=False)
            if self.trace_file: self.trace_file.close(); self.trace_file=None
            self.motion = None
            if self.owns_mission:
                fcntl.flock(self.mission_lock,fcntl.LOCK_UN);self.owns_mission=False

        def record(self,event,command=None):
            if self.trace_file:
                self.trace_file.write(json.dumps({"event":event,"wall_time":time.time(),
                    "monotonic":time.monotonic(),"sim_ns":self.get_clock().now().nanoseconds,
                    "task_id":self.task["id"],"state":self.task["state"],"pose":self.pose,
                    "truth":self.truth,"tilt":self.tilt,"command":command,
                    "reason":self.task.get("reason","")})+"\n")
                self.trace_file.flush()

        def send_dog(self,vx=0.,yaw=0.,command=0):
            msg = Inputs()
            msg.ly,msg.rx,msg.command = float(vx/.30),float(yaw/.10),command
            self.dog_pub.publish(msg)

        def initialize_dog(self,now):
            if self.blocked_reason:return
            if self.init_stage=="done": return
            if self.pose is None or now-self.fsm_seen>1.5: return
            if self.count_publishers("/control_input")>1:
                self.blocked_reason="another control_input publisher is active"; return
            if now-self.init_started>90:
                self.blocked_reason="trot initialization timeout; restart after checking simulator"; return
            if self.fsm==5:
                self.init_stage="done"; return
            if now<self.init_until:
                self.send_dog(command=self.init_command if self.init_stage=="pulse" else 0)
                return
            if self.init_stage=="pulse":
                self.init_stage,self.init_command,self.init_until="wait",0,now+3.
                return
            self.init_stage,self.init_command,self.init_until="pulse",4 if self.fsm==3 else 2,now+.3

        def stamp(self):
            # PX4 v1.17 ucdr_deserialize treats a zero timestamp as
            # hrt_absolute_time() at receipt (Tools/msg/templates/ucdr/msg.h.em).
            # Echoing outbound timestamps reuses the previous XRCE offset;
            # a time-sync adjustment can then age a fresh heartbeat by seconds.
            # Receipt stamping preserves PX4's normal arrival-rate failsafe.
            return 0

        def px4_command(self,cmd,p1=0.,p2=0.,p3=0.):
            msg=self.PX4Command()
            msg.timestamp=self.stamp();msg.command=cmd
            msg.param1,msg.param2,msg.param3=float(p1),float(p2),float(p3)
            msg.target_system=msg.target_component=msg.source_system=msg.source_component=1
            msg.from_external=True
            self.command_pub.publish(msg);self.last_command=time.monotonic()

        def hold(self):
            if self.status and self.status.arming_state==self.PX4Status.ARMING_STATE_ARMED:
                self.px4_command(self.PX4Command.VEHICLE_CMD_DO_SET_MODE,1,4,3)

        def stream(self):
            while not self.heartbeat_stop.wait(.05):
                with self.lock:
                    if not self.stream_enabled or self.stream_target is None: continue
                    target=list(self.stream_target);stamp=self.stamp()
                mode=self.OffboardMode();mode.timestamp=stamp;mode.position=True
                sp=self.Setpoint();sp.timestamp=stamp
                sp.position=[target[1]-args.origin_y,target[0]-args.origin_x,args.origin_z-target[2]]
                sp.velocity=sp.acceleration=sp.jerk=[math.nan]*3
                sp.yaw=sp.yawspeed=math.nan
                try:
                    self.offboard_pub.publish(mode);self.setpoint_pub.publish(sp)
                    sent=time.monotonic()
                    if self.last_stream_sent:self.stream_max_gap=max(self.stream_max_gap,sent-self.last_stream_sent)
                    self.last_stream_sent=sent;self.stream_count+=1
                except Exception:
                    if not self.heartbeat_stop.is_set(): self.get_logger().error("Offboard stream publication failed")

        def drone_tick(self,now):
            S=self.PX4Status
            sim_time=self.get_clock().now().nanoseconds/1e9
            dt=sim_time-self.setpoint_sim_time;self.setpoint_sim_time=sim_time
            if now-self.mission_start>max(180.,self.task["length_m"]/args.drone_speed*2.5+60): raise ValueError("flight timeout")
            if now-self.mission_start<2.: return
            if self.status.nav_state!=S.NAVIGATION_STATE_OFFBOARD:
                if self.ever_offboard: raise ValueError("PX4 unexpectedly exited Offboard")
                if now-self.mission_start>20: raise ValueError("Offboard entry timed out")
                if now-self.last_command>1: self.px4_command(self.PX4Command.VEHICLE_CMD_DO_SET_MODE,1,6)
                return
            self.ever_offboard=True
            if self.status.arming_state!=S.ARMING_STATE_ARMED:
                if now-self.mission_start>20: raise ValueError("arming timed out")
                if now-self.last_command>1: self.px4_command(self.PX4Command.VEHICLE_CMD_COMPONENT_ARM_DISARM,1)
                return
            target=self.flight_path[self.waypoint_index]
            self.stream_target=self.trajectory.tick(target,self.pose[:3],dt)
            if math.dist(self.pose[:3],self.flight_progress_pose)>.15:
                self.flight_progress_pose,self.flight_progress_at=list(self.pose[:3]),now
            if now-self.flight_progress_at>30: raise ValueError("flight stopped making progress")
            final=self.waypoint_index==len(self.flight_path)-1
            if self.trajectory.done and math.dist(self.pose[:3],target)<(.20 if not final else .45):
                if not final:
                    self.waypoint_index+=1
                elif not args.require_ground_truth or math.dist(self.truth,self.task["target"])<.5:
                    if self.reached_at is None: self.reached_at=now
                    if now-self.reached_at>=2: self.begin_stop("succeeded","goal reached; confirming Hold")
            else: self.reached_at=None

        def tick(self):
            with self.lock:
                now=time.monotonic()
                if args.platform=="dog": self.initialize_dog(now)
                if not self.busy():
                    if args.platform=="dog" and self.init_stage=="done": self.send_dog()
                    return
                state=self.task["state"]
                if args.platform=="dog":
                    self.peak_tilt=max(self.peak_tilt,self.tilt,getattr(self,"odom_tilt",0))
                if state in ("planning","preview"): return
                if state in ("stopping","cancelling"):
                    if args.platform=="dog":
                        self.send_dog()
                        sim=self.get_clock().now().nanoseconds/1e9
                        if self.pose:self.stop_samples.append((sim,list(self.pose[:2])))
                        recent=[p for t,p in self.stop_samples if sim-t<.5]
                        moving=len(recent)<2 or math.dist(recent[0],recent[-1])>.04
                        stale=self.pose is None or now-self.pose_seen>1.5 or now-self.fsm_seen>1.5
                        unstable=max(self.tilt,getattr(self,"odom_tilt",0))>.15
                        drift=self.pose is not None and math.dist(self.pose[:2],self.stop_pose[:2])>.30
                        settled=sim-self.stop_sim_started>=2. and not (moving or stale or unstable or drift)
                        if settled:
                            if self.stopped_state=="failed":self.blocked_reason="navigation fault; inspect robot before restarting"
                            self.finish(self.stopped_state,self.task["reason"])
                        elif stale or drift or now>=self.stop_until:
                            self.blocked_reason="stop not confirmed; check robot before restarting navigation"
                            self.finish("failed",self.blocked_reason)
                    else:
                        if now-self.last_command>.4: self.hold()
                        confirmed=self.status and now-self.status_seen<1.5 and (
                            self.status.nav_state==self.PX4Status.NAVIGATION_STATE_AUTO_LOITER or
                            self.status.arming_state!=self.PX4Status.ARMING_STATE_ARMED)
                        if confirmed:
                            self.stream_enabled=False
                            end=self.stopped_state
                            if end=="succeeded" and (now-self.pose_seen>1.5 or math.dist(self.pose[:3],self.task["target"])>=.5):
                                end="failed";self.task["reason"]="Hold confirmed but final goal error exceeded tolerance"
                            self.finish(end,self.task["reason"])
                        elif now>=self.stop_until:
                            self.blocked_reason="Hold not confirmed; inspect PX4 failsafe before restarting"
                            self.stream_enabled=False
                            self.finish("failed",self.blocked_reason)
                    return
                try:
                    if args.platform=="dog":
                        self.peak_tilt=max(self.peak_tilt,self.tilt,getattr(self,"odom_tilt",0))
                    elif args.require_ground_truth and self.pose and self.truth:
                        self.task["max_localization_error_m"]=max(self.task["max_localization_error_m"],math.dist(self.pose[:3],self.truth))
                    reason=self.ready_reason()
                    if reason: raise ValueError(reason)
                    point=list(self.pose[:3])
                    if args.shared_world and math.dist(point,self.other_pose)<1.0:
                        raise ValueError("other robot physical clearance is insufficient")
                    low_takeoff=(args.platform=='drone' and self.task['takeoff'] and point[2]<1.2
                                 and math.dist(point[:2],self.task['takeoff_xy'])<=.25)
                    if not low_takeoff and (args.platform=="dog" or point[2]>=.8):
                        if not grid.safe(point,physical=True): raise ValueError("left observed-free physical clearance")
                    truth_low_takeoff=(args.require_ground_truth and self.task['takeoff'] and self.truth[2]<1.2
                                       and math.dist(self.truth[:2],self.task['takeoff_xy'])<=.25)
                    if args.require_ground_truth and not truth_low_takeoff and self.truth[2]>=.8 and not grid.safe(self.truth,physical=True):
                        raise ValueError("physical truth left observed-free clearance")
                    if args.platform=="dog":
                        self.peak_tilt=max(self.peak_tilt,self.tilt,getattr(self,"odom_tilt",0))
                        if self.motion.deviation(self.pose[:2])>.30: raise ValueError("tracking deviation exceeded 0.30 m")
                        vx,yaw,done=self.motion.tick([self.pose[0],self.pose[1],self.pose[3]],self.get_clock().now().nanoseconds/1e9)
                        self.send_dog(vx,yaw)
                        if now-self.last_trace>.2: self.record("sample",[vx,yaw]);self.last_trace=now
                        if done: self.finish("succeeded","settled goal reached")
                    else:
                        self.drone_tick(now)
                        if now-self.last_trace>.2: self.record("sample",self.stream_target);self.last_trace=now
                except Exception as exc:
                    self.begin_stop("failed",str(exc))

        def publish_path(self):
            msg=RosPath();msg.header.frame_id="map";msg.header.stamp=self.get_clock().now().to_msg()
            for point in self.task["path"]:
                pose=PoseStamped();pose.header=msg.header
                pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=point
                pose.pose.orientation.w=1.;msg.poses.append(pose)
            self.pub_path.publish(msg)

        def publish_status(self):
            msg=String();msg.data=json.dumps(self.snapshot(),allow_nan=False);self.pub_status.publish(msg)
            if self.pose:
                p=PoseStamped();p.header.frame_id="map";p.header.stamp=self.get_clock().now().to_msg()
                p.pose.position.x,p.pose.position.y,p.pose.position.z=self.pose[:3]
                p.pose.orientation.z=math.sin(self.pose[3]/2);p.pose.orientation.w=math.cos(self.pose[3]/2)
                self.pub_pose.publish(p)

        def clicked(self,msg):
            try: self.submit({"x":msg.point.x,"y":msg.point.y,"frame":msg.header.frame_id,"map_id":grid.map_id})
            except Exception as exc: self.get_logger().warning(f"Click rejected: {exc}")

        def goal_pose(self,msg):
            try:self.submit({'x':msg.pose.position.x,'y':msg.pose.position.y,'z':msg.pose.position.z,
                             'frame':msg.header.frame_id,'map_id':grid.map_id})
            except Exception as exc:self.get_logger().warning(f'Pose goal rejected: {exc}')

        def execute_service(self,request,response):
            try:
                self.execute(self.task["id"] if self.task else None);response.success=True;response.message="executing"
            except Exception as exc: response.message=str(exc)
            return response

        def cancel_service(self,request,response):
            response.success=self.cancel();response.message="cancel requested" if response.success else "no active task"
            return response

        def action_goal(self,goal):
            return GoalResponse.ACCEPT if not self.busy() and not self.ready_reason() and goal.pose.header.frame_id=="map" else GoalResponse.REJECT

        def action_cancel(self,handle):
            task_id=self.action_ids.get(bytes(handle.goal_id.uuid))
            if task_id: self.cancel(task_id)
            return CancelResponse.ACCEPT

        def action_execute(self,handle):
            key=bytes(handle.goal_id.uuid)
            result=NavigateToPose.Result()
            try:
                p=handle.request.pose.pose.position
                task_id=self.submit({"x":p.x,"y":p.y,"z":p.z,"frame":"map","map_id":grid.map_id,"execute":True})
                self.action_ids[key]=task_id
                while rclpy.ok():
                    if handle.is_cancel_requested: self.cancel(task_id)
                    snap=self.snapshot();task=snap["task"]
                    if task["id"]!=task_id:
                        with self.lock: task=copy.deepcopy(self.results.get(task_id))
                        if task is None: raise ValueError("task result unavailable")
                    if task["state"] in TERMINAL: break
                    feedback=NavigateToPose.Feedback()
                    feedback.current_pose.header.frame_id="map"
                    if snap["pose"]:
                        feedback.current_pose.pose.position.x,feedback.current_pose.pose.position.y,feedback.current_pose.pose.position.z=snap["pose"][:3]
                        feedback.current_pose.pose.orientation.w=1.
                        dims=2 if args.platform=='dog' else 3
                        feedback.distance_remaining=float(math.dist(snap["pose"][:dims],task["target"][:dims]))
                    handle.publish_feedback(feedback);time.sleep(.1)
                if task["state"]=="succeeded": handle.succeed()
                elif task["state"]=="cancelled" and handle.is_cancel_requested: handle.canceled()
                else:
                    if hasattr(result,"error_code"): result.error_code=1
                    if hasattr(result,"error_msg"): result.error_msg=task["reason"]
                    handle.abort()
            except Exception as exc:
                if hasattr(result,"error_code"): result.error_code=1
                if hasattr(result,"error_msg"): result.error_msg=str(exc)
                handle.abort()
            finally: self.action_ids.pop(key,None)
            return result

    node=Navigation()
    map_cache={}
    occupied=np.argwhere(grid.states==2)
    stride=max(1,math.ceil(len(occupied)/25000))
    scene_bytes=json.dumps({'cell':.3,'origin':[-52,-40,0],'map_id':grid.map_id,
                           'display_stride':stride,'occupied_voxels':occupied[::stride].tolist()}).encode()
    html=(Path(__file__).resolve().parent/"click_navigation.html").read_bytes()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*unused): pass
        def reply(self,code,payload,mime="application/json"):
            body=payload if isinstance(payload,bytes) else json.dumps(payload,allow_nan=False).encode()
            self.send_response(code);self.send_header("Content-Type",mime+"; charset=utf-8")
            self.send_header("Content-Length",str(len(body)));self.send_header("Cache-Control","no-store")
            self.send_header("X-Content-Type-Options","nosniff");self.end_headers()
            try: self.wfile.write(body)
            except (BrokenPipeError,ConnectionResetError): pass

        def do_GET(self):
            path=urlsplit(self.path)
            if path.path=="/": self.reply(200,html,"text/html")
            elif path.path=="/api/map":
                try:
                    z=float(parse_qs(path.query).get('z',[.5 if args.platform=='dog' else 6.])[0])
                    if not math.isfinite(z) or not 0<=z<10.8:raise ValueError('height outside map')
                    key=grid.index([0,0,z])[2]
                    with node.lock:
                        if key not in map_cache:map_cache[key]=json.dumps(grid.view(z)).encode()
                    self.reply(200,map_cache[key])
                except ValueError as exc:self.reply(400,{'error':str(exc)})
            elif path.path=="/api/scene": self.reply(200,scene_bytes)
            elif path.path=="/api/status": self.reply(200,node.snapshot())
            else: self.reply(404,{"error":"not found"})

        def do_POST(self):
            # Same-origin local UI only. Reject browser cross-origin writes.
            if self.headers.get("Origin") and self.headers["Origin"] not in (f"http://127.0.0.1:{args.port}",f"http://localhost:{args.port}"):
                return self.reply(403,{"error":"origin rejected"})
            try:
                if self.headers.get_content_type()!="application/json": raise ValueError("JSON content type required")
                size=int(self.headers.get("Content-Length","0"))
                if not 0<size<=4096: raise ValueError("invalid request length")
                data=json.loads(self.rfile.read(size))
                if not isinstance(data,dict): raise ValueError("JSON object required")
                if self.path=="/api/goal": result={"task_id":node.submit(data)}
                elif self.path=="/api/execute": node.execute(data.get("task_id"));result={"ok":True}
                elif self.path=="/api/cancel": result={"ok":node.cancel(data.get("task_id"))}
                elif self.path=="/api/camera": result=node.focus_camera()
                else: return self.reply(404,{"error":"not found"})
                self.reply(200,result)
            except (ValueError,KeyError,TypeError) as exc: self.reply(409,{"error":str(exc)})

    server=ThreadingHTTPServer(("127.0.0.1",args.port),Handler)
    server.daemon_threads=True
    threading.Thread(target=server.serve_forever,daemon=True).start()
    executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    print(f"CLICK_NAV_READY http://localhost:{args.port} platform={args.platform} map={grid.map_id}",flush=True)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally:
        node.cancel()
        until=time.monotonic()+(16 if args.platform=="dog" else 4)
        while node.busy() and rclpy.ok() and time.monotonic()<until:
            executor.spin_once(timeout_sec=.05)
        node.heartbeat_stop.set()
        if args.platform=="drone": node.heartbeat.join(timeout=1)
        else: node.send_dog()
        node.cancel_event.set();node.pool.shutdown(wait=True,cancel_futures=True)
        server.shutdown();server.server_close()
        executor.shutdown();node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=="__main__": main()
