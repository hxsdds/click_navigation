"""ROS-independent map contract and motion primitives for click navigation."""
import hashlib
import json
import itertools
import math
import time
from pathlib import Path
import numpy as np
import plan_3d
from plan_goal_3d import inflate, simplify_grid_path

CELL = .3
ORIGIN = np.array([-52., -40., 0.])
SHAPE = (346, 266, 36)
TERMINAL = {"succeeded", "cancelled", "failed", "rejected"}


def resolve_navigation_grid(root, override=None):
    root=Path(root)
    if override:return root/override,None
    metadata=json.loads((root/'config/static_navigation_map.json').read_text())
    for field in ('known_grid','ground_grid','world'):
        path=root/metadata[field]['file']
        if hashlib.sha256(path.read_bytes()).hexdigest()!=metadata[field]['sha256']:
            raise ValueError(f'{field} changed since map validation; rebuild and validate the map')
    if metadata['cell_m']!=CELL or tuple(metadata['shape'])!=SHAPE or metadata['origin']!=ORIGIN.tolist():
        raise ValueError('map metadata does not match navigation grid contract')
    return root/metadata['known_grid']['file'],metadata

def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))

def segment_distance(p, a, b):
    p, a, b = np.array(p), np.array(a), np.array(b)
    d = b-a
    t = np.clip(np.dot(p-a,d)/max(1e-12,np.dot(d,d)),0,1)
    return float(np.linalg.norm(p-a-t*d))

def execution_points(points, spacing=1.5):
    result = [list(points[0])]
    for a,b in zip(points[:-1],points[1:]):
        n = max(1,math.ceil(math.dist(a,b)/spacing))
        result.extend([[x+(y-x)*i/n for x,y in zip(a,b)] for i in range(1,n+1)])
    return result

def advance_setpoint(point,target,actual,dt,speed=.6,max_lead=.6,vertical_speed=.4):
    """Bound trajectory speed in simulation time and position lead."""
    delta=np.asarray(target)-np.asarray(point);length=float(np.linalg.norm(delta))
    if length<1e-9:return list(target)
    duration=max(length/speed,abs(delta[2])/vertical_speed)
    candidate=np.asarray(point)+delta*min(1.,max(0.,min(dt,.2))/duration)
    return list(candidate) if np.linalg.norm(candidate-np.asarray(actual))<=max_lead else list(point)


def target_coordinates(data, platform):
    target=[float(data['x']),float(data['y']),.5 if platform=='dog' else float(data.get('z',6.))]
    if not all(math.isfinite(v) for v in target):raise ValueError('invalid coordinate')
    if platform=='drone' and not 1.2<=target[2]<=9.3:
        raise ValueError('goal: flight height must be between 1.2 and 9.3 m; landing is not a navigation goal')
    return target


class FlightTrajectory:
    """Rest-to-rest trapezoids along checked straight segments, in sim time.

    Pausing the virtual clock when tracking lead grows preserves the same
    checked segment. No corner cutting or online replanning is performed.
    """
    def __init__(self, point, speed=.6, vertical_speed=.4, acceleration=.35):
        self.point=list(point);self.origin=list(point);self.target=None
        self.speed=speed;self.vertical_speed=vertical_speed;self.acceleration=acceleration
        self.elapsed=0.;self.done=True

    def tick(self,target,actual,dt,max_lead=.6):
        if self.target!=list(target):
            if not self.done:raise ValueError('trajectory corner changed before stop')
            self.origin=list(self.point);self.target=list(target);self.elapsed=0.;self.done=False
        delta=np.array(target)-self.origin;length=float(np.linalg.norm(delta))
        if length<1e-9:self.point=list(target);self.done=True;return self.point
        vmax=min(self.speed,self.vertical_speed*length/max(abs(delta[2]),1e-12))
        a=self.acceleration;ta=min(vmax/a,math.sqrt(length/a));v=a*ta
        tc=max(0.,(length-a*ta*ta)/v);total=2*ta+tc
        t=min(total,self.elapsed+max(0.,min(dt,.2)))
        if t<ta:distance=.5*a*t*t
        elif t<ta+tc:distance=.5*a*ta*ta+v*(t-ta)
        else:distance=length-.5*a*(total-t)**2
        candidate=(np.array(self.origin)+delta*distance/length).tolist()
        if math.dist(candidate,actual)<=max_lead:
            self.point=candidate;self.elapsed=t;self.done=t>=total
        return list(self.point)

class GridMap:
    def __init__(self, filename, platform):
        raw = Path(filename).read_bytes()
        if len(raw) != math.prod(SHAPE):
            raise ValueError("map size does not match 0.3 m map contract")
        self.states = np.frombuffer(raw,dtype=np.uint8).reshape(SHAPE)
        if not np.isin(self.states,[0,1,2]).all():
            raise ValueError("invalid map state")
        self.map_id = hashlib.sha256(raw).hexdigest()
        self.platform = platform
        self.radius = .65 if platform == "dog" else .95
        self.blocked = inflate(self.states != 1,self.radius,CELL,
                               vertical_radius=.3 if platform == "dog" else None)
        self.physical = inflate(self.states != 1,.35 if platform == "dog" else .65,
                                CELL,vertical_radius=.3 if platform == "dog" else None)
        if platform=='drone':
            # Route centres remain in [1.35,9.45] m; ordinary exact goals
            # use [1.2,9.3] m. No route may skim the ground or map ceiling.
            self.blocked[:,:,:4]=True;self.blocked[:,:,32:]=True
        self.ground_states=None
        self.ground_projection_id=None
        ground_file=Path(str(filename).replace('_known_0p3.bin','_ground_0p3.bin'))
        if ground_file!=Path(filename) and ground_file.exists():
            ground=np.frombuffer(ground_file.read_bytes(),dtype=np.uint8)
            if len(ground)!=SHAPE[0]*SHAPE[1] or not np.isin(ground,[0,1,2]).all():
                raise ValueError('invalid ground-band grid')
            self.ground_states=ground.reshape(SHAPE[:2])
            self.ground_projection_id=hashlib.sha256(ground.tobytes()).hexdigest()
            if platform=='dog':
                self.blocked[:]=True;self.physical[:]=True
                for mask,radius in ((self.blocked,.65),(self.physical,.35)):
                    mask[:,:,1]=inflate((self.ground_states!=1)[:,:,None],radius,CELL,vertical_radius=0)[:,:,0]

    def index(self, point):
        return tuple(np.floor((np.asarray(point)-ORIGIN)/CELL).astype(int))

    def valid_index(self, i):
        return all(0 <= j < n for j,n in zip(i,SHAPE))

    def safe(self, point, physical=False):
        i = self.index(point)
        return self.valid_index(i) and not (self.physical if physical else self.blocked)[i]

    def check(self, point, label):
        if len(point)!=3 or not all(math.isfinite(v) for v in point):
            raise ValueError(f"{label}: invalid coordinate")
        i = self.index(point)
        if not self.valid_index(i):
            raise ValueError(f"{label}: outside map")
        state=self.ground_states[i[:2]] if getattr(self,'ground_states',None) is not None and i[2]==1 else self.states[i]
        if state!=1:
            raise ValueError(f"{label}: {'unknown' if state==0 else 'occupied'}")
        if self.blocked[i]:
            raise ValueError(f"{label}: insufficient clearance")
        return i

    def check_segments(self, points, physical=False):
        for a,b in zip(points[:-1],points[1:]):
            if not self.line_free(a,b,self.physical if physical else self.blocked):
                raise ValueError("route or exact endpoint leaves observed-free clearance")

    def check_takeoff(self, xy, top=2.7):
        if self.ground_states is None:
            self.check_segments([[*xy,.8],[*xy,top]],physical=True);return
        # The sphere clearance model intersects the floor during launch.
        # Verify the ENTIRE low launch column separately, excluding only the
        # floor band below 15 cm, with 30 cm horizontal tracking reserve.
        lo=self.index([xy[0]-.95,xy[1]-.95,0]);hi=self.index([xy[0]+.95,xy[1]+.95,top+.65])
        if not self.valid_index(lo) or not self.valid_index(hi):raise ValueError('takeoff column outside map')
        area=(slice(lo[0],hi[0]+1),slice(lo[1],hi[1]+1))
        if np.any(self.ground_states[area]!=1) or np.any(self.states[area+(slice(1,hi[2]+1),)]!=1):
            raise ValueError('takeoff column contains occupied or unknown volume')

    def line_free(self,a,b,blocked):
        """Continuous-endpoint supercover; includes every touched edge cell."""
        a=(np.array(a)-ORIGIN)/CELL;b=(np.array(b)-ORIGIN)/CELL
        cur=np.floor(a).astype(int).tolist();end=np.floor(b).astype(int).tolist()
        delta=b-a;step=[int(v>0)-int(v<0) for v in delta]
        tmax=[((cur[i]+(1 if step[i]>0 else 0)-a[i])/delta[i]) if step[i] else math.inf for i in range(3)]
        tdelta=[abs(1/v) if v else math.inf for v in delta]
        def free(index):return self.valid_index(index) and not blocked[tuple(index)]
        if not free(cur) or not free(end):return False
        while cur!=end:
            boundary=min(tmax)
            if boundary>1+1e-10:return False
            axes=[i for i,v in enumerate(tmax) if abs(v-boundary)<=1e-10 and cur[i]!=end[i]]
            if not axes:return False
            for count in range(1,len(axes)+1):
                for subset in itertools.combinations(axes,count):
                    neighbour=cur.copy()
                    for i in subset:neighbour[i]+=step[i]
                    if not free(neighbour):return False
            for i in axes:cur[i]+=step[i];tmax[i]+=tdelta[i]
        return True

    def plan(self, start, target, cancel=None, timeout=25, obstacles=()):
        deadline = time.monotonic()+timeout
        stop = lambda: time.monotonic()>deadline or (cancel is not None and cancel.is_set())
        si,gi = self.check(start,"start"),self.check(target,"goal")
        blocked = self.blocked
        if obstacles:
            blocked = blocked.copy()
            # The other vehicle is stationary while the mission lease is
            # held. Rasterize its body plus both-vehicle tracking clearance.
            for centre,radius in obstacles:
                lo=np.maximum(0,np.floor((np.array(centre)-radius-ORIGIN)/CELL).astype(int)-1)
                hi=np.minimum(SHAPE,np.ceil((np.array(centre)+radius-ORIGIN)/CELL).astype(int)+1)
                if np.any(hi<=lo): continue
                coords=np.ogrid[tuple(slice(int(a),int(b)) for a,b in zip(lo,hi))]
                mask=sum((ORIGIN[i]+(coords[i]+.5)*CELL-centre[i])**2 for i in range(3)) <= (radius+CELL*math.sqrt(3)/2)**2
                region=tuple(slice(int(a),int(b)) for a,b in zip(lo,hi))
                blocked[region] |= mask
            if blocked[si] or blocked[gi]: raise ValueError("start or goal too close to the other robot")
        if stop():raise ValueError('planning cancelled or timed out')
        if self.line_free(start,target,blocked):
            if stop():raise ValueError('planning cancelled or timed out')
            return [list(start),list(target)]
        if self.platform=='drone':
            # Checked continuous segments and horizontal-layer searches avoid
            # spending the 3D search budget on large, mostly open volumes.
            # Every ascent, descent and corner connection uses the SAME mask.
            levels=sorted(range(4,32),key=lambda k:abs((k+.5)*CELL-start[2])+abs((k+.5)*CELL-target[2]))
            columns=[];candidates=[]
            for k in levels:
                if stop():raise ValueError('planning cancelled or timed out')
                z=(k+.5)*CELL;a=[start[0],start[1],z];b=[target[0],target[1],z]
                if not self.line_free(start,a,blocked) or not self.line_free(b,target,blocked):continue
                columns.append((k,a,b))
                if self.line_free(a,b,blocked):candidates.append([list(start),a,b,list(target)])
            if candidates:
                route=min(candidates,key=lambda p:sum(math.dist(a,b) for a,b in zip(p[:-1],p[1:])))
                return self.shorten_checked(route,blocked,stop)
            for k,a,b in columns[:4]:
                route,reason=plan_3d.astar(blocked[:,:,k:k+1],(si[0],si[1],0),(gi[0],gi[1],0),CELL,
                                          ground=True,should_stop=stop)
                if route is None:continue
                route=simplify_grid_path(route,blocked[:,:,k:k+1],should_stop=stop)
                points=[list(start),a]+[[ORIGIN[0]+(i+.5)*CELL,ORIGIN[1]+(j+.5)*CELL,(k+.5)*CELL] for i,j,_ in route]+[b,list(target)]
                if all(self.line_free(u,v,blocked) for u,v in zip(points[:-1],points[1:])):
                    return self.shorten_checked(points,blocked,stop)
        indices,reason = plan_3d.astar(blocked,si,gi,CELL,
                                     ground=self.platform=="dog",should_stop=stop)
        if indices is None:
            raise ValueError(str(reason))
        if stop():
            raise ValueError("planning cancelled or timed out")
        if self.platform in ("dog","drone"):
            indices = simplify_grid_path(indices,blocked,should_stop=stop)
        else:
            kept = [indices[0]]
            for i in range(1,len(indices)-1):
                a,b,c = np.array(indices[i-1]),np.array(indices[i]),np.array(indices[i+1])
                if np.any(b-a != c-b): kept.append(indices[i])
            if len(indices)>1: kept.append(indices[-1])
            indices = kept
        path = [(ORIGIN+(np.asarray(i)+.5)*CELL).tolist() for i in indices]
        # Exact endpoints and subcell connections are checked, not snapped.
        path = [list(start)]+path+[list(target)]
        # Cell centres near the actual start/goal are not real corners. A
        # 10 cm quantization kink otherwise creates needless turn/reverse
        # pulses. Remove it only when the exact connecting line is checked.
        if len(path)>2 and math.dist(path[0],path[1])<.45 and self.line_free(path[0],path[2],blocked):
            del path[1]
        if len(path)>2 and math.dist(path[-1],path[-2])<.45 and self.line_free(path[-3],path[-1],blocked):
            del path[-2]
        self.check_segments(path)
        for a,b in zip(path[:-1],path[1:]):
            if not self.line_free(a,b,blocked):
                raise ValueError("exact route connection intersects the other robot clearance")
        if stop(): raise ValueError("planning cancelled or timed out")
        return path

    def shorten_checked(self,points,blocked,stop=lambda:False):
        result=[list(points[0])];i=0
        while i<len(points)-1:
            if stop():raise ValueError('planning cancelled or timed out')
            j=len(points)-1
            while j>i+1 and not self.line_free(points[i],points[j],blocked):j-=1
            if math.dist(result[-1],points[j])>1e-9:result.append(list(points[j]))
            i=j
        return result

    def view(self, z):
        k = int(self.index([0,0,z])[2])
        if not 0<=k<SHAPE[2]: raise ValueError("height outside map")
        raw=self.states[:,:,k]
        clearance_available=self.platform!='dog' or k==1
        same_occ=inflate((raw==2)[:,:,None],.35 if self.platform=='dog' else .65,CELL,vertical_radius=0)[:,:,0]
        same_unknown=inflate((raw==0)[:,:,None],.35 if self.platform=='dog' else .65,CELL,vertical_radius=0)[:,:,0]
        # Exclusive display reasons on raw free cells: body, unknown in this
        # slice, other-height/boundary body clearance, tracking reserve.
        reason=np.zeros(raw.shape,dtype=np.uint8)
        reason[(raw==1)&self.blocked[:,:,k]]=4
        reason[(raw==1)&self.physical[:,:,k]]=3
        reason[(raw==1)&self.physical[:,:,k]&same_unknown]=2
        reason[(raw==1)&self.physical[:,:,k]&same_occ]=1
        if not clearance_available:reason[:]=0
        return {"map_id":self.map_id,"frame":"map","cell":CELL,
                "origin":ORIGIN.tolist(),"width":SHAPE[0],"height":SHAPE[1],
                "platform":self.platform,"slice_index":k,"slice_bounds":[k*CELL,(k+1)*CELL],
                "ground_projection_id":self.ground_projection_id,
                "clearance_available":clearance_available,
                "ground_band_m":[.15,.90] if getattr(self,'ground_states',None) is not None else None,
                "slice_center":(k+.5)*CELL,"planning_radius":self.radius,
                "physical_radius":.35 if self.platform=='dog' else .65,
                "reason_labels":{"1":"机体净空：本层障碍","2":"机体净空：本层未知",
                                 "3":"机体净空：其他高度或地图边界","4":"跟踪与规划余量"},
                "reasons":reason.T.ravel().tolist(),
                "z":z,"states":self.states[:,:,k].T.ravel().tolist(),
                "blocked":(self.blocked[:,:,k] if clearance_available else np.zeros(raw.shape,bool)).T.ravel().astype(int).tolist()}

class DogMotion:
    """Verified stop/turn/drive pulse timing, interruptible at every tick."""
    def __init__(self,path,target,now,speed=.30,yaw_speed=.10):
        self.speed,self.yaw_speed=speed,yaw_speed
        self.points = execution_points([p[:2] for p in path])
        self.points[-1] = list(target[:2])
        self.target = list(target[:2])
        self.index = 1 if len(self.points)>1 else 0
        self.phase,self.until = "choose",now
        self.yaw_command,self.vx_command = 0.,0.
        self.settle_at = None
        self.progress_at,self.progress_pose = now,None
        length = sum(math.dist(a,b) for a,b in zip(self.points[:-1],self.points[1:]))
        self.deadline = now+max(120.,length/.12+len(self.points)*20.)

    def tick(self,pose,now):
        if now>self.deadline: raise ValueError("navigation timeout")
        x,y,yaw = pose
        if self.progress_pose is None or math.dist([x,y],self.progress_pose[:2])>.05 or abs(wrap(yaw-self.progress_pose[2]))>.05:
            self.progress_pose,self.progress_at = list(pose),now
        if now-self.progress_at>30: raise ValueError("robot stopped making progress")
        if self.settle_at is not None:
            if now-self.settle_at<2: return 0.,0.,False
            if math.dist([x,y],self.target)>=.35: raise ValueError("settled goal error exceeds 0.35 m")
            return 0.,0.,True
        if self.phase!="choose" and now<self.until:
            return self.vx_command,self.yaw_command,False
        if self.phase=="before_turn":
            self.phase,self.until = "turn",now+self.turn_time
            self.vx_command,self.yaw_command = 0.,self.turn_sign*self.yaw_speed
            return 0.,self.yaw_command,False
        if self.phase=="turn":
            self.phase,self.until = "after_turn",now+.8
            self.vx_command,self.yaw_command = 0.,0.
            return 0.,0.,False
        self.phase = "choose"
        point = self.points[self.index]
        final = self.index==len(self.points)-1
        corner = False
        if not final and self.index>0:
            a,b = self.points[self.index-1],self.points[self.index+1]
            corner = abs(wrap(math.atan2(point[1]-a[1],point[0]-a[0])-math.atan2(b[1]-point[1],b[0]-point[0])))>.01
        distance = math.dist([x,y],point)
        if distance<(.25 if final else .12 if corner else .45):
            if final: self.settle_at = now
            else: self.index+=1
            self.vx_command,self.yaw_command = 0.,0.
            return 0.,0.,False
        error = wrap(math.atan2(point[1]-y,point[0]-x)-yaw)
        if abs(error)>.08:
            self.turn_time = min(2.,max(.2,(abs(error)-.08)/self.yaw_speed))
            self.turn_sign = -1. if error>0 else 1.
            self.phase,self.until = "before_turn",now+1.
            self.vx_command,self.yaw_command = 0.,0.
        else:
            self.phase,self.until = "drive",now+.4
            self.vx_command = min(.15,self.speed) if (final or corner) and distance<.8 else self.speed
            self.yaw_command = 0.
        return self.vx_command,self.yaw_command,False

    def deviation(self,pose):
        return min((segment_distance(pose[:2],a,b) for a,b in zip(self.points[:-1],self.points[1:])),default=0.)
