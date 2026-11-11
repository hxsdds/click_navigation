import math
import json
import hashlib,tempfile
from pathlib import Path
import sys
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'src/uav_dog_nav2/scripts')]
from click_nav_core import FlightTrajectory,target_coordinates,GridMap,SHAPE,resolve_navigation_grid


class StaticNavigationTests(unittest.TestCase):
    def test_manifest_rejects_changed_world_or_map(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);(root/'config').mkdir();meta={'cell_m':.3,'shape':list(SHAPE),'origin':[-52.,-40.,0.]}
            for name in ('known_grid','ground_grid','world'):
                (root/name).write_bytes(b'validated');meta[name]={'file':name,'sha256':hashlib.sha256(b'validated').hexdigest()}
            (root/'config/static_navigation_map.json').write_text(json.dumps(meta))
            self.assertEqual(resolve_navigation_grid(root)[0],root/'known_grid')
            (root/'world').write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError,'world changed'):resolve_navigation_grid(root)
    def test_slice_metadata_is_json_serializable(self):
        m=GridMap.__new__(GridMap);m.platform='drone';m.radius=.95;m.map_id='test';m.ground_projection_id=None
        m.ground_states=None;m.states=np.ones(SHAPE,np.uint8);m.blocked=np.zeros(SHAPE,bool);m.physical=m.blocked
        view=json.loads(json.dumps(m.view(6.)))
        self.assertEqual(view['slice_index'],20);self.assertAlmostEqual(view['slice_center'],6.15)
    def test_three_dimensional_planner_must_rise_and_descend(self):
        for low_wall in (True,False):
            m=GridMap.__new__(GridMap);m.platform='drone';m.ground_states=None
            m.states=np.ones(SHAPE,np.uint8);m.blocked=np.zeros(SHAPE,bool)
            m.blocked[:,:,:4]=True;m.blocked[:,:,32:]=True
            if low_wall:m.blocked[150,:,:18]=True
            else:m.blocked[150,:,12:]=True
            m.physical=m.blocked.copy()
            z=3. if low_wall else 6.
            path=m.plan([-10.,-4.,z],[0.,-4.,z])
            heights=[p[2] for p in path]
            self.assertGreater(max(heights),5.4) if low_wall else self.assertLess(min(heights),3.6)
            self.assertEqual(path[-1],[0.,-4.,z]);m.check_segments(path)
    def test_xyz_goal_contract(self):
        self.assertEqual(target_coordinates({'x':2,'y':3,'z':4},'drone'),[2.,3.,4.])
        self.assertEqual(target_coordinates({'x':2,'y':3},'drone'),[2.,3.,6.])
        self.assertEqual(target_coordinates({'x':2,'y':3,'z':4},'dog'),[2.,3.,.5])
        for z in (0,1.,9.4,math.inf,math.nan):
            with self.assertRaises(ValueError):target_coordinates({'x':0,'y':0,'z':z},'drone')

    def test_trajectory_velocity_acceleration_and_exact_corners(self):
        traj=FlightTrajectory([0.,0.,1.5]);samples=[np.array(traj.point)]
        for goal in ([3.,0.,4.],[3.,-3.,2.]):
            for _ in range(1000):
                p=traj.tick(goal,traj.point,.05);samples.append(np.array(p))
                if traj.done:break
            self.assertTrue(traj.done);np.testing.assert_allclose(traj.point,goal,atol=1e-9)
        velocity=np.diff(samples,axis=0)/.05
        self.assertLessEqual(np.linalg.norm(velocity,axis=1).max(),.6000001)
        self.assertLessEqual(np.abs(velocity[:,2]).max(),.4000001)
        self.assertLessEqual(np.linalg.norm(np.diff(velocity,axis=0)/.05,axis=1).max(),.350001)

    def test_trajectory_lead_pause_sim_pause_and_corner_guard(self):
        t=FlightTrajectory([0.,0.,2.])
        for _ in range(200):t.tick([10.,0.,2.],[0.,0.,2.],.05)
        self.assertLessEqual(math.dist(t.point,[0,0,2]),.6)
        previous=t.point.copy();t.tick([10.,0.,2.],[0,0,2],-1)
        self.assertEqual(t.point,previous)
        with self.assertRaisesRegex(ValueError,'corner'):t.tick([10,1,2],t.point,.05)


if __name__=='__main__':unittest.main()
