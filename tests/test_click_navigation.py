"""Map endpoint safety, cancellation and interruptible gait regression."""
import math
from pathlib import Path
import sys
import threading
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/"scripts"),str(ROOT/"src/uav_dog_nav2/scripts")]
import click_nav_core as core
import plan_3d
from plan_goal_3d import grid_line_free, simplify_grid_path


class ClickNavigationTests(unittest.TestCase):
    def test_drone_setpoint_velocity_and_lead_are_bounded(self):
        p=core.advance_setpoint([0,0,1],[10,0,1],[0,0,1],.1)
        self.assertAlmostEqual(p[0],.06)
        self.assertEqual(core.advance_setpoint([.59,0,1],[10,0,1],[0,0,1],.1),[.59,0,1])
        self.assertEqual(core.advance_setpoint([0,0,1],[10,0,1],[0,0,1],-1),[0.,0.,1.])
    def fake_map(self):
        m = core.GridMap.__new__(core.GridMap)
        m.platform = "dog"
        m.states = np.ones(core.SHAPE,dtype=np.uint8)
        m.blocked = np.zeros(core.SHAPE,dtype=bool)
        m.physical = m.blocked.copy()
        return m

    def test_astar_cancellation_is_honoured(self):
        path, reason = plan_3d.astar(np.zeros((10,10,1),bool),(0,0,0),(9,9,0),.3,
                                    should_stop=lambda:True)
        self.assertIsNone(path)
        self.assertIn("cancel",reason)

    def test_shortcut_check_and_simplification_can_be_cancelled(self):
        grid=np.zeros((20,20,2),bool)
        with self.assertRaisesRegex(TimeoutError,"cancel"):
            grid_line_free((0,0,0),(19,19,0),grid,lambda:True)
        with self.assertRaisesRegex(TimeoutError,"cancel"):
            simplify_grid_path([(0,0,0),(1,0,0),(2,0,0)],grid,lambda:True)

    def test_optimized_traversal_agrees_with_reference(self):
        import itertools
        def reference(first,last,blocked):
            cur=np.array(first);target=np.array(last);direction=target-cur
            step=np.sign(direction).astype(int);delta=np.full(3,np.inf)
            moving=direction!=0;delta[moving]=1/np.abs(direction[moving]);crossing=delta/2
            if blocked[tuple(cur)]:return False
            while not np.array_equal(cur,target):
                axes=np.flatnonzero(np.isclose(crossing,crossing.min(),rtol=0,atol=1e-10))
                for size in range(1,len(axes)+1):
                    for subset in itertools.combinations(axes,size):
                        n=cur.copy();n[list(subset)]+=step[list(subset)]
                        if blocked[tuple(n)]:return False
                cur[axes]+=step[axes];crossing[axes]+=delta[axes]
            return True
        rng=np.random.default_rng(20261108)
        for _ in range(200):
            grid=rng.random((12,12,6))<.08
            first=tuple(rng.integers([0,0,0],[12,12,6]))
            last=tuple(rng.integers([0,0,0],[12,12,6]))
            self.assertEqual(grid_line_free(first,last,grid),reference(first,last,grid))

    def test_requested_endpoint_is_preserved(self):
        m=self.fake_map()
        start,target=[-4.03,-6.02,.5],[-1.21,-5.73,.5]
        path=m.plan(start,target)
        self.assertEqual(path[0],start)
        self.assertEqual(path[-1],target)

    def test_nearby_grid_centres_do_not_create_small_turns(self):
        m=self.fake_map()
        start,target=[-3.96,-6.02,.5],[-1,-6,.5]
        self.assertEqual(m.plan(start,target),[start,target])

    def test_continuous_endpoint_diagonal_cannot_cut_corner(self):
        m=self.fake_map()
        a=(core.ORIGIN+np.array([100.1,100.1,1.5])*core.CELL).tolist()
        b=(core.ORIGIN+np.array([102.9,102.9,1.5])*core.CELL).tolist()
        m.blocked[101,100,1]=True
        self.assertFalse(m.line_free(a,b,m.blocked))

    def test_other_robot_is_treated_as_an_obstacle(self):
        m=self.fake_map()
        path=m.plan([-4,-6,.5],[4,-6,.5],obstacles=[([0,-6,.5],1.3)])
        self.assertTrue(all(core.segment_distance([0,-6,.5],a,b)>1.3 for a,b in zip(path[:-1],path[1:])))
        with self.assertRaisesRegex(ValueError,"other robot"):
            m.plan([-4,-6,.5],[0,-6,.5],obstacles=[([0,-6,.5],1.3)])

    def test_exact_endpoint_connection_cannot_cut_blocked_cell(self):
        m=self.fake_map()
        m.blocked[m.index([-3.5,-6,.5])]=True
        with self.assertRaisesRegex(ValueError,"clearance"):
            m.check_segments([[-4,-6,.5],[-3,-6,.5]])

    def test_map_unknown_occupied_boundary_and_clearance(self):
        m=self.fake_map()
        for value,reason in [(0,"unknown"),(2,"occupied")]:
            m.states[m.index([-4,-6,.5])]=value
            with self.assertRaisesRegex(ValueError,reason):m.check([-4,-6,.5],"goal")
        m.states[m.index([-4,-6,.5])]=1
        m.blocked[m.index([-4,-6,.5])]=True
        with self.assertRaisesRegex(ValueError,"clearance"):m.check([-4,-6,.5],"goal")
        with self.assertRaisesRegex(ValueError,"outside"):m.check([100,0,.5],"goal")
        with self.assertRaisesRegex(ValueError,"invalid"):m.check([math.nan,0,.5],"goal")

    def test_cancel_event_terminates_planning(self):
        m=self.fake_map();event=threading.Event();event.set()
        with self.assertRaisesRegex(ValueError,"cancel"):
            m.plan([-4,-6,.5],[1,-6,.5],event)

    def test_tracking_points_preserve_corner_and_spacing(self):
        pts=core.execution_points([[0,0],[10,0],[10,3]])
        self.assertIn([10.,0.],pts)
        self.assertTrue(all(math.dist(a,b)<=1.5+1e-8 for a,b in zip(pts[:-1],pts[1:])))

    def test_dog_must_settle_and_error_is_checked_again(self):
        motion=core.DogMotion([[0,0,.5],[1,0,.5]],[1,0,.5],0)
        self.assertFalse(motion.tick([.8,0,0],0)[2])
        self.assertFalse(motion.tick([.8,0,0],1.9)[2])
        self.assertTrue(motion.tick([.8,0,0],2.01)[2])
        with self.assertRaisesRegex(ValueError,"settled"):
            motion.tick([.4,0,0],2.1)

    def test_turn_and_drive_are_not_simultaneous(self):
        m=core.DogMotion([[0,0,.5],[0,2,.5]],[0,2,.5],0)
        self.assertEqual(m.tick([0,0,0],0)[:2],(0.,0.))
        vx,yaw,done=m.tick([0,0,0],1.01)
        self.assertEqual(vx,0.)
        self.assertEqual(yaw,-.1)
        self.assertFalse(done)
        # A tick can always be suppressed externally; no blocking sleep/pulse.
        self.assertEqual(m.tick([0,0,0],1.06)[:2],(0.,-.1))

    def test_stationary_robot_times_out(self):
        m=core.DogMotion([[0,0,.5],[10,0,.5]],[10,0,.5],0)
        m.tick([0,0,0],0)
        with self.assertRaisesRegex(ValueError,"progress"):m.tick([0,0,0],31)

    def test_heading_progress_does_not_count_as_stall(self):
        m=core.DogMotion([[0,0,.5],[-10,0,.5]],[-10,0,.5],0)
        for i in range(40):m.tick([0,0,i*.08],i)


if __name__=="__main__":unittest.main()
