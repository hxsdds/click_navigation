"""Safety checks for the offline occupancy planner."""

import importlib.util
import pathlib
import unittest

import numpy as np


MODULE_PATH = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "plan_3d.py"
SPEC = importlib.util.spec_from_file_location("plan_3d", MODULE_PATH)
planner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(planner)


class PlannerTests(unittest.TestCase):
    def test_ground_robot_cannot_fly_over_wall(self):
        blocked = np.zeros((5, 5, 4), dtype=bool)
        blocked[2, :, 1] = True
        start, goal = (0, 2, 1), (4, 2, 1)
        self.assertIsNone(planner.astar(blocked, start, goal, 0.3, ground=True)[0])
        self.assertIsNotNone(planner.astar(blocked, start, goal, 0.3, ground=False)[0])

    def test_blocked_goal_is_not_silently_moved(self):
        blocked = np.zeros((4, 4, 3), dtype=bool)
        blocked[3, 3, 1] = True
        path, reason = planner.astar(blocked, (0, 0, 1), (3, 3, 1), 0.3)
        self.assertIsNone(path)
        self.assertEqual(reason, "goal blocked")

    def test_diagonal_cannot_cut_blocked_corner(self):
        blocked = np.ones((2, 2, 1), dtype=bool)
        blocked[0, 0, 0] = False
        blocked[1, 1, 0] = False
        self.assertIsNone(planner.astar(blocked, (0, 0, 0), (1, 1, 0), 0.3,
                                        ground=True)[0])

    def test_space_diagonal_cannot_cut_blocked_face(self):
        blocked = np.ones((2, 2, 2), dtype=bool)
        for cell in ((0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1),
                     (1, 1, 1)):
            blocked[cell] = False
        self.assertIsNone(planner.astar(blocked, (0, 0, 0), (1, 1, 1),
                                        0.3)[0])

    def test_outside_coordinate_is_outside_grid(self):
        x0, _, y0, _, z0, _ = planner.BOUNDS
        self.assertEqual(planner.to_index(x0 - 0.01, y0, z0, 0.3)[0], -1)


if __name__ == "__main__":
    unittest.main()
