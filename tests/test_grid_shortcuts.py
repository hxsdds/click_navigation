"""A simplified route must preserve the inflated grid's obstacle clearance."""
import sys
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"scripts"))
from plan_goal_3d import grid_line_free, simplify_grid_path, inflate


class ShortcutTests(unittest.TestCase):
    def test_body_cannot_extend_into_unknown_outside_grid(self):
        grid = np.zeros((10, 10, 10), dtype=bool)
        grown = inflate(grid, .65, .3)
        self.assertTrue(grown[1, 5, 5])
        self.assertTrue(grown[5, 5, 8])
        self.assertFalse(grown[3, 5, 5])
        flat = inflate(grid, .65, .3, vertical_radius=.3)
        self.assertTrue(flat[5, 5, 0])
        self.assertFalse(flat[5, 5, 1])

    def test_open_staircase_becomes_one_segment(self):
        grid = np.zeros((5, 5, 1), dtype=bool)
        path = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (2, 1, 0), (2, 2, 0)]
        self.assertEqual(simplify_grid_path(path, grid), [path[0], path[-1]])

    def test_wall_detour_remains_clear(self):
        grid = np.zeros((5, 5, 1), dtype=bool)
        grid[2, :3, 0] = True
        path = [(0, 0, 0), (0, 1, 0), (0, 2, 0), (0, 3, 0),
                (1, 3, 0), (2, 3, 0), (3, 3, 0), (4, 3, 0),
                (4, 2, 0), (4, 1, 0), (4, 0, 0)]
        result = simplify_grid_path(path, grid)
        self.assertGreater(len(result), 2)
        self.assertTrue(all(grid_line_free(a, b, grid) for a, b in zip(result[:-1], result[1:])))

    def test_diagonal_touch_is_blocked_in_both_directions(self):
        grid = np.zeros((3, 3, 3), dtype=bool)
        grid[1, 0, 0] = True
        for a, b in [((0, 0, 0), (2, 2, 2)), ((2, 2, 2), (0, 0, 0))]:
            self.assertFalse(grid_line_free(a, b, grid))

    def test_single_cell_and_blocked_endpoint(self):
        grid = np.zeros((3, 3, 1), dtype=bool)
        self.assertTrue(grid_line_free((0, 0, 0), (0, 0, 0), grid))
        grid[2, 1, 0] = True
        self.assertFalse(grid_line_free((0, 0, 0), (2, 1, 0), grid))
