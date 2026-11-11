#!/usr/bin/env python3
"""Plan a collision-checked route to one specified goal on the saved 3D map.

This is an offline map test, not a flight or walking controller. The body
radius is inflated before A*, and unknown OctoMap voxels are blocked. A
blocked goal is reported as blocked rather than moved silently.
"""

import argparse
import json
import math
import itertools
import sys
from pathlib import Path

import numpy as np

import plan_3d


def grid_line_free(first, last, blocked, should_stop=None):
    """Supercover traversal between cell centres, including edge-touch cells."""
    current = [int(v) for v in first]
    target = [int(v) for v in last]
    direction = [b-a for a,b in zip(current,target)]
    step = [(v>0)-(v<0) for v in direction]
    delta = [1.0/abs(v) if v else math.inf for v in direction]
    crossing = [v/2 for v in delta]
    if blocked[tuple(current)]:
        return False
    while current != target:
        if should_stop is not None and should_stop():
            raise TimeoutError("planning cancelled or timed out")
        boundary = min(crossing)
        axes = [i for i,v in enumerate(crossing) if abs(v-boundary)<=1e-10]
        # At an edge/corner the segment touches every adjacent cell. Accepting
        # only the diagonal successor would reintroduce obstacle corner cuts.
        for size in range(1, len(axes)+1):
            for subset in itertools.combinations(axes, size):
                neighbour = current.copy()
                for axis in subset:
                    neighbour[axis] += step[axis]
                if blocked[tuple(neighbour)]:
                    return False
        for axis in axes:
            current[axis] += step[axis]
            crossing[axis] += delta[axis]
    return True


def simplify_grid_path(indices, blocked, should_stop=None):
    """Remove staircase turns only when the complete shortcut has clearance."""
    if len(indices) < 3:
        return indices
    result = [indices[0]]
    anchor = 0
    while anchor < len(indices)-1:
        if should_stop is not None and should_stop():
            raise TimeoutError("planning cancelled or timed out")
        candidate = len(indices)-1
        while candidate > anchor+1 and not grid_line_free(indices[anchor], indices[candidate], blocked, should_stop):
            candidate -= 1
        result.append(indices[candidate])
        anchor = candidate
    return result


def inflate(blocked, radius, cell, vertical_radius=None):
    steps = math.ceil(radius / cell)
    grown = blocked.copy()
    for dx in range(-steps, steps + 1):
        for dy in range(-steps, steps + 1):
            for dz in range(-steps, steps + 1):
                if vertical_radius is not None and abs(dz) * cell > vertical_radius + 1e-9:
                    continue
                if (dx * dx + dy * dy + dz * dz) * cell * cell > radius * radius:
                    continue
                source = tuple(slice(max(0, -d), min(n, n - d))
                               for d, n in zip((dx, dy, dz), blocked.shape))
                dest = tuple(slice(max(0, d), min(n, n + d))
                             for d, n in zip((dx, dy, dz), blocked.shape))
                grown[dest] |= blocked[source]
    # Outside the exported map is unknown too. Array slicing alone discards
    # those neighbours and would falsely accept a body straddling its edge.
    for axis, extent in enumerate((radius, radius, radius if vertical_radius is None
                                   else min(radius, vertical_radius))):
        edge = math.floor(extent/cell+1e-9)
        if edge:
            low = [slice(None)]*3
            high = [slice(None)]*3
            low[axis], high[axis] = slice(0, edge), slice(-edge, None)
            grown[tuple(low)] = True
            grown[tuple(high)] = True
    return grown


def load_known_grid(path, cell):
    x0, x1, y0, y1, z0, z1 = plan_3d.BOUNDS
    shape = tuple(int((hi - lo) / cell) for lo, hi in
                  ((x0, x1), (y0, y1), (z0, z1)))
    data = np.fromfile(path, dtype=np.uint8)
    if data.size != math.prod(shape):
        raise ValueError(f"OctoMap grid has {data.size} cells; expected {math.prod(shape)}")
    return data.reshape(shape)


def plan(platform, goal_xy, states, cell=0.3, start_xy=(-4.0, -6.0),
         drone_start_altitude=1.0, drone_goal_altitude=6.0, drone_radius=0.65,
         dog_radius=0.35):
    radius, start_z, goal_z, ground = (
        (dog_radius, 0.5, 0.5, True) if platform == "dog"
        else (drone_radius, drone_start_altitude, drone_goal_altitude, False))
    # 0=unknown, 1=observed free, 2=occupied. Unknown is never flyable.
    blocked = inflate(states != 1, radius, cell,
                      vertical_radius=0.3 if ground else None)
    start = plan_3d.to_index(*start_xy, start_z, cell)
    goal = plan_3d.to_index(*goal_xy, goal_z, cell)
    for label, index in (("start", start), ("goal", goal)):
        if not all(0 <= index[a] < states.shape[a] for a in range(3)):
            return {"platform": platform, "ok": False, "reason": f"{label} outside map",
                    "goal": [*goal_xy, goal_z]}
        if states[index] != 1:
            condition = "unknown" if states[index] == 0 else "occupied"
            return {"platform": platform, "ok": False,
                    "reason": f"{label} {condition} in OctoMap",
                    "goal": [*goal_xy, goal_z]}
        if blocked[index]:
            return {"platform": platform, "ok": False,
                    "reason": f"{label} clearance intersects unknown or occupied space",
                    "goal": [*goal_xy, goal_z]}
    indices, cost = plan_3d.astar(blocked, start, goal, cell, ground=ground)
    if indices is None:
        return {"platform": platform, "ok": False, "reason": cost,
                "goal": [*goal_xy, goal_z]}
    if ground:
        indices = simplify_grid_path(indices, blocked)
        cost = sum(math.dist(a, b) for a, b in zip(indices[:-1], indices[1:]))
    x0, _, y0, _, z0, _ = plan_3d.BOUNDS
    path = [[round(x0 + (i + 0.5) * cell, 3),
             round(y0 + (j + 0.5) * cell, 3),
             round(z0 + (k + 0.5) * cell, 3)] for i, j, k in indices]
    return {"platform": platform, "ok": True, "goal": [*goal_xy, goal_z],
            "start": [*start_xy, start_z],
            "length_m": round(cost * cell, 3), "radius_m": radius,
            "path": path}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--goal-x", type=float, required=True)
    parser.add_argument("--goal-y", type=float, required=True)
    parser.add_argument("--start-x", type=float, default=-4.0)
    parser.add_argument("--start-y", type=float, default=-6.0)
    parser.add_argument("--drone-start-altitude", type=float, default=1.0)
    parser.add_argument("--drone-goal-altitude", type=float, default=6.0)
    parser.add_argument("--drone-radius", type=float, default=0.95,
                        help="planning radius includes 0.30 m tracking margin")
    parser.add_argument("--dog-radius", type=float, default=0.65,
                        help="planning radius includes 0.30 m tracking margin")
    parser.add_argument("--platform", choices=("dog", "drone", "both"), default="both")
    parser.add_argument("--known-grid", default="maps/chemical_plant_v4_static11_known_0p3.bin",
                        help="0=3D unknown, 1=observed free, 2=occupied; export with audit_octomap")
    parser.add_argument("--output", default="")
    args = parser.parse_args()
    states = load_known_grid(args.known_grid, 0.3)
    platforms = ("dog", "drone") if args.platform == "both" else (args.platform,)
    result = {item: plan(item, (args.goal_x, args.goal_y), states,
                         start_xy=(args.start_x, args.start_y),
                         drone_start_altitude=args.drone_start_altitude,
                         drone_goal_altitude=args.drone_goal_altitude,
                         drone_radius=args.drone_radius, dog_radius=args.dog_radius)
              for item in platforms}
    for item in platforms:
        route = result[item]
        if route["ok"]:
            print(f"{item}: PASS {route['length_m']:.1f} m, "
                  f"{len(route['path'])} cells, radius {route['radius_m']:.2f} m")
        else:
            print(f"{item}: FAIL {route['reason']}")
    if args.output:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print(f"routes written: {output}")
    return 0 if all(route["ok"] for route in result.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
