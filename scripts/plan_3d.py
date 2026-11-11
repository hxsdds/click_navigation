#!/usr/bin/env python3
"""3D plannability test on the saved occupancy map.

Loads the occupied-voxel cloud, builds a 3D grid over the plant and runs A*
with a clearance radius for the two kinds of body:

  * dog   - 0.35 m radius, paths stay on the z=0.5 m slice
  * drone - 0.5 m radius, at 6 m

Two world models are tested:

  * optimistic  - everything that is not occupied is free (what a planner does
                  when unknown space is assumed traversable)
  * conservative - unknown columns from the 2D map are treated as blocked

A map that only works optimistically has holes; a map that works conservatively
is the one worth trusting for avoidance work.

    python3 scripts/plan_3d.py --pcd maps/x_occupied.pcd
"""

import argparse
import heapq
import itertools
import math
import re
import sys

import numpy as np

BOUNDS = (-52.0, 52.0, -40.0, 40.0, 0.0, 11.0)  # x0,x1, y0,y1, z0,z1
START = (-4.0, -6.0)
GOALS = [
    ("road east", 36.0, -6.0),
    ("road west", -36.0, -6.0),
    ("yard east", 20.0, -20.0),
    ("yard west", -20.0, -20.0),
    ("north gap", 0.0, 20.0),
    ("far north", 0.0, 33.0),
    ("east margin", 44.0, 0.0),
    ("west margin", -44.0, 0.0),
]


def load_pcd(path):
    raw = open(path, "rb").read()
    marker = b"DATA binary\n"
    return np.frombuffer(raw[raw.index(marker) + len(marker):],
                         dtype=np.float32).reshape(-1, 3)


def load_2d_unknown(yaml_path):
    """Unknown cells of the 2D map, as (x, y) metre coordinates."""
    text = open(yaml_path, encoding="utf-8").read()
    resolution = float(re.search(r"resolution:\s*([\d.eE+-]+)", text).group(1))
    origin = [float(v) for v in
              re.search(r"origin:\s*\[([^\]]+)\]", text).group(1).split(",")[:2]]
    image_name = re.search(r"image:\s*(\S+)", text).group(1)
    raw = open(yaml_path.rsplit("/", 1)[0] + "/" + image_name, "rb").read()
    fields, index = [], 0
    while len(fields) < 4:
        while raw[index:index + 1].isspace():
            index += 1
        start = index
        while not raw[index:index + 1].isspace():
            index += 1
        fields.append(raw[start:index])
    width, height = int(fields[1]), int(fields[2])
    index += 1
    image = np.frombuffer(raw[index:index + width * height],
                          dtype=np.uint8).reshape(height, width)
    unknown = (image > 10) & (image < 250)
    rows, columns = np.nonzero(unknown)
    xs = origin[0] + (columns + 0.5) * resolution
    ys = origin[1] + (height - rows - 0.5) * resolution
    return xs, ys, resolution


def build_grid(points, cell, radius, unknown_block=None):
    x0, x1, y0, y1, z0, z1 = BOUNDS
    nx = int((x1 - x0) / cell)
    ny = int((y1 - y0) / cell)
    nz = int((z1 - z0) / cell)
    occupied = np.zeros((nx, ny, nz), dtype=bool)

    index = np.floor((points - np.array([x0, y0, z0])) / cell).astype(int)
    valid = ((index[:, 0] >= 0) & (index[:, 0] < nx)
             & (index[:, 1] >= 0) & (index[:, 1] < ny)
             & (index[:, 2] >= 0) & (index[:, 2] < nz))
    index = index[valid]
    occupied[index[:, 0], index[:, 1], index[:, 2]] = True

    if unknown_block is not None:
        xs, ys, _ = unknown_block
        ux = np.floor((xs - x0) / cell).astype(int)
        uy = np.floor((ys - y0) / cell).astype(int)
        ok = (ux >= 0) & (ux < nx) & (uy >= 0) & (uy < ny)
        occupied[ux[ok], uy[ok], :] = True

    # inflate by the body radius (chebyshev, in cells)
    steps = max(1, int(round(radius / cell)))
    for _ in range(steps):
        grown = occupied.copy()
        grown[1:, :, :] |= occupied[:-1, :, :]
        grown[:-1, :, :] |= occupied[1:, :, :]
        grown[:, 1:, :] |= occupied[:, :-1, :]
        grown[:, :-1, :] |= occupied[:, 1:, :]
        grown[:, :, 1:] |= occupied[:, :, :-1]
        grown[:, :, :-1] |= occupied[:, :, 1:]
        occupied = grown
    return occupied, (nx, ny, nz)


def to_index(x, y, z, cell):
    x0, _, y0, _, z0, _ = BOUNDS
    return (math.floor((x - x0) / cell), math.floor((y - y0) / cell),
            math.floor((z - z0) / cell))


def astar(blocked, start, goal, cell, ground=False, should_stop=None):
    nx, ny, nz = blocked.shape
    for label, node in (("start", start), ("goal", goal)):
        if not all(0 <= node[i] < blocked.shape[i] for i in range(3)):
            return None, f"{label} outside the grid"
        if blocked[node]:
            return None, f"{label} blocked"

    neighbours = [(di, dj, dk)
                  for di in (-1, 0, 1) for dj in (-1, 0, 1) for dk in (-1, 0, 1)
                  if (di, dj, dk) != (0, 0, 0) and (not ground or dk == 0)]
    open_set = [(0.0, start)]
    came = {}
    cost = {start: 0.0}
    while open_set:
        if should_stop is not None and should_stop():
            return None, "planning cancelled or timed out"
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = [current]
            while path[-1] in came:
                path.append(came[path[-1]])
            return path[::-1], cost[current]
        for delta in neighbours:
            node = (current[0] + delta[0], current[1] + delta[1], current[2] + delta[2])
            if not (0 <= node[0] < nx and 0 <= node[1] < ny and 0 <= node[2] < nz):
                continue
            if blocked[node]:
                continue
            # Every intermediate axis-aligned cell of a diagonal move must be
            # free. Checking only the three single-axis cells misses 3D
            # corner cuts through occupied two-axis cells.
            moved_axes = [axis for axis in range(3) if delta[axis]]
            if any(blocked[tuple(current[a] + (delta[a] if a in selected else 0)
                                 for a in range(3))]
                   for count in range(1, len(moved_axes))
                   for selected in itertools.combinations(moved_axes, count)):
                continue
            step = math.sqrt(sum(d * d for d in delta))
            new_cost = cost[current] + step
            if new_cost < cost.get(node, float("inf")):
                cost[node] = new_cost
                came[node] = current
                heuristic = math.dist(node, goal)
                heapq.heappush(open_set, (new_cost + heuristic, node))
    return None, "no path"


def run(blocked, cell, altitude, label, radius, ground=False):
    print(f"\n{label} (radius {radius:.2f} m, altitude {altitude:.1f} m):")
    start = to_index(START[0], START[1], altitude, cell)
    ok = 0
    for name, x, y in GOALS:
        goal = to_index(x, y, altitude, cell)
        path, reason = astar(blocked, start, goal, cell, ground=ground)
        if path is None:
            print(f"   {name:12s} FAIL ({reason})")
            continue
        length = reason * cell          # A* returns the true cost in cells
        # path_clearance runs on the already-inflated grid: r free cells around
        # the path means radius + r*cell of true clearance.
        clearance = radius + path_clearance(blocked, path) * cell
        print(f"   {name:12s} {length:6.1f} m, clearance >= {clearance:4.2f} m")
        ok += 1
    return ok


def path_clearance(blocked, path):
    """Largest radius (in cells) around the path that stays free, sampled."""
    nx, ny, nz = blocked.shape
    best = 99
    for index in range(0, len(path), max(1, len(path) // 25)):
        i, j, k = path[index]
        radius = 0
        while radius < 8:
            r = radius + 1
            found = False
            for di in range(-r, r + 1):
                for dj in range(-r, r + 1):
                    for dk in range(-r, r + 1):
                        ni, nj, nk = i + di, j + dj, k + dk
                        if (0 <= ni < nx and 0 <= nj < ny and 0 <= nk < nz
                                and blocked[ni, nj, nk]):
                            found = True
                            break
                    if found:
                        break
                if found:
                    break
            if found:
                break
            radius = r
        best = min(best, radius)
    return best


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pcd", default="maps/chemical_plant_v4_3d_occupied.pcd")
    parser.add_argument("--grid2d", default="maps/chemical_plant_v4_3d_projected.yaml")
    parser.add_argument("--cell", type=float, default=0.3)
    args = parser.parse_args()

    points = load_pcd(args.pcd)
    print(f"occupied voxels: {len(points)} from {args.pcd}")
    print(f"grid cell      : {args.cell:.2f} m")

    unknown = None
    try:
        unknown = load_2d_unknown(args.grid2d)
        print(f"unknown 2D cells: {len(unknown[0])} from {args.grid2d}")
    except Exception as exc:  # noqa: BLE001
        print(f"(no conservative model: {exc})")

    total = 0
    for label, radius, altitude in [("dog", 0.35, 0.5), ("drone", 0.5, 6.0)]:
        blocked, _ = build_grid(points, args.cell, radius)
        total += run(blocked, args.cell, altitude, f"optimistic / {label}", radius,
                     ground=(label == "dog"))
        if unknown is not None:
            blocked, _ = build_grid(points, args.cell, radius, unknown_block=unknown)
            total += run(blocked, args.cell, altitude, f"conservative / {label}", radius,
                         ground=(label == "dog"))

    expected = len(GOALS) * (4 if unknown is not None else 2)
    print(f"\ntotal reachable goals: {total}/{expected}")
    return 0 if total == expected else 1


if __name__ == "__main__":
    sys.exit(main())
