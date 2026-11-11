#!/usr/bin/env python3
"""Query static SDF box/cylinder/sphere surfaces, including nested poses."""
import argparse
import math
import xml.etree.ElementTree as ET
import numpy as np


def pose(element):
    v = [float(x) for x in (element.findtext("pose") or "0 0 0 0 0 0").split()]
    x, y, z, r, p, yaw = v
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(yaw), math.sin(yaw)
    t = np.eye(4)
    t[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                 [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    t[:3, 3] = [x, y, z]
    return t


def surfaces(element, transform=None, name=""):
    transform = np.eye(4) if transform is None else transform
    for child in element:
        if child.tag in ("model", "link", "collision", "visual"):
            path = name + "/" + child.get("name", child.tag)
            total = transform @ pose(child)
            if child.tag in ("collision", "visual"):
                yield child.tag, path, total, child.find("geometry")
            else:
                yield from surfaces(child, total, path)


def distance(point, transform, geometry):
    q = transform[:3, :3].T @ (point - transform[:3, 3])
    box = geometry.find("box/size")
    if box is not None:
        delta = np.abs(q) - np.array([float(x) for x in box.text.split()]) / 2
        return float(np.linalg.norm(np.maximum(delta, 0)) + min(max(delta), 0))
    cylinder = geometry.find("cylinder")
    if cylinder is not None:
        delta = np.array([np.linalg.norm(q[:2])-float(cylinder.findtext("radius")), abs(q[2])-float(cylinder.findtext("length"))/2])
        return float(np.linalg.norm(np.maximum(delta, 0)) + min(max(delta), 0))
    sphere = geometry.find("sphere/radius")
    if sphere is not None:
        return float(np.linalg.norm(q)-float(sphere.text))
    return math.inf


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("world")
    parser.add_argument("point", nargs=3, type=float)
    args = parser.parse_args()
    root = ET.parse(args.world).getroot().find("world")
    rows = [(distance(np.array(args.point), t, g), kind, name)
            for kind, name, t, g in surfaces(root) if g is not None]
    for row in sorted(rows)[:16]:
        print(f"{row[0]:.3f} m {row[1]} {row[2]}")


if __name__ == "__main__":
    main()
