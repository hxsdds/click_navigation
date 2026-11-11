#!/usr/bin/env python3
"""Add an idempotent, measurement-only odometry plugin to the SITL x500."""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("model_sdf")
    args = parser.parse_args()
    path = Path(args.model_sdf)
    tree = ET.parse(path)
    model = tree.getroot().find("model")
    name = "gz::sim::systems::OdometryPublisher"
    if any(plugin.get("name") == name for plugin in model.findall("plugin")):
        return
    plugin = ET.SubElement(model, "plugin", filename="gz-sim-odometry-publisher-system", name=name)
    for key, value in (("dimensions", "3"), ("odom_frame", "map"),
                       ("robot_base_frame", "x500_0/base_link"),
                       ("odom_topic", "/px4/ground_truth/odometry"),
                       ("odom_publish_frequency", "30")):
        ET.SubElement(plugin, key).text = value
    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    main()
