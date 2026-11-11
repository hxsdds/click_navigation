#!/usr/bin/env python3
"""Send a MAVLink v1 GCS heartbeat to a local PX4 SITL UDP port."""

import argparse
import socket
import struct
import time


def x25(data):
    crc = 0xFFFF
    for value in data:
        value ^= crc & 0xFF
        value ^= (value << 4) & 0xFF
        crc = ((crc >> 8) ^ (value << 8) ^ (value << 3) ^ (value >> 4)) & 0xFFFF
    return crc


def heartbeat(sequence):
    # HEARTBEAT: MAV_TYPE_GCS=6, MAV_AUTOPILOT_INVALID=8, MAV_STATE_ACTIVE=4.
    payload = struct.pack("<IBBBBB", 0, 6, 8, 0, 4, 3)
    header = bytes((len(payload), sequence & 0xFF, 255, 190, 0))
    checksum = x25(header + payload + bytes((50,)))
    return bytes((0xFE,)) + header + payload + struct.pack("<H", checksum)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=18570)
    parser.add_argument("--duration", type=float, default=180.0)
    args = parser.parse_args()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        end = time.monotonic() + args.duration
        sequence = 0
        while time.monotonic() < end:
            sock.sendto(heartbeat(sequence), ("127.0.0.1", args.port))
            sequence += 1
            time.sleep(0.5)


if __name__ == "__main__":
    main()
