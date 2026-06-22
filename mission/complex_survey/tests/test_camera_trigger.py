#!/usr/bin/env python3
"""
Test: goto a waypoint, then trigger camera_component.py to capture and
upload an image, then download it back via MAVLink FTP.

Requires camera_component.py to already be running:
    python3 camera_component.py --connection udp:127.0.0.1:14552 --sysid 1

Usage:
    python3 test_camera_trigger.py --connection udp:127.0.0.1:14551 \\
        --lat 39.35274776 --lon 22.94155630
"""

import argparse
import sys
import time

from pymavlink import mavutil

from drone import Drone, MAVLinkDispatcher
from mavftp import FtpClient, FtpError


MAV_COMP_ID_CAMERA = mavutil.mavlink.MAV_COMP_ID_CAMERA   # 100
CAPTURE_COMMAND    = mavutil.mavlink.MAV_CMD_USER_1        # 31010


def fly_and_wait(drone, dispatcher, lat, lon, alt, pos_tolerance_m=2.0, timeout_s=30.0):
    drone.goto_waypoint(lat, lon, alt)
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        dispatcher.update()
        if drone.distance_to_target() <= pos_tolerance_m:
            return True
        time.sleep(0.3)
    return False


def request_capture(dispatcher, sysid, request_id) -> None:
    """Sends MAV_CMD_USER_1 directly to the camera component's compid.
    Not using dispatcher.send_command_long() here since that hardcodes
    target_component=1 (the autopilot) -- we need compid=100 instead."""
    dispatcher._conn.mav.command_long_send(
        sysid, MAV_COMP_ID_CAMERA,
        CAPTURE_COMMAND, 0,
        float(request_id), 0, 0, 0, 0, 0, 0,
    )


def wait_for_camera_ack(dispatcher, timeout_s=5.0) -> bool:
    """Waits for a COMMAND_ACK whose SOURCE component is the camera
    (compid=100) -- COMMAND_ACK itself has no addressing fields, so we
    identify the sender via the message's transport-level source."""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = dispatcher._conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.get_srcComponent() != MAV_COMP_ID_CAMERA:
            continue
        if msg.command != CAPTURE_COMMAND:
            continue
        print("[TEST] Camera ACK: result={0}".format(msg.result))
        return msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
    return False


def main() -> int:
    parser = argparse.ArgumentParser(description="Test goto + camera trigger + FTP download")
    parser.add_argument("--connection", required=True, help="MAVLink connection string (mission's port).")
    parser.add_argument("--lat", required=True, type=float)
    parser.add_argument("--lon", required=True, type=float)
    parser.add_argument("--alt-m", type=float, default=15.0)
    parser.add_argument("--request-id", type=int, default=1)
    parser.add_argument("--download-dir", default="test_downloads")
    args = parser.parse_args()

    import os
    os.makedirs(args.download_dir, exist_ok=True)

    dispatcher = MAVLinkDispatcher(args.connection, discovery_timeout_s=10.0)
    sysids = dispatcher.discover()
    if not sysids:
        print("[TEST] No drones discovered.")
        return 1

    sysid = sysids[0]
    drone = Drone(dispatcher, sysid)
    dispatcher.register(drone)

    print("[TEST] Arming and taking off to {0}m...".format(args.alt_m))
    drone.arm()
    drone.takeoff(args.alt_m)
    deadline = time.time() + 30.0
    while time.time() < deadline:
        dispatcher.update()
        if drone.get_altitude() >= args.alt_m - 1.0:
            break
        time.sleep(0.5)
    print("[TEST] Reached altitude.")

    print("[TEST] Flying to ({0}, {1})...".format(args.lat, args.lon))
    fly_and_wait(drone, dispatcher, args.lat, args.lon, args.alt_m)
    print("[TEST] Arrived.")

    print("\n[TEST] Sending capture request id={0} to camera component...".format(args.request_id))
    request_capture(dispatcher, sysid, args.request_id)

    print("[TEST] Waiting for camera ACK...")
    confirmed = wait_for_camera_ack(dispatcher, timeout_s=10.0)
    if not confirmed:
        print("[TEST] FAILED -- no ACK (or rejected) from camera component.")
    else:
        remote_path = "logs/capture_{0}.jpg".format(args.request_id)
        local_path  = "{0}/capture_{1}.jpg".format(args.download_dir, args.request_id)
        print("[TEST] Downloading {0} via MAVFTP...".format(remote_path))

        ftp = FtpClient(dispatcher._conn, target_system=sysid, target_component=1)
        try:
            ftp.get_file(remote_path, local_path, timeout=10.0)
            print("[TEST] SUCCESS -- saved to {0}".format(local_path))
        except FtpError as exc:
            print("[TEST] FTP download failed: {0}".format(exc))

    print("\n[TEST] Returning home and landing...")
    drone.goto_home()
    deadline = time.time() + 30.0
    while time.time() < deadline:
        dispatcher.update()
        if drone.distance_to_target() <= 3.0:
            break
        time.sleep(0.5)

    drone.land()
    deadline = time.time() + 30.0
    while time.time() < deadline:
        dispatcher.update()
        if not drone.is_armed():
            break
        time.sleep(0.5)

    dispatcher.close()
    print("[TEST] Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())