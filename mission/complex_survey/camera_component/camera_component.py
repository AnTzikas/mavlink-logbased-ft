#!/usr/bin/env python3
"""
Camera component.

Plays the role of a physical camera module attached to the drone --
identifies itself with the SAME sysid as the drone, but a DIFFERENT
component id (MAV_COMP_ID_CAMERA), exactly like a real camera component
would. ArduPilot itself never needs to know this command exists, since
it's addressed to our own (sysid, compid), not the autopilot's.

Flow:
    1. Listen for MAV_CMD_USER_1, addressed to our own (sysid, compid).
       param1 carries a request_id chosen by the caller -- this becomes
       part of the remote filename, so the caller already knows exactly
       what file to expect without us needing to send it back.
    2. Capture a frame from Gazebo (reuses camera.py as-is).
    3. PUT it to the autopilot's logs/ directory via MAVLink FTP.
    4. Send COMMAND_ACK back to whoever sent the request.

Sends periodic heartbeats so ArduPilot's MAVLink router learns which
channel (sysid, compid) lives on -- without this, ArduPilot wouldn't
know where to forward commands addressed to this component.

IMPORTANT -- connection type matters:
    Connect via a genuine SITL TCP telemetry port (e.g. tcp:127.0.0.1:5762),
    NOT a MAVProxy --out UDP mirror. ArduPilot's MAVLink routing (which
    decides where to forward commands addressed to this component) only
    recognizes genuine distinct links -- the --out UDP mirrors are a
    MAVProxy-level mirroring feature and are NOT treated as separate
    routable links by ArduPilot itself, so commands targeting this
    component would never arrive if connected that way.

    The sender (mission.py) does NOT need to change -- it can stay on
    its existing --out UDP connection. Only the RECEIVING side (this
    component) needs to be on a real link for ArduPilot to know where
    to forward things addressed to it.

Usage:
    python3 camera_component.py --connection tcp:127.0.0.1:5762 --sysid 1
"""
import argparse
import sys
import time
from pathlib import Path
 
# This file lives in camera/, but mavftp.py lives in the project root --
# add the parent directory to sys.path so the import below still resolves.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
 
from pymavlink import mavutil
 
from camera import Camera
from mavftp import FtpClient, FtpError

MAV_COMP_ID_CAMERA = mavutil.mavlink.MAV_COMP_ID_CAMERA   # 100, standard reserved id
CAPTURE_COMMAND    = mavutil.mavlink.MAV_CMD_USER_1        # custom, ArduPilot ignores this


def main() -> int:
    parser = argparse.ArgumentParser(description="Camera component -- listens for capture requests.")
    parser.add_argument("--connection", required=True,
                         help="MAVLink connection string. Must be a genuine SITL TCP "
                              "telemetry port (e.g. tcp:127.0.0.1:5762), NOT a --out UDP mirror.")
    parser.add_argument("--sysid", type=int, required=True, help="System id of the drone this camera is attached to.")
    parser.add_argument("--autopilot-compid", type=int, default=1, help="Component id of the autopilot (usually 1).")
    parser.add_argument("--world-name", default="large_mission")
    parser.add_argument("--drone-name", default="drone1")
    parser.add_argument("--capture-dir", default="camera_component_captures",
                         help="Local scratch directory for grabbed frames before upload.")
    parser.add_argument("--remote-dir", default="logs",
                         help="Directory on the autopilot's filesystem to upload into.")
    args = parser.parse_args()

    print("[CAMERA_COMPONENT] Connecting on {0} as sysid={1} compid={2}...".format(
        args.connection, args.sysid, MAV_COMP_ID_CAMERA))
    conn = mavutil.mavlink_connection(
        args.connection,
        source_system=args.sysid,
        source_component=MAV_COMP_ID_CAMERA,
    )

    camera = Camera(
        output_dir=args.capture_dir,
        world_name=args.world_name,
        drone_name=args.drone_name,
    )
    ftp = FtpClient(conn, target_system=args.sysid, target_component=args.autopilot_compid)

    print("[CAMERA_COMPONENT] Enabling Gazebo camera stream...")
    camera.enable_stream()

    last_heartbeat_sent = 0.0

    print("[CAMERA_COMPONENT] Ready. Listening for capture requests...")
    while True:
        now = time.time()
        if now - last_heartbeat_sent >= 1.0:
            # Required for ArduPilot's MAVLink router to learn that
            # (sysid, MAV_COMP_ID_CAMERA) lives on this channel --
            # without this, commands addressed to us would never
            # get forwarded here by ArduPilot.
            conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_CAMERA,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0,
            )
            last_heartbeat_sent = now

        msg = conn.recv_match(type="COMMAND_LONG", blocking=True, timeout=0.5)
        if msg is None:
            continue

        if msg.command != CAPTURE_COMMAND:
            continue
        if msg.target_system != args.sysid or msg.target_component != MAV_COMP_ID_CAMERA:
            continue

        requester_sysid = msg.get_srcSystem()
        requester_compid = msg.get_srcComponent()
        request_id = int(msg.param1)

        print("[CAMERA_COMPONENT] Capture request id={0} from sysid={1} compid={2}".format(
            request_id, requester_sysid, requester_compid))

        try:
            local_path = camera.capture("capture_{0}".format(request_id))
            if local_path is None:
                raise RuntimeError("Gazebo capture failed")

            remote_path = "{0}/capture_{1}.jpg".format(args.remote_dir, request_id)
            ftp.put_file(local_path, remote_path)
            print("[CAMERA_COMPONENT] Uploaded -> {0}".format(remote_path))

            conn.mav.command_ack_send(
                CAPTURE_COMMAND,
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
            )

        except (FtpError, Exception) as exc:
            print("[CAMERA_COMPONENT] Capture/upload failed: {0}".format(exc))
            conn.mav.command_ack_send(
                CAPTURE_COMMAND,
                mavutil.mavlink.MAV_RESULT_FAILED,
            )


if __name__ == "__main__":
    sys.exit(main())