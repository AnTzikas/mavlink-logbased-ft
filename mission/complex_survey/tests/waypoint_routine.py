#!/usr/bin/env python3
"""
Test: target a SPECIFIC drone by sysid, fly to a waypoint, and run the
FULL inspection routine exactly as mission.py does it:

  goto WP
    -> INSPECT NADIR (gimbal straight down, capture, FTP download, detect)
       -> if an anomaly is found, run the STAGE-2 four-side sweep:
            for each of south/east/north/west:
              goto a 5 m offset hover point at confirm-alt
              point gimbal obliquely toward the target
              capture, FTP download, detect
       -> print a per-side summary
  return home
  land

Requires camera_component.py running for THIS drone's sysid, e.g.:
    python3 camera_component.py --connection udp:127.0.0.1:14552 --sysid 1

Usage (single shared socket, two drones present -> pick one with --sysid):
    python3 tests/test_full_routine.py \
        --connection udp:127.0.0.1:14551 \
        --sysid 2 \
        --lat 39.35274776 --lon 22.94155630

This reuses mission.py's own geometry/gimbal helpers and detector, so the
routine is identical to production -- it is a focused harness for ONE drone,
not a re-implementation.
"""

import argparse
import os
import sys
import time
from pathlib import Path

# tests/ lives under complex_survey/; add the parent so drone/mission/etc resolve.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from pymavlink import mavutil

from drone import Drone, MAVLinkDispatcher
from mavftp import FtpClient, FtpError
from detector import Detector
# Reuse the EXACT helpers the mission uses, so geometry + gimbal math match.
from mission import (
    INSPECT_DIRECTIONS,
    offset_position,
    absolute_bearing_toward_target,
    yaw_relative_to_heading,
)


def _update_for(dispatcher, seconds: float) -> None:
    """Pump the dispatcher for a fixed wall-clock duration."""
    deadline = time.time() + seconds
    while time.time() < deadline:
        dispatcher.update()


def fly_and_wait(drone, dispatcher, lat, lon, alt, tol_m=2.0, timeout_s=60.0) -> bool:
    drone.goto_waypoint(lat, lon, alt, airspeed=5.0)
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        dispatcher.update()
        if drone.distance_to_target() <= tol_m:
            return True
    return False


def do_capture(drone, dispatcher, ftp, request_id, local_path,
               settle_s=2.0, ack_timeout_s=20.0, ftp_timeout_s=10.0) -> str | None:
    """Trigger one external capture and download it. Returns the local path on
    success, or None on failure (no ack / rejected / FTP error). Mirrors
    mission.py's _poll_capture_command flow but in a simple blocking form."""
    # Let the gimbal settle physically before capturing.
    _update_for(dispatcher, settle_s)

    drone.request_external_capture(request_id)

    # Wait for the camera component's ack (pump the link while waiting).
    deadline = time.time() + ack_timeout_s
    ack = None
    while time.time() < deadline:
        dispatcher.update()
        ack = drone.get_camera_ack(request_id)
        if ack is not None:
            break
    if ack is None:
        print("    [CAPTURE] no camera ack within {0:.0f}s -- giving up.".format(ack_timeout_s))
        return None
    if ack != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("    [CAPTURE] camera returned failure (result={0}).".format(ack))
        return None

    remote_path = "logs/capture_{0}.jpg".format(request_id)
    try:
        ftp.get_file(remote_path, local_path, timeout=ftp_timeout_s)
        return local_path
    except FtpError as exc:
        print("    [CAPTURE] FTP download failed: {0}".format(exc))
        return None


def main() -> int:
    p = argparse.ArgumentParser(description="Full inspection-routine test for ONE drone (by sysid).")
    p.add_argument("--connection", required=True, help="MAVLink connection string (mission's port).")
    p.add_argument("--sysid", type=int, required=True, help="Target THIS drone's sysid (e.g. 1 or 2).")
    p.add_argument("--lat", required=True, type=float, help="Waypoint latitude.")
    p.add_argument("--lon", required=True, type=float, help="Waypoint longitude.")
    p.add_argument("--alt-m", type=float, default=10.0, help="Cruise / nadir altitude.")
    p.add_argument("--confirm-alt-m", type=float, default=5.0, help="Stage-2 oblique hover altitude.")
    p.add_argument("--confirm-offset-m", type=float, default=5.0, help="Stage-2 horizontal offset from target.")
    p.add_argument("--anomaly-threshold", type=float, default=0.15, help="Min confidence to treat nadir as 'something here'.")
    p.add_argument("--confirm-threshold", type=float, default=0.6, help="Min confidence to count a side as confirmed.")
    p.add_argument("--settle-s", type=float, default=2.0, help="Gimbal settle time before each capture.")
    p.add_argument("--download-dir", default="test_downloads", help="Where to save captures (per-run subfolder by sysid).")
    p.add_argument("--model-path", default=None, help="YOLO .pt path (defaults to detector's built-in path).")
    p.add_argument("--no-detect", action="store_true", help="Capture + download only; skip running YOLO (still does nadir + sweep using a forced anomaly).")
    args = p.parse_args()

    out_dir = os.path.join(args.download_dir, "sys{0}".format(args.sysid))
    os.makedirs(out_dir, exist_ok=True)

    detector = None
    if not args.no_detect:
        print("[TEST] Loading detector...")
        detector = Detector(model_path=args.model_path) if args.model_path else Detector()

    # --- connect, target the requested sysid ---
    dispatcher = MAVLinkDispatcher(args.connection, discovery_timeout_s=10.0)
    sysids = dispatcher.discover()
    if not sysids:
        print("[TEST] No drones discovered.")
        return 1
    if args.sysid not in sysids:
        print("[TEST] Requested sysid={0} not among discovered {1}.".format(args.sysid, sysids))
        return 1

    drone = Drone(dispatcher, args.sysid)
    dispatcher.register(drone)
    ftp = FtpClient(dispatcher._conn, target_system=args.sysid, target_component=1)
    print("[TEST] Targeting sysid={0} (discovered {1}).".format(args.sysid, sysids))

    # --- arm + takeoff ---
    print("[TEST] Arming + taking off to {0}m...".format(args.alt_m))
    drone.arm()
    drone.takeoff(args.alt_m)
    deadline = time.time() + 60.0
    while time.time() < deadline:
        dispatcher.update()
        if drone.get_altitude() >= args.alt_m - 1.0:
            break
    print("[TEST] At altitude {0:.1f}m.".format(drone.get_altitude()))

    # --- goto the waypoint ---
    print("[TEST] Flying to ({0:.7f}, {1:.7f})...".format(args.lat, args.lon))
    if not fly_and_wait(drone, dispatcher, args.lat, args.lon, args.alt_m):
        print("[TEST] WARNING: did not reach waypoint within timeout, continuing anyway.")
    print("[TEST] Arrived.")

    request_id = 0

    # ================= STAGE 1: NADIR =================
    print("\n[TEST] === STAGE 1: NADIR ===")
    drone.point_gimbal(pitch_deg=-90, yaw_deg=0)   # straight down
    request_id += 1
    nadir_path = os.path.join(out_dir, "nadir.jpg")
    got = do_capture(drone, dispatcher, ftp, request_id, nadir_path, settle_s=args.settle_s)

    anomaly = None
    if got is None:
        print("[TEST] Nadir capture failed -- aborting inspection.")
    elif args.no_detect:
        print("[TEST] --no-detect: forcing a synthetic anomaly to exercise the sweep.")
        anomaly = (2, "car", 1.0)   # class_id, name, conf
    else:
        anomaly = detector.detect_anomaly(nadir_path, threshold=args.anomaly_threshold)
        if anomaly is None:
            print("[TEST] Nadir: empty (no anomaly above threshold).")
        else:
            print("[TEST] Nadir anomaly: {0} (conf={1:.2f}).".format(anomaly[1], anomaly[2]))

    # ================= STAGE 2: FOUR-SIDE SWEEP =================
    if anomaly is not None:
        anomaly_class_id, anomaly_class_name, _ = anomaly
        aim_pitch = -__import__("math").degrees(
            __import__("math").atan2(args.confirm_alt_m, args.confirm_offset_m)
        )
        side_results = []
        confirmations = []
        print("\n[TEST] === STAGE 2: 4-SIDE SWEEP (anomaly={0}) ===".format(anomaly_class_name))
        for direction in INSPECT_DIRECTIONS:
            hov_lat, hov_lon = offset_position(args.lat, args.lon, direction, args.confirm_offset_m)
            bearing = absolute_bearing_toward_target(direction)
            print("[TEST] side={0}: goto offset hover...".format(direction))
            fly_and_wait(drone, dispatcher, hov_lat, hov_lon, args.confirm_alt_m, tol_m=2.0)

            heading = drone.get_heading() or 0.0
            yaw = yaw_relative_to_heading(bearing, heading)
            drone.point_gimbal(pitch_deg=aim_pitch, yaw_deg=yaw)

            request_id += 1
            side_path = os.path.join(out_dir, "side_{0}.jpg".format(direction))
            sgot = do_capture(drone, dispatcher, ftp, request_id, side_path, settle_s=args.settle_s)

            if sgot is None or args.no_detect or detector is None:
                side_results.append((direction, None, None))
                continue

            dets = detector.detect_all(side_path, min_conf=0.05)
            same_conf = None
            best = None
            for cid, name, conf in dets:
                if cid == anomaly_class_id and same_conf is None:
                    same_conf = conf
                if best is None or conf > best[2]:
                    best = (cid, name, conf)
            side_results.append((direction, same_conf, best))

            match_conf = same_conf
            match_name = anomaly_class_name
            if (match_conf is None or match_conf < args.confirm_threshold) and best is not None:
                if best[2] >= args.confirm_threshold:
                    match_conf, match_name = best[2], best[1]
            if match_conf is not None and match_conf >= args.confirm_threshold:
                confirmations.append((direction, match_conf, match_name))

        # summary
        print("\n[TEST] --- SWEEP SUMMARY (anomaly: {0}) ---".format(anomaly_class_name))
        for direction, same_conf, best in side_results:
            same_s = "{0:.2f}".format(same_conf) if same_conf is not None else "-"
            best_s = "{0} {1:.2f}".format(best[1], best[2]) if best else "none"
            print("    {0:<6} same-class={1:<5} best={2}".format(direction, same_s, best_s))
        if confirmations:
            d, c, n = max(confirmations, key=lambda x: x[1])
            print("  -> CONFIRMED: {0} (conf={1:.2f}) from {2} side.".format(n, c, d))
        else:
            print("  -> UNCONFIRMED: nadir anomaly not corroborated by any side.")

    # ================= RETURN + LAND =================
    print("\n[TEST] Returning home + landing...")
    home = drone.get_home()
    fly_and_wait(drone, dispatcher, home.lat, home.lon, args.alt_m, tol_m=3.0)
    drone.land()
    deadline = time.time() + 60.0
    while time.time() < deadline:
        dispatcher.update()
        if not drone.is_armed():
            break

    dispatcher.close()
    print("[TEST] Done. Captures saved under {0}/".format(out_dir))
    return 0


if __name__ == "__main__":
    sys.exit(main())