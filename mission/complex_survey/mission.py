import argparse
import math
import os
import re
import sys
import time
import shlex
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, List, Optional, Sequence, Tuple

from pymavlink import mavutil

from drone import Drone, Waypoint, MAVLinkDispatcher
from mavftp import FtpClient, FtpError
from detector import Detector


MISSION_CMD_NAV_WAYPOINT         = 16
MISSION_CMD_NAV_LOITER_UNLIM     = 17
MISSION_CMD_NAV_LOITER_TURNS     = 18
MISSION_CMD_NAV_RETURN_TO_LAUNCH = 20
MISSION_CMD_NAV_LAND             = 21
MISSION_CMD_NAV_TAKEOFF          = 22
MISSION_CMD_NAV_SPLINE_WAYPOINT  = 82

INSPECT_DIRECTIONS = ["south", "east", "north", "west"]


@dataclass
class MissionWaypoint:
    seq: int
    lat: float
    lon: float
    alt: float
    source_index: int


@dataclass
class DroneCommand:
    kind: str
    label: str
    waypoint: Optional[MissionWaypoint] = None
    home_target: Optional[Waypoint] = None
    target_point: Optional[Waypoint] = None     # for goto_side
    direction: Optional[str] = None             # for goto_side / capture_side
    bearing_deg: Optional[float] = None         # for capture_side
    pitch_deg: Optional[float] = None           # for capture_side
    request_id: Optional[int] = None            # for inspect_nadir / capture_side
    altitude: Optional[float] = None
    timeout_s: float = 60.0
    tolerance_m: float = 2.0
    issued: bool = False
    issued_at: Optional[float] = None           # wall-clock stamp (sentinel/guard only)
    last_progress_at: Optional[float] = None    # wall-clock stamp (vestigial; vt version is authoritative)
    issued_at_vt: Optional[int] = None          # vehicle-time (time_boot_ms) stamp at issue
    last_progress_vt: Optional[int] = None      # vehicle-time stamp at last progress
    best_metric: Optional[float] = None


@dataclass
class DroneRuntime:
    name: str
    drone: Drone
    home: Waypoint
    takeoff_alt_m: float
    airspeed_m_s: float
    phase: str = "READY"
    assigned_waypoint: Optional[MissionWaypoint] = None
    command_queue: List[DroneCommand] = field(default_factory=list)
    command_index: int = 0
    completed_waypoints: int = 0
    failed: bool = False
    failure_reason: Optional[str] = None
    ftp: Optional["FtpClient"] = None   # set in connect_all(), used for capture downloads

    # --- inspection state, reset at the start of each waypoint's inspection ---
    anomaly_class_id: Optional[int] = None
    anomaly_class_name: Optional[str] = None
    anomaly_confidence: Optional[float] = None
    confirmations: List[tuple] = field(default_factory=list)
    side_results: List[tuple] = field(default_factory=list)
    inspect_sides_completed: int = 0

    def active_command(self) -> Optional[DroneCommand]:
        if 0 <= self.command_index < len(self.command_queue):
            return self.command_queue[self.command_index]
        return None

    def is_idle(self) -> bool:
        return not self.failed and self.assigned_waypoint is None and not self.command_queue

    def reset(self) -> None:
        self.phase             = "READY"
        self.assigned_waypoint = None
        self.command_queue     = []
        self.command_index     = 0

    def reset_inspection_state(self) -> None:
        self.anomaly_class_id      = None
        self.anomaly_class_name    = None
        self.anomaly_confidence    = None
        self.confirmations         = []
        self.side_results          = []
        self.inspect_sides_completed = 0


def haversine_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    from math import atan2, cos, radians, sin, sqrt
    R       = 6371000.0
    phi1    = radians(lat1)
    phi2    = radians(lat2)
    dphi    = radians(lat2 - lat1)
    dlambda = radians(lon2 - lon1)
    a = sin(dphi / 2.0) ** 2 + cos(phi1) * cos(phi2) * sin(dlambda / 2.0) ** 2
    return R * 2.0 * atan2(sqrt(a), sqrt(1.0 - a))


def offset_position(lat: float, lon: float, direction: str, distance_m: float) -> Tuple[float, float]:
    """Returns (lat, lon) offset from the given point by distance_m in the
    given compass direction (north/south/east/west)."""
    lat_scale = 111195.0
    lon_scale = 111195.0 * math.cos(math.radians(lat))

    if direction == "north":
        return lat + distance_m / lat_scale, lon
    if direction == "south":
        return lat - distance_m / lat_scale, lon
    if direction == "east":
        return lat, lon + distance_m / lon_scale
    if direction == "west":
        return lat, lon - distance_m / lon_scale
    raise ValueError("Unknown direction: {0}".format(direction))


def absolute_bearing_toward_target(direction: str) -> float:
    """Standard compass bearing FROM a hover position offset in `direction`
    TOWARD the target it was offset from (0=North, 90=East, 180=South, 270=West)."""
    return {
        "south": 0.0,
        "north": 180.0,
        "east":  270.0,
        "west":  90.0,
    }[direction]


def yaw_relative_to_heading(absolute_bearing_deg: float, vehicle_heading_deg: float) -> float:
    """Converts a desired absolute compass bearing into the yaw value to
    send to point_gimbal(), correcting for the vehicle's actual current
    heading. Result is normalized to -180..180."""
    return (absolute_bearing_deg - vehicle_heading_deg + 180.0) % 360.0 - 180.0


def parse_mission_file(mission_path: str) -> Tuple[List[MissionWaypoint], Optional[float], List[int]]:
    with open(mission_path, "r", encoding="utf-8") as handle:
        lines = [line.strip() for line in handle if line.strip()]

    if not lines:
        raise ValueError("Mission file is empty.")
    if not lines[0].startswith("QGC WPL"):
        raise ValueError("Mission file does not look like a QGC waypoint file.")

    parsed_waypoints: List[MissionWaypoint] = []
    takeoff_alt_m    = None
    ignored_commands = set()

    for line in lines[1:]:
        parts = re.split(r"\s+", line)
        if len(parts) < 12:
            raise ValueError("Malformed mission line: {0}".format(line))

        mission_index = int(parts[0])
        current_flag  = int(parts[1])
        command       = int(parts[3])
        lat           = float(parts[8])
        lon           = float(parts[9])
        alt           = float(parts[10])

        if mission_index == 0 and current_flag == 1 and command == MISSION_CMD_NAV_WAYPOINT:
            continue
        if command == MISSION_CMD_NAV_TAKEOFF:
            if takeoff_alt_m is None:
                takeoff_alt_m = max(1.0, alt)
            continue
        if command in (MISSION_CMD_NAV_WAYPOINT, MISSION_CMD_NAV_SPLINE_WAYPOINT):
            parsed_waypoints.append(
                MissionWaypoint(
                    seq=len(parsed_waypoints) + 1,
                    lat=lat, lon=lon, alt=alt,
                    source_index=mission_index,
                )
            )
            continue
        if command in (
            MISSION_CMD_NAV_LAND,
            MISSION_CMD_NAV_RETURN_TO_LAUNCH,
            MISSION_CMD_NAV_LOITER_UNLIM,
            MISSION_CMD_NAV_LOITER_TURNS,
        ):
            continue
        ignored_commands.add(command)

    if not parsed_waypoints:
        raise ValueError("No navigation waypoints found in mission file.")
    if takeoff_alt_m is None:
        takeoff_alt_m = max(5.0, parsed_waypoints[0].alt)

    return parsed_waypoints, takeoff_alt_m, sorted(ignored_commands)


def _resolve_cli_tokens(argv: Optional[Sequence[str]]) -> Optional[Sequence[str]]:
    """
    Decide where the mission parameters come from, in priority order:
      1. argv passed to main() directly (tests / programmatic callers)
      2. real command-line args  (python3 mission.py --connection ...)
      3. a config file of CLI-style tokens, path from $MISSION_CONFIG
         (default: mission.conf in the working dir)

    The file uses the SAME flags as the CLI -- one or many per line,
    with '#' comments allowed:

        --connection udp:127.0.0.1:14550
        --mission    /app/missions/large.waypoints
        --airspeed-m-s 5        # cruise speed
    """
    if argv is not None:
        return argv                      # explicit caller wins
    if len(sys.argv) > 1:
        return None                      # real CLI args -> let argparse read sys.argv

    config_path = os.environ.get("MISSION_CONFIG", "mission.conf")
    if not os.path.exists(config_path):
        sys.exit("[CONFIG] No CLI args and config file not found: {0}".format(config_path))

    tokens: List[str] = []
    with open(config_path, "r", encoding="utf-8") as handle:
        for raw in handle:
            line = raw.split("#", 1)[0].strip()   # strip comments + whitespace
            if line:
                tokens.extend(shlex.split(line))   # handles quotes, multiple per line

    print("[CONFIG] Loaded {0} tokens from {1}".format(len(tokens), config_path))
    return tokens


class MissionController:

    def __init__(
        self,
        connection_string: str,
        route_waypoints: Sequence[MissionWaypoint],
        takeoff_alt_m: float,
        airspeed_m_s: float,
        poll_interval_s: float,
        waypoint_tolerance_m: float,
        takeoff_tolerance_m: float,
        landing_tolerance_m: float,
        status_interval_s: float,
        heartbeat_timeout_s: float,
        no_progress_timeout_s: float,
        battery_threshold_pct: int = 20,
        charge_delay_s: float = 60.0,
        discovery_timeout_s: float = 10.0,
        capture_settle_s: float = 2.0,
        capture_dir: str = "captures",
        confirm_alt_m: float = 5.0,
        confirm_offset_m: float = 5.0,
        anomaly_threshold: float = 0.15,
        confirm_threshold: float = 0.6,
        camera_ack_timeout_s: float = 20.0,
        model_path: Optional[str] = None,
    ) -> None:
        self.connection_string     = connection_string
        self.dispatcher: Optional[MAVLinkDispatcher] = None
        self.pending_waypoints: Deque[MissionWaypoint] = deque(route_waypoints)
        self.takeoff_alt_m         = takeoff_alt_m
        self.airspeed_m_s          = airspeed_m_s
        # poll_interval_s is currently only referenced by the (commented-out)
        # sleep in run(); pacing is handled by the blocking recv inside
        # dispatcher.update(). Kept so the sleep can be re-enabled if needed.
        self.poll_interval_s       = poll_interval_s
        self.waypoint_tolerance_m  = waypoint_tolerance_m
        self.takeoff_tolerance_m   = takeoff_tolerance_m
        self.landing_tolerance_m   = landing_tolerance_m
        self.status_interval_s     = status_interval_s
        self.heartbeat_timeout_s   = heartbeat_timeout_s
        self.no_progress_timeout_s = no_progress_timeout_s
        self.battery_threshold_pct = battery_threshold_pct
        self.charge_delay_s        = charge_delay_s
        self.discovery_timeout_s   = discovery_timeout_s
        self.capture_settle_s      = capture_settle_s
        self.confirm_alt_m         = confirm_alt_m
        self.confirm_offset_m      = confirm_offset_m
        self.anomaly_threshold     = anomaly_threshold
        self.confirm_threshold     = confirm_threshold
        # Must exceed camera_component.py's worst-case retry budget (several
        # attempts with exponential backoff + ffmpeg time per attempt),
        # otherwise mission.py gives up on the ack before the camera even
        # finishes its own legitimate retries. Default 20s is comfortable.
        self.camera_ack_timeout_s  = camera_ack_timeout_s
        self.drones: List[DroneRuntime] = []
        self.total_waypoints       = len(route_waypoints)
        self.finished_waypoints    = 0
        self.last_status_at        = 0.0
        self.capture_dir           = capture_dir   # local dir for FTP-downloaded captures
        self._request_id_counter   = 0

        print("[DETECTOR] Loading YOLO model (this may take a moment)...")
        self.detector = Detector(model_path=model_path) if model_path else Detector()

    def _next_request_id(self) -> int:
        self._request_id_counter += 1
        return self._request_id_counter

    # -----------------------------------------------------------------------
    # Startup
    # -----------------------------------------------------------------------

    def connect_all(self) -> None:
        self.dispatcher = MAVLinkDispatcher(
            self.connection_string,
            discovery_timeout_s=self.discovery_timeout_s,
        )

        sysids = self.dispatcher.discover()
        if not sysids:
            raise RuntimeError("No drones discovered on {0}".format(self.connection_string))

        for sysid in sysids:
            name = "drone-{0}".format(sysid)
            print("[CONNECT] {0} sysid={1}".format(name, sysid))

            self.dispatcher.send_command_long(
                sysid, mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
            )

            drone = Drone(self.dispatcher, sysid)
            drone.status    = "IDLE"
            drone.airspeed  = self.airspeed_m_s
            drone.parameters["WPNAV_SPEED"] = int(self.airspeed_m_s * 100)

            self.dispatcher.register(drone)

            home = self._resolve_home(drone)
            runtime = DroneRuntime(
                name=name,
                drone=drone,
                home=home,
                takeoff_alt_m=self.takeoff_alt_m,
                airspeed_m_s=self.airspeed_m_s,
                ftp=FtpClient(self.dispatcher._conn, target_system=sysid, target_component=1),
            )
            self.drones.append(runtime)
            print("[READY] {0} sysid={1} home=({2:.5f}, {3:.5f})".format(
                name, sysid, home.lat, home.lon))

    def _resolve_home(self, drone: Drone) -> Waypoint:
        for _ in range(20):
            self.dispatcher.update()
            if drone._home is not None:
                return Waypoint(drone._home.lat, drone._home.lon, 0)
            time.sleep(0.5)
        pos = drone.get_position()
        return Waypoint(pos.lat, pos.lon, 0)

    # -----------------------------------------------------------------------
    # Main loop
    # -----------------------------------------------------------------------

    def run(self) -> None:
        if not self.drones:
            raise RuntimeError("No connected drones available.")

        while True:
            self.dispatcher.update()
            self._assign_waypoints_to_idle_drones()

            for runtime in self.drones:
                if runtime.failed:
                    continue
                self._poll_drone(runtime)

            self._report_status_if_needed()

            if self._all_work_completed():
                print("[DONE] Mission completed. Visited {0}/{1} waypoints.".format(
                    self.finished_waypoints, self.total_waypoints))
                return

            if self._mission_is_stuck():
                raise RuntimeError("All drones failed. Mission cannot continue.")

            # Pacing is handled by the blocking recv (with timeout) inside
            # dispatcher.update(), so this explicit sleep is intentionally
            # left disabled. Re-enable if update() is ever made non-blocking.
            # time.sleep(self.poll_interval_s)

    def close(self) -> None:
        if self.dispatcher is not None:
            try:
                self.dispatcher.close()
            except Exception:
                pass

    # -----------------------------------------------------------------------
    # Waypoint assignment -- one at a time, only for idle (grounded) drones
    # -----------------------------------------------------------------------

    def _assign_waypoints_to_idle_drones(self) -> None:
        for runtime in self.drones:
            if runtime.failed or not runtime.is_idle():
                continue
            if not self.pending_waypoints:
                continue

            waypoint = self.pending_waypoints.popleft()
            runtime.assigned_waypoint = waypoint
            runtime.reset_inspection_state()
            runtime.command_queue     = self._build_command_queue(runtime, waypoint)
            runtime.command_index     = 0
            runtime.phase             = "ASSIGNED"
            print("[ASSIGN] {0} <- WP#{1} ({2:.5f}, {3:.5f})".format(
                runtime.name, waypoint.source_index, waypoint.lat, waypoint.lon))

    def _build_inspection_commands(self, waypoint: MissionWaypoint) -> List[DroneCommand]:
        """The inspection phase of a waypoint visit: a single INSPECT_NADIR.
        If the nadir capture finds an anomaly, the Stage-2 four-side
        commands are inserted dynamically afterwards (see
        _handle_inspect_nadir_complete)."""
        return [DroneCommand(
            kind="inspect_nadir",
            label="INSPECT NADIR WP#{0}".format(waypoint.source_index),
            waypoint=waypoint,
            timeout_s=self.capture_settle_s + 10.0,
        )]

    def _build_command_queue(self, runtime: DroneRuntime, waypoint: MissionWaypoint) -> List[DroneCommand]:
        """
        Builds ARM -> TAKEOFF -> GOTO -> [inspection commands] ->
        RETURN HOME. LAND / DISARM are NOT added here -- inserted
        dynamically by _decide_after_return_home(). Stage-2 inspect_side
        commands are ALSO inserted dynamically, right after INSPECT_NADIR,
        only if an anomaly is found there.
        """
        return [
            DroneCommand(kind="arm",    label="ARM",    timeout_s=30.0, tolerance_m=0.0),
            DroneCommand(
                kind="takeoff",
                label="TAKEOFF {0:.1f}m".format(runtime.takeoff_alt_m),
                altitude=runtime.takeoff_alt_m,
                timeout_s=90.0,
                tolerance_m=self.takeoff_tolerance_m,
            ),
            DroneCommand(
                kind="goto",
                label="GOTO WP#{0}".format(waypoint.source_index),
                waypoint=waypoint,
                timeout_s=self.no_progress_timeout_s,
                tolerance_m=self.waypoint_tolerance_m,
            ),
            *self._build_inspection_commands(waypoint),
            DroneCommand(
                kind="goto_home",
                label="RETURN HOME",
                home_target=runtime.home,
                altitude=runtime.takeoff_alt_m,
                timeout_s=self.no_progress_timeout_s,
                tolerance_m=self.waypoint_tolerance_m,
            ),
        ]

    def _build_next_waypoint_commands(self, runtime: DroneRuntime, waypoint: MissionWaypoint) -> List[DroneCommand]:
        """Command queue when drone is already in the air and battery is ok.
        No ARM or TAKEOFF needed."""
        return [
            DroneCommand(
                kind="goto",
                label="GOTO WP#{0}".format(waypoint.source_index),
                waypoint=waypoint,
                timeout_s=self.no_progress_timeout_s,
                tolerance_m=self.waypoint_tolerance_m,
            ),
            *self._build_inspection_commands(waypoint),
            DroneCommand(
                kind="goto_home",
                label="RETURN HOME",
                home_target=runtime.home,
                altitude=runtime.takeoff_alt_m,
                timeout_s=self.no_progress_timeout_s,
                tolerance_m=self.waypoint_tolerance_m,
            ),
        ]

    def _build_inspect_side_commands(self, runtime: DroneRuntime, waypoint: MissionWaypoint) -> List[DroneCommand]:
        """Builds the 8 dynamically-inserted Stage 2 commands: for each of
        the 4 directions, a goto_side followed by a capture_side."""
        aim_angle_deg = -math.degrees(
            math.atan2(self.confirm_alt_m, self.confirm_offset_m)
        )
        commands: List[DroneCommand] = []
        for direction in INSPECT_DIRECTIONS:
            hover_lat, hover_lon = offset_position(
                waypoint.lat, waypoint.lon, direction, self.confirm_offset_m
            )
            bearing = absolute_bearing_toward_target(direction)
            commands.append(
                DroneCommand(
                    kind="goto_side",
                    label="GOTO SIDE {0} WP#{1}".format(direction, waypoint.source_index),
                    waypoint=waypoint,
                    direction=direction,
                    target_point=Waypoint(hover_lat, hover_lon, self.confirm_alt_m),
                    timeout_s=self.no_progress_timeout_s,
                    tolerance_m=self.waypoint_tolerance_m,
                )
            )
            commands.append(
                DroneCommand(
                    kind="capture_side",
                    label="CAPTURE SIDE {0} WP#{1}".format(direction, waypoint.source_index),
                    waypoint=waypoint,
                    direction=direction,
                    bearing_deg=bearing,
                    pitch_deg=aim_angle_deg,
                    timeout_s=self.capture_settle_s + 10.0,
                )
            )
        return commands

    def _decide_after_return_home(self, runtime: DroneRuntime) -> None:
        """Called when RETURN HOME completes. Drone is hovering above home.
        Decides what to do next based on battery level and pending waypoints."""
        battery = runtime.drone.get_battery_remaining()
        insert_pos = runtime.command_index + 1

        if battery < self.battery_threshold_pct:
            print("[BATTERY] {0} battery low ({1}%). Landing to recharge.".format(
                runtime.name, battery))
            runtime.command_queue[insert_pos:] = [
                DroneCommand(kind="land",        label="LAND",        timeout_s=120.0, tolerance_m=self.landing_tolerance_m),
                DroneCommand(kind="disarm",       label="DISARM",      timeout_s=30.0,  tolerance_m=0.0),
                DroneCommand(kind="charge_wait",  label="CHARGING",    timeout_s=self.charge_delay_s + 10.0),
            ]
            runtime.assigned_waypoint = None

        elif self.pending_waypoints:
            waypoint = self.pending_waypoints.popleft()
            runtime.assigned_waypoint = waypoint
            runtime.reset_inspection_state()
            print("[ASSIGN] {0} <- WP#{1} (in air, battery {2}%)".format(
                runtime.name, waypoint.source_index, battery))
            runtime.command_queue[insert_pos:] = self._build_next_waypoint_commands(runtime, waypoint)

        else:
            print("[BATTERY] {0} battery ok ({1}%) but no more waypoints. Landing.".format(
                runtime.name, battery))
            runtime.command_queue[insert_pos:] = [
                DroneCommand(kind="land",   label="LAND",   timeout_s=120.0, tolerance_m=self.landing_tolerance_m),
                DroneCommand(kind="disarm", label="DISARM", timeout_s=30.0,  tolerance_m=0.0),
            ]

    # -----------------------------------------------------------------------
    # Drone polling
    # -----------------------------------------------------------------------

    def _poll_drone(self, runtime: DroneRuntime) -> None:
        self._check_health(runtime)
        if runtime.failed:
            return

        command = runtime.active_command()
        if command is None:
            runtime.reset()
            return

        if not command.issued:
            self._issue_command(runtime, command)
            return

        if self._is_command_complete(runtime, command):

            if command.kind == "goto" and runtime.assigned_waypoint is not None:
                self.finished_waypoints += 1
                print("[COMPLETE] {0} visited WP#{1} ({2}/{3})".format(
                    runtime.name,
                    runtime.assigned_waypoint.source_index,
                    self.finished_waypoints,
                    self.total_waypoints,
                ))
                runtime.completed_waypoints += 1

            if command.kind == "inspect_nadir":
                self._handle_inspect_nadir_complete(runtime, command)

            if command.kind == "capture_side":
                self._handle_capture_side_complete(runtime, command)

            if command.kind == "goto_home":
                self._decide_after_return_home(runtime)

            runtime.command_index += 1
            next_cmd      = runtime.active_command()
            runtime.phase = next_cmd.label if next_cmd is not None else "READY"
            return

        self._check_progress(runtime, command)

    # -----------------------------------------------------------------------
    # Inspection result handling
    # -----------------------------------------------------------------------

    def _handle_inspect_nadir_complete(self, runtime: DroneRuntime, command: DroneCommand) -> None:
        """Stage 1 complete: run anomaly detection on the captured nadir
        frame. If an anomaly is found, dynamically insert the Stage 2
        (4-side) inspection commands right after this one."""
        image_path = getattr(command, "_captured_image", None)
        wp_label = "WP#{0}".format(command.waypoint.source_index)
        if image_path is None:
            print("[INSPECT] {0}: capture failed, skipping inspection.".format(wp_label))
            return

        anomaly = self.detector.detect_anomaly(image_path, threshold=self.anomaly_threshold)
        if anomaly is None:
            print("[INSPECT] {0}: empty.".format(wp_label))
            return

        class_id, class_name, conf = anomaly
        runtime.anomaly_class_id   = class_id
        runtime.anomaly_class_name = class_name
        runtime.anomaly_confidence = conf
        print("[INSPECT] {0}: anomaly ({1}, conf={2:.2f}) -- checking 4 sides.".format(
            wp_label, class_name, conf))

        side_commands = self._build_inspect_side_commands(runtime, command.waypoint)
        insert_pos = runtime.command_index + 1
        runtime.command_queue[insert_pos:insert_pos] = side_commands

    def _handle_capture_side_complete(self, runtime: DroneRuntime, command: DroneCommand) -> None:
        """One Stage 2 side complete: run detection, accumulate results.
        When all 4 sides are done, print the final summary."""
        image_path = getattr(command, "_captured_image", None)
        direction = command.direction

        if image_path is None:
            runtime.side_results.append((direction, None, None))
        else:
            all_detections = self.detector.detect_all(image_path, min_conf=0.05)

            same_class_conf = None
            best_overall = None
            for det_class_id, det_name, det_conf in all_detections:
                if det_class_id == runtime.anomaly_class_id and same_class_conf is None:
                    same_class_conf = det_conf
                if best_overall is None or det_conf > best_overall[2]:
                    best_overall = (det_class_id, det_name, det_conf)

            runtime.side_results.append((direction, same_class_conf, best_overall))

            match_conf = same_class_conf
            match_name = runtime.anomaly_class_name
            match_id   = runtime.anomaly_class_id
            if (match_conf is None or match_conf < self.confirm_threshold) and best_overall is not None:
                if best_overall[2] >= self.confirm_threshold:
                    match_conf = best_overall[2]
                    match_name = best_overall[1]
                    match_id   = best_overall[0]

            if match_conf is not None and match_conf >= self.confirm_threshold:
                runtime.confirmations.append((direction, match_conf, match_name, match_id))
                # No per-side print here -- the summary table below covers
                # every side in one place once all 4 are done.

        runtime.inspect_sides_completed += 1

        if runtime.inspect_sides_completed >= len(INSPECT_DIRECTIONS):
            self._print_inspection_summary(runtime, command.waypoint)

    def _print_inspection_summary(self, runtime: DroneRuntime, waypoint: MissionWaypoint) -> None:
        print("\n[INSPECT] WP#{0} summary -- Stage 1: {1} (conf={2:.2f})".format(
            waypoint.source_index, runtime.anomaly_class_name, runtime.anomaly_confidence))
        for direction, same_class_conf, best_overall in runtime.side_results:
            same_str  = "{0:.2f}".format(same_class_conf) if same_class_conf is not None else "-"
            best_str  = "{0} {1:.2f}".format(best_overall[1], best_overall[2]) if best_overall else "none"
            print("    {0:<6} same-class={1:<5} best={2}".format(direction, same_str, best_str))

        if runtime.confirmations:
            direction, conf, name, _id = max(runtime.confirmations, key=lambda c: c[1])
            print("  -> CONFIRMED: {0} (conf={1:.2f}) from {2} side.\n".format(name, conf, direction))
        else:
            print("  -> UNCONFIRMED: anomaly seen at nadir but no side crossed the threshold.\n")

    # -----------------------------------------------------------------------
    # Command issuing
    # -----------------------------------------------------------------------

    def _issue_command(self, runtime: DroneRuntime, command: DroneCommand) -> None:
        try:
            if command.kind == "arm":
                result = runtime.drone.arm()
            elif command.kind == "takeoff":
                result = runtime.drone.takeoff(command.altitude)
            elif command.kind == "goto":
                wp     = command.waypoint
                result = runtime.drone.goto_waypoint(wp.lat, wp.lon, wp.alt, runtime.airspeed_m_s)
            elif command.kind == "goto_home":
                h      = command.home_target
                result = runtime.drone.goto_waypoint(h.lat, h.lon, command.altitude, runtime.airspeed_m_s)
            elif command.kind == "goto_side":
                tp     = command.target_point
                result = runtime.drone.goto_waypoint(tp.lat, tp.lon, tp.alt, runtime.airspeed_m_s)
            elif command.kind == "land":
                result = runtime.drone.land()
            elif command.kind == "disarm":
                result = runtime.drone.disarm()
            elif command.kind == "charge_wait":
                result = "SUCCESS"
            elif command.kind == "inspect_nadir":
                runtime.drone.point_gimbal(pitch_deg=-90, yaw_deg=0)
                result = "SUCCESS"
            elif command.kind == "capture_side":
                current_heading = runtime.drone.get_heading() or 0.0
                yaw_to_send = yaw_relative_to_heading(command.bearing_deg, current_heading)
                runtime.drone.point_gimbal(pitch_deg=command.pitch_deg, yaw_deg=yaw_to_send)
                result = "SUCCESS"
            else:
                raise RuntimeError("Unknown command kind: {0}".format(command.kind))
        except Exception as exc:
            self._mark_failed(runtime, "Command {0} raised {1}".format(command.label, exc))
            return

        if result != "SUCCESS":
            self._mark_failed(runtime, "Command {0} returned {1}".format(command.label, result))
            return

        command.issued           = True
        command.issued_at        = time.time()
        command.last_progress_at = command.issued_at
        command.issued_at_vt     = runtime.drone.get_boot_ms()
        command.last_progress_vt = command.issued_at_vt
        command.best_metric      = self._remaining_metric(runtime, command)
        runtime.phase            = command.label
        # Skip the per-command print for Stage 2 sub-steps -- the
        # [INSPECT] summary already covers what happened on each side,
        # so printing every goto_side/capture_side here is just noise.
        if command.kind not in ("goto_side", "capture_side"):
            print("[COMMAND] {0}: {1}".format(runtime.name, command.label))

    def _is_command_complete(self, runtime: DroneRuntime, command: DroneCommand) -> bool:
        try:
            if command.kind == "arm":
                return runtime.drone.is_armed()
            if command.kind == "takeoff":
                return runtime.drone.get_altitude() >= (command.altitude - command.tolerance_m)
            if command.kind in ("goto", "goto_home", "goto_side"):
                return runtime.drone.distance_to_target() <= command.tolerance_m
            if command.kind == "land":
                return not runtime.drone.is_armed() or runtime.drone.get_altitude() <= command.tolerance_m
            if command.kind == "disarm":
                return not runtime.drone.is_armed()
            if command.kind == "charge_wait":
                # Vehicle-time so a restore mid-charge resumes correctly
                # rather than counting the dead/checkpointed gap.
                now_vt = runtime.drone.get_boot_ms()
                if now_vt is None or command.issued_at_vt is None:
                    return False
                return (now_vt - command.issued_at_vt) >= self.charge_delay_s * 1000.0
            if command.kind in ("inspect_nadir", "capture_side"):
                return self._poll_capture_command(runtime, command)
        except Exception as exc:
            self._mark_failed(runtime, "Completion check failed for {0}: {1}".format(command.label, exc))
            return False
        return False

    def _poll_capture_command(self, runtime: DroneRuntime, command: DroneCommand) -> bool:
        """
        Drives an inspect_nadir / capture_side command through 3 phases:

          1. Settle  -- wait capture_settle_s after the gimbal command
                        was sent, so it has time to physically move
                        before the camera actually captures anything.
          2. Trigger -- send request_external_capture() exactly once,
                        right when the settle period elapses.
          3. Collect -- wait for the camera component's ACK, then
                        download the file via MAVFTP. A separate
                        ack-timeout applies here, since waiting for an
                        ack is a different kind of wait than settling.

        All timing is measured on the autopilot clock (time_boot_ms) so a
        checkpoint/restore landing inside any phase resumes correctly
        instead of counting the time the process spent dead.

        Returns True once the command is fully done (image downloaded,
        OR conclusively failed/timed out -- either way we move on
        rather than stalling the whole mission on one bad capture).
        """
        now_vt = runtime.drone.get_boot_ms()
        if now_vt is None or command.issued_at_vt is None:
            return False   # no vehicle-time clock yet -- wait for telemetry

        # Phase 1: settling
        if (now_vt - command.issued_at_vt) < self.capture_settle_s * 1000.0:
            return False

        # Phase 2: send the trigger exactly once, right as settling ends
        if command.request_id is None:
            command.request_id = self._next_request_id()
            runtime.drone.request_external_capture(command.request_id)
            command._trigger_sent_vt = now_vt
            return False

        # Phase 3: already resolved in an earlier tick?
        if hasattr(command, "_captured_image"):
            return True

        ack_result = runtime.drone.get_camera_ack(command.request_id)

        if ack_result is None:
            if (now_vt - command._trigger_sent_vt) > self.camera_ack_timeout_s * 1000.0:
                print("[CAPTURE] {0}: no camera ack within {1:.0f}s -- giving up.".format(
                    command.label, self.camera_ack_timeout_s))
                command._captured_image = None
                return True
            return False   # still waiting, within the ack timeout

        if ack_result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("[CAPTURE] {0}: camera component returned failure (result={1}).".format(
                command.label, ack_result))
            command._captured_image = None
            return True

        # Ack accepted -- download the file via MAVFTP
        wp_folder = "wp{0:02d}".format(command.waypoint.source_index)
        local_dir = os.path.join(self.capture_dir, wp_folder)
        os.makedirs(local_dir, exist_ok=True)
        label = "nadir" if command.kind == "inspect_nadir" else "side_{0}".format(command.direction)
        local_path  = os.path.join(local_dir, "{0}.jpg".format(label))
        remote_path = "logs/capture_{0}.jpg".format(command.request_id)

        try:
            runtime.ftp.get_file(remote_path, local_path, timeout=10.0)
            command._captured_image = local_path
        except FtpError as exc:
            print("[CAPTURE] {0}: FTP download failed: {1}".format(command.label, exc))
            command._captured_image = None

        return True

    def _remaining_metric(self, runtime: DroneRuntime, command: DroneCommand) -> Optional[float]:
        try:
            if command.kind == "takeoff":
                return abs(command.altitude - runtime.drone.get_altitude())
            if command.kind in ("goto", "goto_home", "goto_side"):
                return runtime.drone.distance_to_target()
            if command.kind == "land":
                return runtime.drone.get_altitude()
            if command.kind == "charge_wait":
                # Remaining charge seconds on the vehicle clock (display/
                # best_metric only -- charge_wait is skipped in _check_progress).
                now_vt = runtime.drone.get_boot_ms()
                if command.issued_at_vt is not None and now_vt is not None:
                    return max(0.0, self.charge_delay_s - (now_vt - command.issued_at_vt) / 1000.0)
            # Note: inspect_nadir / capture_side have no entry here --
            # _check_progress() skips them entirely (see below), since
            # their timing (settle + trigger + ack-wait) is fully
            # handled inside _poll_capture_command() instead.
        except Exception:
            return None
        return None

    def _check_progress(self, runtime: DroneRuntime, command: DroneCommand) -> None:
        if command.issued_at is None:
            return
        if command.kind in ("charge_wait", "inspect_nadir", "capture_side"):
            return

        # Measure elapsed time on the autopilot's own clock (time_boot_ms),
        # which travels inside the replayed telemetry and is rebuilt on
        # restore. Wall clock (time.time()) would also count the time the
        # process spent checkpointed/dead, which falsely trips the
        # no-progress timeout right after a slow restore.
        now_vt = runtime.drone.get_boot_ms()
        if now_vt is None:
            return   # no telemetry yet -- can't measure, so don't enforce

        # If the command was issued before any position fix was available,
        # adopt the first sample we see as the baseline (count from here).
        if command.issued_at_vt is None:
            command.issued_at_vt     = now_vt
            command.last_progress_vt = now_vt
            return

        timeout_ms = command.timeout_s * 1000.0

        if (now_vt - command.issued_at_vt) > timeout_ms and command.kind in ("arm", "disarm"):
            self._mark_failed(runtime, "Timeout waiting for {0}".format(command.label))
            return

        metric = self._remaining_metric(runtime, command)
        if metric is not None:
            if command.best_metric is None or metric < (command.best_metric - 0.5):
                command.best_metric      = metric
                command.last_progress_vt = now_vt
            last_vt = command.last_progress_vt if command.last_progress_vt is not None else command.issued_at_vt
            if (now_vt - last_vt) > timeout_ms:
                self._mark_failed(runtime, "No progress on {0}".format(command.label))

    def _check_health(self, runtime: DroneRuntime) -> None:
        try:
            # Heartbeat age on the vehicle clock -- restore-safe. Returns
            # None until both a heartbeat stamp and a current position
            # sample exist, in which case we don't enforce (avoids a
            # false timeout on the first tick right after a restore).
            age = runtime.drone.get_heartbeat_age_vt()
            if age is not None and age > self.heartbeat_timeout_s:
                self._mark_failed(runtime, "Heartbeat timeout ({0:.1f}s)".format(age))
                return
            cmd = runtime.active_command()
            if cmd is not None and cmd.kind not in ("land", "disarm", "charge_wait"):
                if not runtime.drone.is_armed() and runtime.command_index > 0:
                    self._mark_failed(runtime, "Drone disarmed unexpectedly.")
        except Exception as exc:
            self._mark_failed(runtime, "Health check failed: {0}".format(exc))

    def _mark_failed(self, runtime: DroneRuntime, reason: str) -> None:
        if runtime.failed:
            return
        runtime.failed         = True
        runtime.failure_reason = reason
        runtime.phase          = "FAILED"
        print("[FAIL] {0}: {1}".format(runtime.name, reason))

        if runtime.assigned_waypoint is not None:
            self.pending_waypoints.appendleft(runtime.assigned_waypoint)
            print("[REQUEUE] WP#{0} pushed back to queue.".format(
                runtime.assigned_waypoint.source_index))
        try:
            runtime.drone.land()
        except Exception:
            pass

    # -----------------------------------------------------------------------
    # Status / completion checks
    # -----------------------------------------------------------------------

    def _all_work_completed(self) -> bool:
        if self.pending_waypoints:
            return False
        for runtime in self.drones:
            if runtime.failed:
                continue
            if runtime.assigned_waypoint is not None or runtime.command_queue:
                return False
        return True

    def _mission_is_stuck(self) -> bool:
        if not self.pending_waypoints:
            return False
        return all(runtime.failed for runtime in self.drones)

    def _report_status_if_needed(self) -> None:
        # Wall-clock throttle is intentional here: this only gates how often
        # a human-facing status line prints; it never fails anything. After a
        # restore the worst case is one immediate status print, then normal.
        now = time.time()
        if now - self.last_status_at < self.status_interval_s:
            return
        self.last_status_at = now
        print("[STATUS] finished={0}/{1}, pending={2}".format(
            self.finished_waypoints, self.total_waypoints, len(self.pending_waypoints)))
        for runtime in self.drones:
            pos     = runtime.drone.get_position()
            armed   = "ARMED" if runtime.drone.is_armed() else "DISARMED"
            battery = runtime.drone.get_battery_remaining()
            line    = "  {0}: phase={1}, armed={2}, battery={3}%, pos=({4:.5f}, {5:.5f}, {6:.1f})".format(
                runtime.name, runtime.phase, armed, battery,
                pos.lat, pos.lon, pos.alt,
            )
            if runtime.assigned_waypoint is not None:
                line += ", wp=WP#{0}".format(runtime.assigned_waypoint.source_index)
            if runtime.failed and runtime.failure_reason:
                line += ", reason={0}".format(runtime.failure_reason)
            print(line)


# -----------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------

def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Drone mission controller with two-stage object inspection."
    )
    parser.add_argument("--connection",            required=True,             help="MAVLink connection string (e.g. udp:127.0.0.1:14553).")
    parser.add_argument("--mission",               required=True,             help="Path to QGC .waypoints file.")
    parser.add_argument("--discovery-timeout-s",   type=float, default=10.0, help="Seconds to listen for drones on startup.")
    parser.add_argument("--takeoff-alt-m",         type=float, default=None, help="Override takeoff altitude.")
    parser.add_argument("--airspeed-m-s",          type=float, default=5.0,  help="Cruise airspeed in m/s.")
    parser.add_argument("--poll-interval-s",       type=float, default=0.5,  help="Main loop polling interval (only used if the run()-loop sleep is re-enabled).")
    parser.add_argument("--waypoint-tolerance-m",  type=float, default=3.0,  help="Waypoint arrival radius.")
    parser.add_argument("--takeoff-tolerance-m",   type=float, default=1.0,  help="Takeoff altitude tolerance.")
    parser.add_argument("--landing-tolerance-m",   type=float, default=0.4,  help="Landing altitude tolerance.")
    parser.add_argument("--status-interval-s",     type=float, default=5.0,  help="Status print interval.")
    parser.add_argument("--heartbeat-timeout-s",   type=float, default=10.0, help="Heartbeat failure threshold.")
    parser.add_argument("--no-progress-timeout-s", type=float, default=45.0, help="No-progress failure threshold.")
    parser.add_argument("--battery-threshold",     type=int,   default=20,   help="Battery percentage below which drone lands to recharge.")
    parser.add_argument("--charge-delay-s",        type=float, default=60.0, help="Simulated recharge delay in seconds.")
    parser.add_argument("--capture-settle-s",      type=float, default=2.0,  help="Seconds to let the gimbal settle before each capture.")
    parser.add_argument("--capture-dir",           type=str,   default="captures", help="Local directory to save downloaded captures into (organized per-waypoint).")
    parser.add_argument("--confirm-alt-m",         type=float, default=5.0,  help="Altitude for Stage 2 oblique confirmation.")
    parser.add_argument("--confirm-offset-m",      type=float, default=5.0,  help="Horizontal offset from target during Stage 2.")
    parser.add_argument("--anomaly-threshold",     type=float, default=0.15, help="Stage 1: minimum confidence to count as 'something detected'.")
    parser.add_argument("--confirm-threshold",     type=float, default=0.6,  help="Stage 2: minimum confidence to count as confirmed.")
    parser.add_argument("--camera-ack-timeout-s",  type=float, default=20.0, help="Max seconds to wait for the camera component's ACK before giving up on a capture.")
    parser.add_argument("--model-path", type=str, default=None, help="Path to the YOLO .pt model. Defaults to the detector's built-in path if omitted.")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_argument_parser()
    args = parser.parse_args(_resolve_cli_tokens(argv))

    route_waypoints, mission_takeoff_alt, ignored = parse_mission_file(args.mission)
    takeoff_alt_m = args.takeoff_alt_m if args.takeoff_alt_m is not None else mission_takeoff_alt

    if ignored:
        print("[WARN] Ignoring unsupported mission commands: {0}".format(
            ", ".join(str(c) for c in ignored)))

    controller = MissionController(
        connection_string=args.connection,
        route_waypoints=route_waypoints,
        takeoff_alt_m=takeoff_alt_m,
        airspeed_m_s=args.airspeed_m_s,
        poll_interval_s=args.poll_interval_s,
        waypoint_tolerance_m=args.waypoint_tolerance_m,
        takeoff_tolerance_m=args.takeoff_tolerance_m,
        landing_tolerance_m=args.landing_tolerance_m,
        status_interval_s=args.status_interval_s,
        heartbeat_timeout_s=args.heartbeat_timeout_s,
        no_progress_timeout_s=args.no_progress_timeout_s,
        battery_threshold_pct=args.battery_threshold,
        charge_delay_s=args.charge_delay_s,
        discovery_timeout_s=args.discovery_timeout_s,
        capture_settle_s=args.capture_settle_s,
        capture_dir=args.capture_dir,
        confirm_alt_m=args.confirm_alt_m,
        confirm_offset_m=args.confirm_offset_m,
        anomaly_threshold=args.anomaly_threshold,
        confirm_threshold=args.confirm_threshold,
        camera_ack_timeout_s=args.camera_ack_timeout_s,
        model_path=args.model_path,
    )

    try:
        controller.connect_all()
        controller.run()
    finally:
        controller.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())