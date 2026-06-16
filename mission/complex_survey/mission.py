import argparse
import re
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, List, Optional, Sequence, Tuple

from pymavlink import mavutil

from drone import Drone, Waypoint, MAVLinkDispatcher


MISSION_CMD_NAV_WAYPOINT         = 16
MISSION_CMD_NAV_LOITER_UNLIM     = 17
MISSION_CMD_NAV_LOITER_TURNS     = 18
MISSION_CMD_NAV_RETURN_TO_LAUNCH = 20
MISSION_CMD_NAV_LAND             = 21
MISSION_CMD_NAV_TAKEOFF          = 22
MISSION_CMD_NAV_SPLINE_WAYPOINT  = 82


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
    altitude: Optional[float] = None
    timeout_s: float = 60.0
    tolerance_m: float = 2.0
    issued: bool = False
    issued_at: Optional[float] = None
    last_progress_at: Optional[float] = None
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


def haversine_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    from math import atan2, cos, radians, sin, sqrt
    R       = 6371000.0
    phi1    = radians(lat1)
    phi2    = radians(lat2)
    dphi    = radians(lat2 - lat1)
    dlambda = radians(lon2 - lon1)
    a = sin(dphi / 2.0) ** 2 + cos(phi1) * cos(phi2) * sin(dlambda / 2.0) ** 2
    return R * 2.0 * atan2(sqrt(a), sqrt(1.0 - a))


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
    ) -> None:
        self.connection_string     = connection_string
        self.dispatcher: Optional[MAVLinkDispatcher] = None
        self.pending_waypoints: Deque[MissionWaypoint] = deque(route_waypoints)
        self.takeoff_alt_m         = takeoff_alt_m
        self.airspeed_m_s          = airspeed_m_s
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
        self.drones: List[DroneRuntime] = []
        self.total_waypoints       = len(route_waypoints)
        self.finished_waypoints    = 0
        self.last_status_at        = 0.0

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
            )
            self.drones.append(runtime)
            print("[READY] {0} sysid={1} home=({2:.7f}, {3:.7f})".format(
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

            time.sleep(self.poll_interval_s)

    def close(self) -> None:
        if self.dispatcher is not None:
            try:
                self.dispatcher.close()
            except Exception:
                pass

    # -----------------------------------------------------------------------
    # Waypoint assignment — one at a time, only for idle (grounded) drones
    # -----------------------------------------------------------------------

    def _assign_waypoints_to_idle_drones(self) -> None:
        for runtime in self.drones:
            if runtime.failed or not runtime.is_idle():
                continue
            if not self.pending_waypoints:
                continue

            waypoint = self.pending_waypoints.popleft()
            runtime.assigned_waypoint = waypoint
            runtime.command_queue     = self._build_command_queue(runtime, waypoint)
            runtime.command_index     = 0
            runtime.phase             = "ASSIGNED"
            print("[ASSIGN] {0} <- WP#{1} ({2:.7f}, {3:.7f})".format(
                runtime.name, waypoint.source_index, waypoint.lat, waypoint.lon))

    def _build_command_queue(self, runtime: DroneRuntime, waypoint: MissionWaypoint) -> List[DroneCommand]:
        """
        Builds ARM → TAKEOFF → GOTO → RETURN HOME.
        LAND / DISARM are NOT added here — they are inserted dynamically
        by _decide_after_return_home() based on battery level.
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
        """
        Command queue when drone is already in the air and battery is ok.
        No ARM or TAKEOFF needed.
        """
        return [
            DroneCommand(
                kind="goto",
                label="GOTO WP#{0}".format(waypoint.source_index),
                waypoint=waypoint,
                timeout_s=self.no_progress_timeout_s,
                tolerance_m=self.waypoint_tolerance_m,
            ),
            DroneCommand(
                kind="goto_home",
                label="RETURN HOME",
                home_target=runtime.home,
                altitude=runtime.takeoff_alt_m,
                timeout_s=self.no_progress_timeout_s,
                tolerance_m=self.waypoint_tolerance_m,
            ),
        ]

    def _decide_after_return_home(self, runtime: DroneRuntime) -> None:
        """
        Called when RETURN HOME completes. Drone is hovering above home.
        Decides what to do next based on battery level and pending waypoints.
        """
        battery = runtime.drone.get_battery_remaining()
        insert_pos = runtime.command_index + 1

        if battery < self.battery_threshold_pct:
            # Battery low: land, disarm, simulate charge, become idle
            print("[BATTERY] {0} battery low ({1}%). Landing to recharge.".format(
                runtime.name, battery))
            runtime.command_queue[insert_pos:] = [
                DroneCommand(kind="land",        label="LAND",        timeout_s=120.0, tolerance_m=self.landing_tolerance_m),
                DroneCommand(kind="disarm",       label="DISARM",      timeout_s=30.0,  tolerance_m=0.0),
                DroneCommand(kind="charge_wait",  label="CHARGING",    timeout_s=self.charge_delay_s + 10.0),
            ]
            # Clear assigned waypoint so after charge the drone gets fresh assignment
            runtime.assigned_waypoint = None

        elif self.pending_waypoints:
            # Battery ok and more work to do: go directly to next waypoint
            waypoint = self.pending_waypoints.popleft()
            runtime.assigned_waypoint = waypoint
            print("[ASSIGN] {0} <- WP#{1} (in air, battery {2}%)".format(
                runtime.name, waypoint.source_index, battery))
            runtime.command_queue[insert_pos:] = self._build_next_waypoint_commands(runtime, waypoint)

        else:
            # Battery ok but no more waypoints: land and disarm cleanly
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
            # Command queue exhausted — drone is on the ground and idle
            runtime.reset()
            return

        if not command.issued:
            self._issue_command(runtime, command)
            return

        if self._is_command_complete(runtime, command):

            # Count waypoint as finished when GOTO completes
            if command.kind == "goto" and runtime.assigned_waypoint is not None:
                self.finished_waypoints += 1
                print("[COMPLETE] {0} visited WP#{1} ({2}/{3})".format(
                    runtime.name,
                    runtime.assigned_waypoint.source_index,
                    self.finished_waypoints,
                    self.total_waypoints,
                ))
                runtime.completed_waypoints += 1

            # When RETURN HOME completes, decide what comes next
            if command.kind == "goto_home":
                self._decide_after_return_home(runtime)

            runtime.command_index += 1
            next_cmd      = runtime.active_command()
            runtime.phase = next_cmd.label if next_cmd is not None else "READY"
            return

        self._check_progress(runtime, command)

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
            elif command.kind == "land":
                result = runtime.drone.land()
            elif command.kind == "disarm":
                result = runtime.drone.disarm()
            elif command.kind == "charge_wait":
                # No MAVLink command — just start the timer
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
        command.best_metric      = self._remaining_metric(runtime, command)
        runtime.phase            = command.label
        print("[COMMAND] {0}: {1}".format(runtime.name, command.label))

    def _is_command_complete(self, runtime: DroneRuntime, command: DroneCommand) -> bool:
        try:
            if command.kind == "arm":
                return runtime.drone.is_armed()
            if command.kind == "takeoff":
                return runtime.drone.get_altitude() >= (command.altitude - command.tolerance_m)
            if command.kind in ("goto", "goto_home"):
                return runtime.drone.distance_to_target() <= command.tolerance_m
            if command.kind == "land":
                return not runtime.drone.is_armed() or runtime.drone.get_altitude() <= command.tolerance_m
            if command.kind == "disarm":
                return not runtime.drone.is_armed()
            if command.kind == "charge_wait":
                # Complete when charge_delay_s has elapsed since issued
                return (time.time() - command.issued_at) >= self.charge_delay_s
        except Exception as exc:
            self._mark_failed(runtime, "Completion check failed for {0}: {1}".format(command.label, exc))
            return False
        return False

    def _remaining_metric(self, runtime: DroneRuntime, command: DroneCommand) -> Optional[float]:
        try:
            if command.kind == "takeoff":
                return abs(command.altitude - runtime.drone.get_altitude())
            if command.kind in ("goto", "goto_home"):
                return runtime.drone.distance_to_target()
            if command.kind == "land":
                return runtime.drone.get_altitude()
            if command.kind == "charge_wait":
                if command.issued_at is not None:
                    return max(0.0, self.charge_delay_s - (time.time() - command.issued_at))
        except Exception:
            return None
        return None

    def _check_progress(self, runtime: DroneRuntime, command: DroneCommand) -> None:
        if command.issued_at is None:
            return
        # charge_wait uses its own timer — no watchdog needed
        if command.kind == "charge_wait":
            return
        now = time.time()
        if now - command.issued_at > command.timeout_s and command.kind in ("arm", "disarm"):
            self._mark_failed(runtime, "Timeout waiting for {0}".format(command.label))
            return
        metric = self._remaining_metric(runtime, command)
        if metric is not None:
            if command.best_metric is None or metric < (command.best_metric - 0.5):
                command.best_metric      = metric
                command.last_progress_at = now
            last = command.last_progress_at or command.issued_at
            if now - last > command.timeout_s:
                self._mark_failed(runtime, "No progress on {0}".format(command.label))

    def _check_health(self, runtime: DroneRuntime) -> None:
        try:
            age = runtime.drone.get_heartbeat_age()
            if age > self.heartbeat_timeout_s:
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
            line    = "  {0}: phase={1}, armed={2}, battery={3}%, pos=({4:.7f}, {5:.7f}, {6:.1f})".format(
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
        description="Multi-drone mission controller — one waypoint per trip."
    )
    parser.add_argument("--connection",            required=True,             help="MAVLink connection string (e.g. udp:127.0.0.1:14553).")
    parser.add_argument("--mission",               required=True,             help="Path to QGC .waypoints file.")
    parser.add_argument("--discovery-timeout-s",   type=float, default=10.0, help="Seconds to listen for drones on startup.")
    parser.add_argument("--takeoff-alt-m",         type=float, default=None, help="Override takeoff altitude.")
    parser.add_argument("--airspeed-m-s",          type=float, default=5.0,  help="Cruise airspeed in m/s.")
    parser.add_argument("--poll-interval-s",       type=float, default=0.5,  help="Main loop polling interval.")
    parser.add_argument("--waypoint-tolerance-m",  type=float, default=3.0,  help="Waypoint arrival radius.")
    parser.add_argument("--takeoff-tolerance-m",   type=float, default=1.0,  help="Takeoff altitude tolerance.")
    parser.add_argument("--landing-tolerance-m",   type=float, default=0.4,  help="Landing altitude tolerance.")
    parser.add_argument("--status-interval-s",     type=float, default=5.0,  help="Status print interval.")
    parser.add_argument("--heartbeat-timeout-s",   type=float, default=10.0, help="Heartbeat failure threshold.")
    parser.add_argument("--no-progress-timeout-s", type=float, default=45.0, help="No-progress failure threshold.")
    parser.add_argument("--battery-threshold",     type=int,   default=20,   help="Battery percentage below which drone lands to recharge.")
    parser.add_argument("--charge-delay-s",        type=float, default=60.0, help="Simulated recharge delay in seconds.")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_argument_parser()
    args   = parser.parse_args(argv)

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
    )

    try:
        controller.connect_all()
        controller.run()
    finally:
        controller.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())