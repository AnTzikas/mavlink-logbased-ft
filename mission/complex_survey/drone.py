from pymavlink import mavutil
import time
import math

# ArduCopter custom mode IDs
COPTER_MODE_GUIDED = 4
COPTER_MODE_RTL    = 6
COPTER_MODE_LAND   = 9


class Waypoint:
    def __init__(self, lat: float, lon: float, alt: float):
        self.lat = lat
        self.lon = lon
        self.alt = alt


class ParameterProxy:
    """Dict-like object that sends PARAM_SET to the drone on write."""

    def __init__(self, mav, sysid: int, compid: int = 1):
        self._mav    = mav
        self._sysid  = sysid
        self._compid = compid

    def __setitem__(self, name: str, value: float) -> None:
        self._mav.param_set_send(
            self._sysid,
            self._compid,
            name.encode("utf-8"),
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )


class MAVLinkDispatcher:
    """Owns the single MAVLink connection, discovers drones and routes
    incoming messages to the correct Drone instance by sysid."""

    def __init__(self, connection_string: str, discovery_timeout_s: float = 10.0):
        self._conn               = mavutil.mavlink_connection(connection_string)
        self._drones: dict[int, "Drone"] = {}
        self.discovery_timeout_s = discovery_timeout_s

    def discover(self) -> list[int]:
        print("[DISCOVER] Listening for heartbeats ({0}s)...".format(self.discovery_timeout_s))
        seen: dict[int, float] = {}
        deadline = time.time() + self.discovery_timeout_s

        while time.time() < deadline:
            msg = self._conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
            if msg is None:
                continue
            sysid = msg.get_srcSystem()
            if sysid in (0, 255):   # ignore broadcast and GCS heartbeats
                continue
            if sysid not in seen:
                seen[sysid] = time.time()
                print("[DISCOVER] Found drone sysid={0}".format(sysid))

        found = sorted(seen.keys())
        print("[DISCOVER] Done. Found {0} drone(s): {1}".format(len(found), found))
        return found

    def register(self, drone: "Drone") -> None:
        self._drones[drone._sysid] = drone

    def update(self, max_messages: int = 100) -> None:
        """Drain pending MAVLink messages and route each to the correct drone."""
        for _ in range(max_messages):
            msg = self._conn.recv_match(blocking=False)
            if msg is None:
                break
            sysid = msg.get_srcSystem()
            if sysid in self._drones:
                self._drones[sysid]._process_message(msg)

        # Age the heartbeat timer for every registered drone
        for drone in self._drones.values():
            if drone._last_hb_at is not None:
                drone._heartbeat_age = time.time() - drone._last_hb_at

    def send_command_long(
        self, sysid: int, command,
        p1=0.0, p2=0.0, p3=0.0, p4=0.0,
        p5=0.0, p6=0.0, p7=0.0,
    ) -> None:
        self._conn.mav.command_long_send(
            sysid, 1, command, 0,
            p1, p2, p3, p4, p5, p6, p7,
        )

    def send_set_mode(self, sysid: int, mode_id: int) -> None:
        self._conn.mav.set_mode_send(
            sysid,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )

    def send_set_position_target(
        self, sysid: int,
        lat: float, lon: float, alt: float,
    ) -> None:
        self._conn.mav.set_position_target_global_int_send(
            0,
            sysid, 1,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000_1111_1111_1000,
            int(lat * 1e7),
            int(lon * 1e7),
            alt,
            0, 0, 0,
            0, 0, 0,
            0, 0,
        )

    def close(self) -> None:
        self._conn.close()


class Drone:
    """Represents a single drone. Provides high-level flight commands and
    maintains cached state updated by MAVLinkDispatcher.update()."""

    def __init__(self, dispatcher: MAVLinkDispatcher, sysid: int, compid: int = 1):
        self._dispatcher = dispatcher
        self._sysid      = sysid
        self._compid     = compid
        self.parameters  = ParameterProxy(dispatcher._conn.mav, sysid, compid)

        # --- cached telemetry state (updated by _process_message) ---
        self._lat          = 0.0
        self._lon          = 0.0
        self._alt_rel      = 0.0
        self._armed        = False
        self._custom_mode  = -1
        self._airspeed     = 0.0
        self._groundspeed  = 0.0
        self._last_hb_at: float | None = None
        self._heartbeat_age = 0.0
        self._home: Waypoint | None = None
        self._battery_remaining: int = 100   # percent, updated by BATTERY_STATUS

        # --- flight state ---
        self._target_point: Waypoint | None = None
        self._target_alt   = 0.0
        self._odometer_start: Waypoint | None = None
        self._odometer     = 0.0

        self.status        = "IDLE"
        self.rtl_altitude  = 10.0
        self.airspeed      = 3.0
        self.parameters["WPNAV_SPEED"] = 300

    # -----------------------------------------------------------------------
    # MAVLink message processing
    # -----------------------------------------------------------------------

    def _process_message(self, msg) -> None:
        t = msg.get_type()

        if t == "GLOBAL_POSITION_INT":
            self._lat     = msg.lat / 1e7
            self._lon     = msg.lon / 1e7
            self._alt_rel = msg.relative_alt / 1000.0

        elif t == "HEARTBEAT":
            self._armed      = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self._custom_mode = msg.custom_mode
            self._last_hb_at  = time.time()
            self._heartbeat_age = 0.0

        elif t == "VFR_HUD":
            self._airspeed    = msg.airspeed
            self._groundspeed = msg.groundspeed

        elif t == "HOME_POSITION":
            self._home = Waypoint(
                msg.latitude  / 1e7,
                msg.longitude / 1e7,
                msg.altitude  / 1000.0,
            )

        elif t == "BATTERY_STATUS":
            # battery_remaining is -1 if not available, otherwise 0-100
            if msg.battery_remaining >= 0:
                self._battery_remaining = msg.battery_remaining

    def _set_mode(self, mode_id: int, timeout: float = 3.0) -> str:
        deadline = time.time() + timeout
        self._dispatcher.send_set_mode(self._sysid, mode_id)
        while time.time() < deadline:
            msg = self._dispatcher._conn.recv_match(blocking=True, timeout=0.5)
            if msg is not None:
                sysid = msg.get_srcSystem()
                if sysid in self._dispatcher._drones:
                    self._dispatcher._drones[sysid]._process_message(msg)
            if self._custom_mode == mode_id:
                return "SUCCESS"
            self._dispatcher.send_set_mode(self._sysid, mode_id)
        return "ERR_MODE_CHANGE"

    # -----------------------------------------------------------------------
    # Telemetry getters
    # -----------------------------------------------------------------------

    def get_airspeed(self) -> float:
        return self._airspeed

    def get_groundspeed(self) -> float:
        return self._groundspeed

    def get_altitude(self) -> float:
        return self._alt_rel

    def get_position(self) -> Waypoint:
        return Waypoint(self._lat, self._lon, self._alt_rel)

    def get_home(self) -> Waypoint:
        if self._home is not None:
            return Waypoint(self._home.lat, self._home.lon, 0)
        return Waypoint(self._lat, self._lon, 0)

    def is_armed(self) -> bool:
        return self._armed

    def get_heartbeat_age(self) -> float:
        return self._heartbeat_age

    def get_battery_remaining(self) -> int:
        """Returns battery percentage 0-100."""
        return self._battery_remaining

    # -----------------------------------------------------------------------
    # Distance helpers
    # -----------------------------------------------------------------------

    def haversine_distance(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        R       = 6371000.0
        phi1    = math.radians(lat1)
        phi2    = math.radians(lat2)
        dphi    = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = (math.sin(dphi / 2) ** 2
             + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2)
        return R * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))

    def distance_to_target(self) -> float:
        if self._target_point is None:
            return float("inf")
        return self.haversine_distance(
            self._lat, self._lon,
            self._target_point.lat, self._target_point.lon,
        )

    def distance_to_target_alt(self) -> float:
        return abs(self._target_alt - self._alt_rel)

    # -----------------------------------------------------------------------
    # Odometer
    # -----------------------------------------------------------------------

    def reset_odometer(self) -> None:
        self._odometer       = 0.0
        self._odometer_start = Waypoint(self._lat, self._lon, self._alt_rel)

    def get_odometer(self) -> float:
        if self._odometer_start is None:
            return 0.0
        dlat  = self._odometer_start.lat - self._lat
        dlon  = self._odometer_start.lon - self._lon
        return math.sqrt(dlat * dlat + dlon * dlon) * 1.113195e5

    # -----------------------------------------------------------------------
    # Flight commands
    # -----------------------------------------------------------------------

    def arm(self) -> str:
        status = self._set_mode(COPTER_MODE_GUIDED)
        if status == "ERR_MODE_CHANGE":
            return "ERR_ARM"
        self._dispatcher.send_command_long(
            self._sysid,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            p1=1,
        )
        self.status = "ARMED"
        return "SUCCESS"

    def disarm(self) -> str:
        self._dispatcher.send_command_long(
            self._sysid,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            p1=0,
        )
        return "SUCCESS"

    def kill_switch(self) -> str:
        try:
            self._dispatcher.send_command_long(
                self._sysid,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                p1=0,
                p2=21196,
            )
        except Exception:
            return "ERR_KILL"
        return "SUCCESS"

    def takeoff(self, altitude: float) -> str:
        self._target_point = Waypoint(self._lat, self._lon, altitude)
        self._target_alt   = altitude
        self._dispatcher.send_command_long(
            self._sysid,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            p7=altitude,
        )
        return "SUCCESS"

    def land(self) -> str:
        self._target_point = Waypoint(self._lat, self._lon, 0)
        self._target_alt   = 0
        status = self._set_mode(COPTER_MODE_LAND)
        if status == "ERR_MODE_CHANGE":
            return "ERR_LAND"
        self.status = "LANDING"
        return "SUCCESS"

    def return_to_launch(self) -> str:
        self.parameters["RTL_ALT"] = self.rtl_altitude
        status = self._set_mode(COPTER_MODE_RTL)
        if status == "ERR_MODE_CHANGE":
            return "ERR_RTL"
        self.status = "RTL"
        return "SUCCESS"

    def goto_waypoint(self, lat: float, lon: float, alt: float, airspeed: float = 1.0) -> str:
        if self.status in ("RTL", "FS_RTL"):
            return "ERR_ABORTING"
        self._target_point = Waypoint(lat, lon, alt)
        self._target_alt   = alt
        self.airspeed      = airspeed
        self.parameters["WPNAV_SPEED"] = airspeed * 100
        self._dispatcher.send_set_position_target(self._sysid, lat, lon, alt)
        return "SUCCESS"

    def goto_home(self, airspeed: float = 1.0) -> str:
        if self.status in ("RTL", "FS_RTL"):
            return "ERR_ABORTING"
        if self._home is None:
            return "ERR_NO_HOME"
        self._target_point = Waypoint(self._home.lat, self._home.lon, self._alt_rel)
        self._target_alt   = self._alt_rel
        self.airspeed      = airspeed
        self._dispatcher.send_set_position_target(
            self._sysid, self._home.lat, self._home.lon, self._alt_rel
        )
        self.status = "CRUISE"
        return "SUCCESS"

    def set_home(self, lat: float, lon: float) -> str:
        try:
            alt = self._home.alt if self._home is not None else 0.0
            if self._target_point is not None:
                old_alt = self._home.alt if self._home is not None else 0.0
                self._target_point = Waypoint(
                    self._target_point.lat,
                    self._target_point.lon,
                    self._target_point.alt + (old_alt - alt),
                )
            self._dispatcher.send_command_long(
                self._sysid,
                mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                p5=lat, p6=lon, p7=alt,
            )
        except Exception:
            return "ERR_HOMEPOINT"
        return "SUCCESS"