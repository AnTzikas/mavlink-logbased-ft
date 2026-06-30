from pymavlink import mavutil
import time
import math
from typing import Optional
import sys, os
from pathlib import Path

# Include Wrapper or Default Pymavlink
current_dir = Path(__file__).resolve().parent
sys.path.append(str(current_dir.parent.parent / "src"))

USE_WRAPPER = os.environ.get("USE_WRAPPER", "0") == "1"
if USE_WRAPPER:
    from wrapper import mavlink_connection as mav_connect
    logger_name = "FT_Mission"
else:
    from pymavlink.mavutil import mavlink_connection as mav_connect
    logger_name = "Std_Mission"

# ArduCopter custom mode IDs
COPTER_MODE_GUIDED = 4
COPTER_MODE_RTL    = 6
COPTER_MODE_LAND   = 9

# Camera component identity -- see camera_component.py
MAV_COMP_ID_CAMERA  = mavutil.mavlink.MAV_COMP_ID_CAMERA   # 100
CAPTURE_COMMAND     = mavutil.mavlink.MAV_CMD_USER_1        # 31010


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
    incoming messages to the correct Drone instance by sysid.

    The connection is created via mav_connect (see top of file): when
    USE_WRAPPER=1 this is the record/replay wrapper, otherwise it is raw
    pymavlink. Either way the object quacks like a pymavlink connection,
    so every self._conn.<method> call (recv_match, mav.*_send, ...) is
    transparently routed through whichever backend is active."""

    def __init__(self, connection_string: str, discovery_timeout_s: float = 10.0,
                 expected_drones: int | None = None):
        self._conn               = mav_connect(connection_string)
        self._drones: dict[int, "Drone"] = {}
        self.discovery_timeout_s = discovery_timeout_s
        # Number of drones to expect. When set, discovery stops as soon as
        # this many distinct drones have been seen -- a CONTENT-based exit
        # condition (depends only on which messages arrived), so it replays
        # identically. A wall-clock window would make a different number of
        # recv_match calls at replay (replay runs faster than real time),
        # desyncing the recorded receive stream from the caller's calls.
        self.expected_drones     = expected_drones

    def discover(self) -> list[int]:
        """Listen for HEARTBEATs and return the distinct drone sysids found.

        REPLAY-DETERMINISM: the loop must issue the SAME sequence of
        recv_match calls at record and at replay, or the wrapper's recorded
        receive stream desyncs from the caller. A wall-clock deadline does NOT
        satisfy this -- replay runs much faster than real time, so a
        `while time.time() < deadline` loop makes a different number of calls.

        Instead the exit condition is CONTENT-based:
          - if expected_drones is set, stop as soon as that many distinct
            drones have been seen (this depends only on which messages
            arrived, which is identical at record and replay);
          - a fixed MAX iteration cap bounds the loop so it can never spin
            forever if a drone never appears (also a fixed count -> the call
            sequence stays identical across record/replay even in the
            not-all-found case).

        The wall-clock timeout is kept ONLY as a coarse outer safety bound for
        the not-found case and is derived into a fixed number of recv attempts,
        not checked against time.time() inside the loop.
        """
        if self.expected_drones is not None:
            print("[DISCOVER] Listening for heartbeats (expecting {0} drone(s))...".format(
                self.expected_drones))
        else:
            print("[DISCOVER] Listening for heartbeats...")

        seen: dict[int, int] = {}
        # Fixed cap on recv_match calls: derived from the timeout and the 1s
        # per-call blocking timeout, so it's a constant number, not a clock
        # check. This is what makes the call sequence deterministic.
        max_attempts = max(1, int(self.discovery_timeout_s))

        for _attempt in range(max_attempts):
            msg = self._conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
            if msg is not None:
                sysid = msg.get_srcSystem()
                if sysid not in (0, 255) and sysid not in seen:   # skip broadcast/GCS
                    seen[sysid] = _attempt
                    print("[DISCOVER] Found drone sysid={0}".format(sysid))
            # Content-based early exit: we have everyone we expected.
            if self.expected_drones is not None and len(seen) >= self.expected_drones:
                break

        found = sorted(seen.keys())
        print("[DISCOVER] Done. Found {0} drone(s): {1}".format(len(found), found))
        return found

    def register(self, drone: "Drone") -> None:
        self._drones[drone._sysid] = drone

    def update(self, max_messages: int = 100, timeout: float = 0.5) -> None:
        """Drain pending MAVLink messages and route each to the correct drone.

        Blocks up to `timeout` for the FIRST message -- this replaces the
        mission loop's sleep(poll_interval): it wakes immediately when data
        arrives, or after `timeout` if the link is quiet, instead of busy-
        spinning. Once the first message is in, the rest of the socket buffer
        is drained non-blocking."""
        msg = self._conn.recv_match(blocking=True, timeout=timeout)
        drained = 0
        while msg is not None and drained < max_messages:
            sysid = msg.get_srcSystem()
            if sysid in self._drones:
                self._drones[sysid]._process_message(msg)
            drained += 1
            msg = self._conn.recv_match(blocking=False)   # drain the rest instantly

        # Age the wall-clock heartbeat timer for every registered drone.
        # (This is the wall-clock liveness signal, separate from the vehicle-
        # time heartbeat age used for restore-safe failure detection.)
        for drone in self._drones.values():
            if drone._last_hb_at is not None:
                drone._heartbeat_age = time.time() - drone._last_hb_at

    def send_command_long(
        self, sysid: int, command,
        p1=0.0, p2=0.0, p3=0.0, p4=0.0,
        p5=0.0, p6=0.0, p7=0.0,
        target_compid: int = 1,
    ) -> None:
        self._conn.mav.command_long_send(
            sysid, target_compid, command, 0,
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
        self._last_hb_at: float | None = None     # wall-clock stamp at last heartbeat
        self._heartbeat_age = 0.0                 # wall-clock heartbeat age (liveness only)
        self._home: Waypoint | None = None
        self._battery_remaining: int = 100        # percent, updated by BATTERY_STATUS
        self._heading_deg: float | None = None    # compass heading, from GLOBAL_POSITION_INT.hdg
        self._boot_ms: int | None = None          # autopilot uptime (GLOBAL_POSITION_INT.time_boot_ms) -- restore-safe clock
        self._last_hb_vt: int | None = None       # vehicle clock (time_boot_ms) at last heartbeat
        # Gimbal attitude feedback (from GIMBAL_DEVICE_ATTITUDE_STATUS).
        # _gimbal_pitch_deg is the ACTUAL pitch reported by the gimbal;
        # _cmd_gimbal_pitch_deg is the last pitch we COMMANDED via point_gimbal.
        # Comparing the two lets the mission wait for the gimbal to actually
        # arrive before capturing, instead of guessing with a fixed timer.
        self._gimbal_pitch_deg: float | None = None
        self._gimbal_yaw_deg: float | None = None
        self._cmd_gimbal_pitch_deg: float | None = None
        self._cmd_gimbal_yaw_deg: float | None = None
        # Ground velocity (m/s) from GLOBAL_POSITION_INT vx/vy/vz. Used by the
        # mission to require the drone has actually STOPPED (not merely arrived
        # within tolerance) before the next command -- so captures happen from
        # a settled, level platform instead of mid-deceleration.
        self._vx = 0.0
        self._vy = 0.0
        self._vz = 0.0

        # Camera ACK tracking. COMMAND_ACK carries no request_id of its
        # own (it only has 'command' and 'result'), so we can't match
        # an incoming ack to a specific request from the message alone.
        # Since this architecture only ever has ONE outstanding capture
        # request in flight at a time per drone, we just track the most
        # recently sent request_id and the most recent ack result, and
        # trust they correspond to each other.
        self._pending_capture_request_id: int | None = None
        self._camera_ack_result: int | None = None

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
            self._boot_ms = msg.time_boot_ms
            # velocity is reported in cm/s -> convert to m/s
            self._vx = msg.vx / 100.0
            self._vy = msg.vy / 100.0
            self._vz = msg.vz / 100.0
            # hdg is compass heading in centidegrees, 0-35999, or
            # 65535 if unknown/unavailable
            if msg.hdg != 65535:
                self._heading_deg = msg.hdg / 100.0

        elif t == "HEARTBEAT":
            # IMPORTANT: camera_component.py shares this drone's sysid,
            # so its heartbeat also arrives here. Only the autopilot's
            # own heartbeat (compid == self._compid) should update
            # armed/mode state -- otherwise the camera's heartbeat
            # (which always reports base_mode=0) would incorrectly
            # make the drone look disarmed.
            if msg.get_srcComponent() == self._compid:
                self._armed       = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self._custom_mode = msg.custom_mode
                self._last_hb_at  = time.time()
                self._last_hb_vt  = self._boot_ms       # snapshot the vehicle clock at this heartbeat
                self._heartbeat_age = 0.0

        elif t == "COMMAND_ACK":
            # The camera component's response to request_external_capture().
            # See class docstring note above on why we track this by
            # "most recent" rather than matching an id in the message.
            if msg.get_srcComponent() == MAV_COMP_ID_CAMERA and msg.command == CAPTURE_COMMAND:
                self._camera_ack_result = msg.result

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

        elif t == "GIMBAL_DEVICE_ATTITUDE_STATUS":
            # Actual gimbal orientation, as a quaternion q=[w,x,y,z]. Convert to
            # pitch/yaw so the mission can tell when the gimbal has reached the
            # commanded angle. angular_velocity_* are often nan in SITL, so we
            # rely on ANGLE arrival (current vs commanded), not velocity.
            q = msg.q
            pitch, yaw = self._quat_to_pitch_yaw(q)
            if pitch is not None:
                self._gimbal_pitch_deg = pitch
                self._gimbal_yaw_deg   = yaw

    def get_speed(self) -> float:
        """Current ground speed magnitude in m/s (sqrt(vx^2+vy^2+vz^2)), from
        GLOBAL_POSITION_INT. The mission uses this to confirm the drone has
        actually stopped at a target before proceeding -- arriving within the
        tolerance radius is not the same as being stationary and level."""
        return math.sqrt(self._vx * self._vx + self._vy * self._vy + self._vz * self._vz)

    @staticmethod
    def _quat_to_pitch_yaw(q):
        """Convert a MAVLink attitude quaternion q=[w,x,y,z] to (pitch_deg,
        yaw_deg). Returns (None, None) on a degenerate/invalid quaternion.
        Pitch is the gimbal's tilt: 0 = horizontal/forward, -90 = straight
        down -- matching the convention point_gimbal() uses."""
        try:
            w, x, y, z = q[0], q[1], q[2], q[3]
            # normalize (guard against drift / zero quaternion)
            n = math.sqrt(w*w + x*x + y*y + z*z)
            if n == 0.0:
                return None, None
            w, x, y, z = w/n, x/n, y/n, z/n
            # pitch (about Y): asin(2*(w*y - z*x)), clamped
            sinp = 2.0 * (w*y - z*x)
            sinp = max(-1.0, min(1.0, sinp))
            pitch = math.degrees(math.asin(sinp))
            # yaw (about Z)
            siny = 2.0 * (w*z + x*y)
            cosy = 1.0 - 2.0 * (y*y + z*z)
            yaw = math.degrees(math.atan2(siny, cosy))
            return pitch, yaw
        except Exception:
            return None, None

    def get_gimbal_pitch(self) -> float | None:
        """Actual gimbal pitch in degrees (0 = horizontal, -90 = down), or
        None if no GIMBAL_DEVICE_ATTITUDE_STATUS has been received yet."""
        return self._gimbal_pitch_deg

    def get_gimbal_yaw(self) -> float | None:
        """Actual gimbal yaw in degrees, or None if not yet known."""
        return self._gimbal_yaw_deg

    def get_commanded_gimbal_pitch(self) -> float | None:
        """The last pitch commanded via point_gimbal(), or None if never set."""
        return self._cmd_gimbal_pitch_deg

    def gimbal_pitch_error(self) -> float | None:
        """Absolute difference (deg) between commanded and actual gimbal pitch,
        or None if either is unknown. The mission uses this to decide the
        gimbal has 'arrived' before capturing."""
        if self._cmd_gimbal_pitch_deg is None or self._gimbal_pitch_deg is None:
            return None
        return abs(self._cmd_gimbal_pitch_deg - self._gimbal_pitch_deg)

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

    def get_heading(self) -> float | None:
        """Compass heading in degrees (0-360), or None if not yet known."""
        return self._heading_deg

    def get_boot_ms(self) -> int | None:
        """Autopilot uptime in ms (GLOBAL_POSITION_INT.time_boot_ms), or None
        if no position message has arrived yet. This clock lives inside the
        replayed telemetry, so it is reconstructed on restore -- unlike
        time.time(), it does not advance while the process is checkpointed/dead."""
        return self._boot_ms

    def get_heartbeat_age_vt(self) -> Optional[float]:
        """Heartbeat age on the autopilot clock (restore-safe). None until we
        have both a heartbeat stamp and a current vehicle-time sample.

        Used for failure detection: it does NOT count time the process spent
        checkpointed/dead, so it won't false-fail right after a slow restore."""
        if self._last_hb_vt is None or self._boot_ms is None:
            return None
        return (self._boot_ms - self._last_hb_vt) / 1000.0

    def get_position(self) -> Waypoint:
        return Waypoint(self._lat, self._lon, self._alt_rel)

    def get_home(self) -> Waypoint:
        if self._home is not None:
            return Waypoint(self._home.lat, self._home.lon, 0)
        return Waypoint(self._lat, self._lon, 0)

    def is_armed(self) -> bool:
        return self._armed

    def get_heartbeat_age(self) -> float:
        """Wall-clock heartbeat age in seconds. This is the LIVENESS signal:
        unlike the vehicle-time version, it keeps advancing even when telemetry
        stops, so it can detect a genuinely dead link. NOT used for the current
        restore-safe failure path (that uses get_heartbeat_age_vt); kept as a
        complementary signal for status/debugging."""
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
        # NOTE: not used by the current mission -- the mission returns home by
        # calling goto_waypoint() with the home coordinates (its "goto_home"
        # command kind), not this method. Kept as library surface.
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

    def capture_image(self, timeout: float = 0.5) -> str:
        """
        NOTE: not used by the current mission -- captures go through the
        external camera component via request_external_capture(). Kept as
        library surface for the autopilot-native MAV_CMD_IMAGE_START_CAPTURE
        path.

        Send MAV_CMD_IMAGE_START_CAPTURE and wait for COMMAND_ACK.
        Returns SUCCESS or ERR_CAPTURE.

        Timeout is kept short (0.5s) because in SITL ArduPilot has no
        camera configured and will never ACK this command -- waiting
        longer would stall the main loop and block every other drone
        from being polled during that time. On real hardware with a
        camera component, the ACK would normally arrive almost
        immediately anyway, well within this window.
        """
        self._dispatcher.send_command_long(
            self._sysid,
            mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE,
            p1=0,   # camera instance (0 = all cameras)
            p2=0,   # interval (0 = single capture)
            p3=1,   # total images to capture
        )

        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self._dispatcher._conn.recv_match(blocking=True, timeout=0.2)
            if msg is not None:
                sysid = msg.get_srcSystem()
                if sysid in self._dispatcher._drones:
                    self._dispatcher._drones[sysid]._process_message(msg)
                if msg.get_type() == "COMMAND_ACK":
                    if msg.command == mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE:
                        if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                            return "SUCCESS"
                        else:
                            return "ERR_CAPTURE"

        # No ACK received in time -- expected in SITL, continue anyway
        print("[CAPTURE] No ACK from sysid={0} within {1}s -- continuing".format(
            self._sysid, timeout))
        return "SUCCESS"

    def point_gimbal(self, pitch_deg: float, roll_deg: float = 0.0, yaw_deg: float = 0.0) -> str:
        """
        Command the gimbal via ArduPilot's mount control.
        pitch_deg: -90 = straight down, 0 = forward/horizontal
        Fire-and-forget -- no ACK wait, since the gimbal's actual
        joint position is observed by polling, not by a MAVLink ACK.
        """
        # Remember what we commanded so the mission can poll for arrival
        # (actual pitch -> gimbal_pitch_error()) instead of a fixed wait.
        self._cmd_gimbal_pitch_deg = pitch_deg
        self._cmd_gimbal_yaw_deg   = yaw_deg
        self._dispatcher.send_command_long(
            self._sysid,
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            p1=pitch_deg,
            p2=roll_deg,
            p3=yaw_deg,
            p7=mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING,
        )
        return "SUCCESS"

    def request_external_capture(self, request_id: int) -> str:
        """
        Sends MAV_CMD_USER_1 to the camera component (same sysid,
        compid=MAV_COMP_ID_CAMERA), asking it to capture a frame and
        upload it via MAVFTP. Fire-and-forget -- the result arrives
        later as a COMMAND_ACK, check it with get_camera_ack().

        request_id becomes part of the remote filename the camera
        component uses (capture_<request_id>.jpg), so the caller
        already knows what file to expect.
        """
        self._pending_capture_request_id = request_id
        self._camera_ack_result          = None   # reset, awaiting a fresh ack
        self._dispatcher.send_command_long(
            self._sysid,
            CAPTURE_COMMAND,
            p1=float(request_id),
            target_compid=MAV_COMP_ID_CAMERA,
        )
        return "SUCCESS"

    def get_camera_ack(self, request_id: int) -> int | None:
        """
        Returns the MAV_RESULT value from the camera component's ACK,
        or None if no ack has arrived yet for this request.

        Returns None (instead of the cached result) if request_id
        doesn't match the most recently sent request -- this is a
        sanity check against calling this for a stale/wrong request.
        """
        if self._pending_capture_request_id != request_id:
            return None
        return self._camera_ack_result