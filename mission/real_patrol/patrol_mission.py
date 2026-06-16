import os
import math
import logging
import time
import sys
from pathlib import Path

# ---------------------------------------------------------------------------
# Path setup — include wrapper if available
# ---------------------------------------------------------------------------
current_dir = Path(__file__).resolve().parent
sys.path.append(str(current_dir.parent.parent / "src"))

USE_WRAPPER = os.environ.get("USE_WRAPPER", "0") == "1"
if USE_WRAPPER:
    from wrapper import mavlink_connection as mav_connect
    logger_name = "FT_Mission"
else:
    from pymavlink.mavutil import mavlink_connection as mav_connect
    logger_name = "Std_Mission"

# ---------------------------------------------------------------------------
# ANSI colours (disabled automatically if output is not a terminal)
# ---------------------------------------------------------------------------
USE_COLOUR = sys.stdout.isatty()
RED    = "\033[91m" if USE_COLOUR else ""
YELLOW = "\033[93m" if USE_COLOUR else ""
GREEN  = "\033[92m" if USE_COLOUR else ""
RESET  = "\033[0m"  if USE_COLOUR else ""

# ---------------------------------------------------------------------------
# Mission config  (all overridable via env vars)
# ---------------------------------------------------------------------------
WAYPOINTS_FILENAME = os.environ.get("WAYPOINTS_FILE", "/app/missions/real_patrol/patrol.waypoints")

ALT_TARGET = float(os.environ.get("TARGET_ALT",  8.0))    # metres AGL
LAPS_TOTAL = int(os.environ.get("TOTAL_LAPS",    1))
LOITER_S   = float(os.environ.get("LOITER_S",    5.0))    # seconds at each WP
CONN_STR   = os.environ.get("CONNECTION_STR",    "udpin:0.0.0.0:14551")
MAX_TIME_S = float(os.environ.get("MAX_TIME_S",  600.0))  # 10 min hard limit
MAX_DIST_M = float(os.environ.get("MAX_DIST_M",  2000.0)) # 2 km hard limit

# Home / landing point (takeoff position — return here at end)
HOME_LAT   = float(os.environ.get("HOME_LAT", 39.3417199))
HOME_LON   = float(os.environ.get("HOME_LON", 22.9348478))

# ArduCopter custom_mode IDs
MODE_GUIDED = 4
MODE_LOITER = 5

# Stuck detector — resend goto after this many consecutive no-progress prints
STUCK_THRESHOLD = int(os.environ.get("STUCK_THRESHOLD", 3))

# ---------------------------------------------------------------------------
# Fail-injection config
# ---------------------------------------------------------------------------
FAIL_ENABLE = os.environ.get("FAIL_ENABLE", "0") == "1"
FAIL_CONFIG = os.environ.get("FAIL_CONFIG", "/app/fails/fail_config.txt")

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(level=logging.INFO, format="%(message)s")
logger = logging.getLogger(logger_name)

BASE_DIR = os.environ.get("CHECKPOINT_BASEDIR", "/mnt/checkpoints")
LOG_DIR  = f"{BASE_DIR}/mission_logs"
os.makedirs(LOG_DIR, exist_ok=True)

# ---------------------------------------------------------------------------
# Mission start time file
# Persists the drone's time_boot_ms at takeoff across container restarts.
# This ensures the time limit is measured against actual drone flight time,
# not Python wall-clock time which resets on every container restart.
# ---------------------------------------------------------------------------
MISSION_START_FILE = os.path.join(LOG_DIR, "mission_start_boot_ms.txt")

TOTAL_MESSAGES = 0


# ---------------------------------------------------------------------------
# Fail injection
# ---------------------------------------------------------------------------
# Full phase reference:
#
#   AFTER_TAKEOFF    drone airborne at ALT_TARGET, hasn't moved yet
#   AFTER_SEND       goto just sent, drone starts moving toward WP
#   DURING_FLYING    drone mid-leg, unknown position between two WPs  ← hardest recovery
#   AFTER_ARRIVE     drone hovering at WP, loiter not started
#   DURING_LOITER    drone hovering at WP, loiter timer ~halfway
#   AFTER_LOITER     drone hovering at WP, loiter complete
#   BEFORE_LAND      mission done, drone still airborne over home
#
# fail_config.txt format (use 0,0 for phases outside the WP loop):
#   # lap, wp, phase
#   0, 0, AFTER_TAKEOFF
#   1, 3, AFTER_SEND
#   1, 3, DURING_FLYING
#   1, 3, AFTER_ARRIVE
#   1, 3, DURING_LOITER
#   1, 3, AFTER_LOITER
#   0, 0, BEFORE_LAND
# ---------------------------------------------------------------------------

def load_fail_config(filename):
    points = []
    with open(filename) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(",")]
            points.append((int(parts[0]), int(parts[1]), parts[2].upper()))
    return points


if FAIL_ENABLE:
    try:
        FAIL_POINTS = load_fail_config(FAIL_CONFIG)
        logger.info(f"{YELLOW}[FAIL] Injection enabled — {len(FAIL_POINTS)} fail point(s) from {FAIL_CONFIG}{RESET}")
        for f_lap, f_wp, f_phase in FAIL_POINTS:
            logger.info(f"{YELLOW}         lap={f_lap}  wp={f_wp}  phase={f_phase}{RESET}")
    except FileNotFoundError:
        logger.error(f"{RED}[FAIL] Config file not found: {FAIL_CONFIG} — disabling fail injection{RESET}")
        FAIL_ENABLE = False
        FAIL_POINTS = []
else:
    FAIL_POINTS = []


def maybe_fail(phase, lap=None, wp=None):
    if not FAIL_ENABLE:
        return
    phase_upper = phase.upper()

    for (f_lap, f_wp, f_phase) in FAIL_POINTS:
        if f_phase != phase_upper:
            continue

        if f_phase in ("AFTER_TAKEOFF", "BEFORE_LAND"):
            match = True
        else:
            match = (lap == f_lap and wp == f_wp)

        if not match:
            continue

        crash_file = os.path.join(LOG_DIR, f"last_crash_{f_phase}_L{f_lap}_W{f_wp}.json")
        if os.path.exists(crash_file):
            continue

        with open(crash_file, "w") as f:
            f.write(f'{{"phase": "{phase_upper}", "lap": {lap}, "wp": {wp}}}')

        logger.error(f"{RED}[FAIL] Triggered at phase={phase_upper}  lap={lap}  wp={wp}{RESET}")
        os._exit(137)


# ---------------------------------------------------------------------------
# Drone clock helpers
# ---------------------------------------------------------------------------

def save_mission_start(boot_ms):
    """
    Persist the drone's time_boot_ms at takeoff to disk.
    On container restart this file lets us recover how long the drone
    has actually been flying, regardless of Python wall-clock resets.
    """
    with open(MISSION_START_FILE, "w") as f:
        f.write(str(boot_ms))
    logger.info(f"[CLOCK] Mission start saved: boot_ms={boot_ms}")


def load_mission_start():
    """
    Load the saved mission start boot_ms.
    Returns None if no file exists (fresh start).
    """
    if not os.path.exists(MISSION_START_FILE):
        return None
    with open(MISSION_START_FILE) as f:
        val = f.read().strip()
        return int(val) if val else None


def drone_elapsed_s(current_boot_ms, start_boot_ms):
    """Elapsed mission time in seconds using the drone's own clock."""
    return (current_boot_ms - start_boot_ms) / 1000.0


# ---------------------------------------------------------------------------
# MAVLink helpers
# ---------------------------------------------------------------------------

def _check_loiter(msg, sysid, abort_flag):
    """Set abort_flag if this heartbeat shows the drone switched to LOITER."""
    if msg.get_type() == "HEARTBEAT" and msg.get_srcSystem() == sysid:
        if msg.custom_mode == MODE_LOITER:
            logger.warning(f"{YELLOW}[LISTENER] LOITER mode detected — aborting mission loop.{RESET}")
            abort_flag[0] = True


def haversine_m(lat1, lon1, lat2, lon2):
    """Flat-earth distance in metres (accurate enough for <500 m)."""
    dlat = (lat2 - lat1) * 111319.5
    dlon = (lon2 - lon1) * 111319.5 * math.cos(math.radians((lat1 + lat2) / 2))
    return math.sqrt(dlat ** 2 + dlon ** 2)


def send_goto(conn, sysid, lat, lon, alt):
    """Send a COMMAND_INT goto command."""
    conn.mav.command_int_send(
        sysid, 1, 6, 192, 0, 0, -1, 0, 0, 0,
        int(lat * 1e7), int(lon * 1e7), int(alt)
    )


def track_arrival(conn, sysid, t_lat, t_lon, abort_flag,
                  tolerance=1.5, lap=None, wp=None, resend_alt=None,
                  start_boot_ms=None, total_dist_m=0.0,
                  max_dist_m=None, max_time_s=None):
    """
    Consume the MAVLink stream until the drone arrives within `tolerance` metres
    OR a LOITER mode change is detected OR a safety limit is exceeded.

    Time and distance limits are enforced using DRONE data:
      - Elapsed time : drone's time_boot_ms vs saved mission start
      - Distance     : real GPS position deltas, not planned leg distances

    Returns (arrived: bool, dist_flown_m: float, abort_reason: str|None)
      arrived=True   → reached waypoint normally
      arrived=False  → aborted (LOITER / limit exceeded)
    """
    global TOTAL_MESSAGES
    count = 0
    during_flying_fired = False
    last_dist  = None
    stuck_count = 0

    # For GPS delta accumulation
    prev_lat = None
    prev_lon = None
    dist_this_leg = 0.0

    while True:
        msg = conn.recv_match(type=["GLOBAL_POSITION_INT", "HEARTBEAT"], blocking=True)
        TOTAL_MESSAGES += 1
        if not msg:
            continue

        _check_loiter(msg, sysid, abort_flag)
        if abort_flag[0]:
            return False, dist_this_leg, "LOITER mode detected during transit"

        if msg.get_type() == "GLOBAL_POSITION_INT" and msg.get_srcSystem() == sysid:
            count += 1
            c_lat = msg.lat / 1e7
            c_lon = msg.lon / 1e7

            # ── Accumulate actual GPS distance ────────────────────────────
            if prev_lat is not None:
                step = haversine_m(prev_lat, prev_lon, c_lat, c_lon)
                # Filter GPS jitter — only count steps > 0.1m
                if step > 0.1:
                    dist_this_leg += step
            prev_lat, prev_lon = c_lat, c_lon

            # ── Distance limit check (drone data) ─────────────────────────
            if max_dist_m is not None:
                running_total = total_dist_m + dist_this_leg
                if running_total > max_dist_m:
                    reason = f"distance limit ({running_total:.0f}m > {max_dist_m:.0f}m)"
                    logger.warning(f"{YELLOW}[LIMIT] {reason}{RESET}")
                    return False, dist_this_leg, reason

            # ── Time limit check (drone clock) ────────────────────────────
            if max_time_s is not None and start_boot_ms is not None:
                elapsed = drone_elapsed_s(msg.time_boot_ms, start_boot_ms)
                if elapsed >= max_time_s:
                    reason = f"time limit ({elapsed:.0f}s >= {max_time_s:.0f}s)"
                    logger.warning(f"{YELLOW}[LIMIT] {reason}{RESET}")
                    return False, dist_this_leg, reason

            dist_to_wp = haversine_m(c_lat, c_lon, t_lat, t_lon)

            if count % 10 == 0:
                elapsed_str = ""
                if start_boot_ms is not None:
                    e = drone_elapsed_s(msg.time_boot_ms, start_boot_ms)
                    elapsed_str = f"  t={e:.0f}s"
                logger.info(f"   ... {dist_to_wp:.1f}m to WP  "
                            f"flown={total_dist_m + dist_this_leg:.0f}m{elapsed_str}")

                # ── Stuck detector ────────────────────────────────────────
                if last_dist is not None and abs(dist_to_wp - last_dist) < 0.5:
                    stuck_count += 1
                    if stuck_count >= STUCK_THRESHOLD and resend_alt is not None:
                        logger.warning(f"{YELLOW}   [STUCK] No progress — resending goto{RESET}")
                        send_goto(conn, sysid, t_lat, t_lon, resend_alt)
                        stuck_count = 0
                else:
                    stuck_count = 0

                last_dist = dist_to_wp

                # ── DURING_FLYING ─────────────────────────────────────────
                if not during_flying_fired:
                    during_flying_fired = True
                    maybe_fail("DURING_FLYING", lap=lap, wp=wp)

            if dist_to_wp < tolerance:
                return True, dist_this_leg, None


def wait_veh_time(conn, sysid, seconds, abort_flag,
                  lap=None, wp=None,
                  start_boot_ms=None, total_dist_m=0.0,
                  max_time_s=None):
    """
    Loiter for `seconds` of vehicle boot-time.
    DURING_LOITER fail is injected once at the halfway point.
    Time limit is also checked here using the drone's clock.
    Returns (ok: bool, abort_reason: str|None)
    """
    global TOTAL_MESSAGES
    logger.info(f"[{sysid}] Loitering {seconds}s...")
    start_ms = None
    during_loiter_fired = False

    while start_ms is None:
        msg = conn.recv_match(type=["GLOBAL_POSITION_INT", "HEARTBEAT"], blocking=True)
        TOTAL_MESSAGES += 1
        if not msg:
            continue
        _check_loiter(msg, sysid, abort_flag)
        if abort_flag[0]:
            return False, "LOITER mode detected during loiter"
        if msg.get_type() == "GLOBAL_POSITION_INT" and msg.get_srcSystem() == sysid:
            start_ms = msg.time_boot_ms

    while True:
        msg = conn.recv_match(type=["GLOBAL_POSITION_INT", "HEARTBEAT"], blocking=True)
        TOTAL_MESSAGES += 1
        if not msg:
            continue

        _check_loiter(msg, sysid, abort_flag)
        if abort_flag[0]:
            return False, "LOITER mode detected during loiter"

        if msg.get_type() == "GLOBAL_POSITION_INT" and msg.get_srcSystem() == sysid:
            loiter_elapsed = (msg.time_boot_ms - start_ms) / 1000.0

            # ── Time limit check during loiter ────────────────────────────
            if max_time_s is not None and start_boot_ms is not None:
                mission_elapsed = drone_elapsed_s(msg.time_boot_ms, start_boot_ms)
                if mission_elapsed >= max_time_s:
                    reason = f"time limit ({mission_elapsed:.0f}s >= {max_time_s:.0f}s)"
                    logger.warning(f"{YELLOW}[LIMIT] {reason}{RESET}")
                    return False, reason

            # ── DURING_LOITER ─────────────────────────────────────────────
            if not during_loiter_fired and loiter_elapsed >= seconds / 2.0:
                during_loiter_fired = True
                maybe_fail("DURING_LOITER", lap=lap, wp=wp)

            if loiter_elapsed >= seconds:
                return True, None


# ---------------------------------------------------------------------------
# Mission Controller
# ---------------------------------------------------------------------------

class MissionController:
    def __init__(self):
        if USE_WRAPPER:
            self.master = mav_connect(CONN_STR, wrapper_log=True)
        else:
            self.master = mav_connect(CONN_STR)
        self.active_sysid = 1

    def init_comms(self):
        global TOTAL_MESSAGES
        logger.info("--- INITIALIZING COMMS ---")
        logger.info(f"Waiting for heartbeat from UAV {self.active_sysid}...")
        while True:
            msg = self.master.recv_match(type="HEARTBEAT", blocking=True)
            TOTAL_MESSAGES += 1
            if msg and msg.get_srcSystem() == self.active_sysid:
                logger.info(f"   -> UAV {self.active_sysid} online.")
                break
        logger.info("--- SYSTEM READY ---\n")

    def exec_takeoff(self):
        """
        Arms, takes off, waits for altitude.
        Returns start_boot_ms — the drone's time_boot_ms at takeoff completion.
        If a saved value exists (restart scenario) that is returned instead,
        preserving the original mission start time across container restarts.
        """
        global TOTAL_MESSAGES
        sysid = self.active_sysid

        # ── Check for restart scenario ────────────────────────────────────
        saved_start = load_mission_start()
        if saved_start is not None:
            logger.info(f"{YELLOW}[CLOCK] Restart detected — using saved mission start: "
                        f"boot_ms={saved_start}{RESET}")

        logger.info(f"[{sysid}] ── LAUNCH SEQUENCE ──────────────────────")
        self.master.mav.command_long_send(sysid, 1, 176, 0, 1, 4, 0, 0, 0, 0, 0)  # GUIDED
        self.master.mav.command_long_send(sysid, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0)  # ARM
        self.master.mav.command_long_send(sysid, 1, 22,  0, 0, 0, 0, 0, 0, 0, ALT_TARGET)  # TAKEOFF
        logger.info(f"[{sysid}] GUIDED — arming and climbing to {ALT_TARGET}m...")

        start_boot_ms = None
        while True:
            msg = self.master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
            TOTAL_MESSAGES += 1
            if msg and msg.get_srcSystem() == sysid:
                alt = msg.relative_alt / 1000.0
                if alt > ALT_TARGET * 0.95:
                    start_boot_ms = msg.time_boot_ms
                    logger.info(f"{GREEN}[{sysid}] Altitude reached ({alt:.1f}m). ✓{RESET}")
                    break

        # Fresh start → save to disk so restarts can recover it
        if saved_start is None:
            save_mission_start(start_boot_ms)
            maybe_fail("AFTER_TAKEOFF")
            return start_boot_ms
        else:
            # Restart → use the original start time
            return saved_start

    def land_at_home(self, start_boot_ms):
        """Fly back to home coordinates and send LAND command."""
        sysid = self.active_sysid
        abort_dummy = [False]
        logger.info(f"\n[{sysid}] ── RETURNING TO HOME ────────────────────")
        send_goto(self.master, sysid, HOME_LAT, HOME_LON, ALT_TARGET)
        track_arrival(self.master, sysid, HOME_LAT, HOME_LON, abort_dummy,
                      tolerance=2.0, resend_alt=ALT_TARGET,
                      start_boot_ms=start_boot_ms)

        maybe_fail("BEFORE_LAND")

        logger.info(f"[{sysid}] Over home — sending LAND.")
        self.master.mav.command_long_send(sysid, 1, 21, 0, 0, 0, 0, 0, 0, 0, 0)  # NAV_LAND

        # Clean up mission start file — mission is truly done
        if os.path.exists(MISSION_START_FILE):
            os.remove(MISSION_START_FILE)
            logger.info("[CLOCK] Mission start file cleared.")


# ---------------------------------------------------------------------------
# Waypoint loader
# ---------------------------------------------------------------------------

def load_waypoints(filename):
    wps = []
    with open(filename) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            wps.append((float(parts[0]), float(parts[1])))
    return wps


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    # ── Load waypoints ────────────────────────────────────────────────────
    try:
        wps = load_waypoints(WAYPOINTS_FILENAME)
    except FileNotFoundError:
        logger.error(f"{RED}Waypoints file not found: {WAYPOINTS_FILENAME}{RESET}")
        sys.exit(1)

    if not wps:
        logger.error(f"{RED}No waypoints loaded — aborting.{RESET}")
        sys.exit(1)

    logger.info(f"Loaded {len(wps)} waypoints from {WAYPOINTS_FILENAME}")

    # ── Connect & takeoff ─────────────────────────────────────────────────
    ctrl = MissionController()
    ctrl.init_comms()

    # start_boot_ms: drone's clock at takeoff — persists across restarts
    start_boot_ms = ctrl.exec_takeoff()

    # ── Mission state ─────────────────────────────────────────────────────
    abort_flag   = [False]
    abort_reason = None
    total_dist_m = 0.0       # accumulated from real GPS deltas
    prev_lat     = HOME_LAT
    prev_lon     = HOME_LON

    # ── Main patrol loop ──────────────────────────────────────────────────
    for lap in range(LAPS_TOTAL):
        logger.info(f"\n{'='*50}")
        logger.info(f"  LAP {lap + 1} / {LAPS_TOTAL}")
        logger.info(f"{'='*50}")

        for i, (wp_lat, wp_lon) in enumerate(wps):
            wp_idx  = i + 1
            lap_idx = lap + 1

            # ── Pre-leg checks (using drone data) ─────────────────────────
            if abort_flag[0]:
                abort_reason = "LOITER mode detected"
                break

            # Send goto ────────────────────────────────────────────────────
            logger.info(f"\n[{ctrl.active_sysid}] WP {wp_idx}/{len(wps)}"
                        f"  flown={total_dist_m:.0f}m")
            send_goto(ctrl.master, ctrl.active_sysid, wp_lat, wp_lon, ALT_TARGET)

            maybe_fail("AFTER_SEND", lap=lap_idx, wp=wp_idx)

            # ── Fly to WP ─────────────────────────────────────────────────
            arrived, dist_flown, reason = track_arrival(
                ctrl.master, ctrl.active_sysid, wp_lat, wp_lon, abort_flag,
                lap=lap_idx, wp=wp_idx, resend_alt=ALT_TARGET,
                start_boot_ms=start_boot_ms, total_dist_m=total_dist_m,
                max_dist_m=MAX_DIST_M, max_time_s=MAX_TIME_S
            )

            # Accumulate real GPS distance regardless of outcome
            total_dist_m += dist_flown

            if not arrived:
                abort_reason = reason
                break

            prev_lat, prev_lon = wp_lat, wp_lon
            logger.info(f"{GREEN}[{ctrl.active_sysid}] ✓ ARRIVED WP {wp_idx}"
                        f"  total_flown={total_dist_m:.0f}m{RESET}")

            maybe_fail("AFTER_ARRIVE", lap=lap_idx, wp=wp_idx)

            # ── Loiter ────────────────────────────────────────────────────
            ok, reason = wait_veh_time(
                ctrl.master, ctrl.active_sysid, LOITER_S, abort_flag,
                lap=lap_idx, wp=wp_idx,
                start_boot_ms=start_boot_ms, total_dist_m=total_dist_m,
                max_time_s=MAX_TIME_S
            )
            if not ok:
                abort_reason = reason
                break

            maybe_fail("AFTER_LOITER", lap=lap_idx, wp=wp_idx)

        if abort_reason:
            break

    # ── Mission end ───────────────────────────────────────────────────────
    # Get final elapsed from drone clock
    final_elapsed = None
    try:
        msg = ctrl.master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=3)
        if msg:
            final_elapsed = drone_elapsed_s(msg.time_boot_ms, start_boot_ms)
    except Exception:
        pass

    elapsed_str = f"{final_elapsed:.0f}s" if final_elapsed else "unknown"

    print(f"\n{'='*50}")
    if abort_reason:
        logger.warning(f"{YELLOW}MISSION ABORTED: {abort_reason}{RESET}")
    else:
        logger.info(f"{GREEN}MISSION COMPLETE — {LAPS_TOTAL} lap(s) flown{RESET}")
    print(f"  Distance (actual GPS) : {total_dist_m:.0f}m / {MAX_DIST_M:.0f}m")
    print(f"  Time     (drone clock): {elapsed_str} / {MAX_TIME_S:.0f}s")
    print(f"{'='*50}\n")

    # ── LOITER abort → drone already loitering, just exit ─────────────────
    if abort_reason and "LOITER" in abort_reason:
        logger.info("Drone is in LOITER — exiting. Drone holds its position.")
        ctrl.master.close()
        sys.exit(0)

    # ── Normal end or limit abort → return to home and land ───────────────
    ctrl.land_at_home(start_boot_ms)
    ctrl.master.close()
    logger.info("Mission program exiting cleanly.")


if __name__ == "__main__":
    main()