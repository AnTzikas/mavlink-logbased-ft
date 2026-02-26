import os
import math
import csv
import logging
import time
from datetime import datetime
import sys
from pathlib import Path

# Include Wrapper or Default Pymavlink
current_dir = Path(__file__).resolve().parent
sys.path.append(str(current_dir.parent / "src"))

USE_WRAPPER = os.environ.get("USE_WRAPPER", "0") == "1"
if USE_WRAPPER:
    # Now that 'src' is in path, we can import 'wrapper' directly
    from wrapper import mavlink_connection as mav_connect
    logger_name = "FT_Mission"
else:
    from pymavlink.mavutil import mavlink_connection as mav_connect
    logger_name = "Std_Mission"

TOTAL_MESSAGES=0
WAYPOINTS_FILENAME="/app/missions/patrol.waypoints"
# Mission Config
ALT_TARGET = float(os.environ.get("TARGET_ALT", 15.0))
LAPS_TOTAL = int(os.environ.get("TOTAL_LAPS", 3))
BATT_CRIT  = float(os.environ.get("BATT_LIMIT", 60.0))
CONN_STR   = os.environ.get("CONNECTION_STR", 'udpin:0.0.0.0:14551')
SPEED_MPS  = float(os.environ.get("SPEED", 5.0))

# Fail injection logic
FAIL_ENABLE = os.environ.get("FAIL_ENABLE", "0") == "1"
FAIL_PHASE  = os.environ.get("FAIL_PHASE", "")
FAIL_LAP    = int(os.environ.get("FAIL_LAP", "1"))   # 1-based
FAIL_WP     = int(os.environ.get("FAIL_WP", "2"))    # 1-based

# Mission Logging Setup
logging.basicConfig(level=logging.INFO, format='%(asctime)s %(message)s', datefmt='%H:%M:%S')
logger = logging.getLogger(logger_name)
BASE_DIR = os.environ.get("CHECKPOINT_BASEDIR", "/mnt/checkpoints")
LOG_DIR = f"{BASE_DIR}/mission_logs"
os.makedirs(LOG_DIR, exist_ok=True)
# timestamp = int(time.time())
MISSION_LOGFILE= f"{LOG_DIR}/mission_log_{FAIL_PHASE}.csv"

if FAIL_ENABLE:
    print(f"Mission program started with fail injection at lap {FAIL_LAP} and waypoint {FAIL_WP}")

def maybe_fail(phase, lap=None, wp=None):
    crash_file = os.path.join(LOG_DIR, "last_crash.json")

    # 1. Check if we already crashed in a previous container life
    if os.path.exists(crash_file) or not FAIL_ENABLE:
        # print("Crash happened")
        return

    # 2. Match the trigger
    if phase.upper() == FAIL_PHASE.upper() :
        # 3. RECORD THE CRASH before dying
        with open(crash_file, "w") as f:
            f.write(f'{{"phase": "{phase}", "lap": {lap}, "wp": {wp}}}')

        logger.error(f"FAIL-INJECT: Triggered at {phase}\n")
        os._exit(137)

def log_metric(event, active_sysid=None, lap=None, wp=None, extra=""):

    with open(MISSION_LOGFILE, "a") as f:
        ts = datetime.now().strftime("%H:%M:%S.%f")
        f.write(f"{ts},{event},{active_sysid},{lap},{wp},{extra}\n")
        f.flush()
        os.fsync(f.fileno())

def fetch_position(conn, sysid):
    """Filters stream for specific System ID and reports status."""
    global TOTAL_MESSAGES
    logger.info(f"[{sysid}] Syncing GPS position...")
    while True:
        msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        TOTAL_MESSAGES += 1
        if msg and msg.get_srcSystem() == sysid:
            return msg.lat / 1e7, msg.lon / 1e7

def track_arrival(conn, sysid, t_lat, t_lon, tolerance=1.5):
    """
    Consumes stream until SPECIFIC drone arrives. 
    Prints distance remaining to show the system is 'alive'.
    """
    global TOTAL_MESSAGES
    logger.info(f"[{sysid}] Moving to target... (Waiting for arrival)")
    # last_print = time.time()
    count = 0
    
    while True:
        msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        TOTAL_MESSAGES += 1
        count += 1
        # Filter: Only process data from the active drone
        if msg and msg.get_srcSystem() == sysid:
            c_lat, c_lon = msg.lat / 1e7, msg.lon / 1e7
            dist = math.sqrt(((t_lat - c_lat) * 111319.5)**2 + ((t_lon - c_lon) * 111319.5)**2)
            
            # [VISUAL FEEDBACK] Print distance every 2 seconds so you know it's not frozen
            if count % 10 == 0:
                logger.info(f"   ... Distance to WP: {dist:.1f}m")

            if dist < tolerance: 
                logger.info(f"[{sysid}] >> ARRIVED at Waypoint (Dist: {dist:.2f}m)")
                return

def wait_veh_time(conn, sysid, seconds):
    """Syncs with SPECIFIC drone's boot time."""
    global TOTAL_MESSAGES
    logger.info(f"[{sysid}] Loitering for {seconds}s (Vehicle Time)...")
    start_ms = None
    
    # Get baseline time
    while start_ms is None:
        msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        TOTAL_MESSAGES += 1
        if msg and msg.get_srcSystem() == sysid:
            start_ms = msg.time_boot_ms
            
    # Wait loop
    while True:
        msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        TOTAL_MESSAGES += 1
        if msg and msg.get_srcSystem() == sysid:
            elapsed = (msg.time_boot_ms - start_ms) / 1000.0
            if elapsed >= seconds: 
                break

def get_battery(conn, sysid):
    global TOTAL_MESSAGES
    while True:
        msg = conn.recv_match(type='SYS_STATUS', blocking=True)
        TOTAL_MESSAGES += 1
        if msg and msg.get_srcSystem() == sysid:
            return msg.battery_remaining

# --- Mission Controller ---
class RelayController:
    def __init__(self):
        
        if USE_WRAPPER:
            self.master = mav_connect(CONN_STR, wrapper_log = True)
        else:
            self.master = mav_connect(CONN_STR)

        self.active_sysid = 1
        self.backup_sysid = 2
        self.handover_done = False

    def init_comms(self):
        global TOTAL_MESSAGES
        logger.info("--- INITIALIZING COMMS ---")
        logger.info("Listening for Heartbeats from UAV 1 & 2...")
        
        seen = set()
        while len(seen) < 2:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True)
            TOTAL_MESSAGES += 1
            if msg:
                sid = msg.get_srcSystem()
                if sid in [1, 2] and sid not in seen:
                    seen.add(sid)
                    logger.info(f"   -> Found UAV {sid}")
        
        # Force the drone to ignore the compass error
        # self.master.mav.param_set_send(
        #     sid, 1, 
        #     b'ARMING_CHECK', 
        #     0, 
        #     mavutil.mavlink.MAV_PARAM_TYPE_INT32
        # )
        self.wait_for_ekf_gps(1)
        # self.wait_for_ekf_gps(2)
        log_metric("INIT_COMMS", extra="both_systems_detected")
        logger.info("--- SYSTEM READY ---")

    def wait_for_ekf_gps(self, sysid):
        """
        Blocks until the drone's EKF3 reports it is using GPS.
        Essential for preventing 'Pre-arm: Need Position Estimate' errors.
        """
        logger.info(f"[{sysid}] Waiting for UAV to be ready for flight...")
        imu0_ok = False
        imu1_ok = False

        while not (imu0_ok and imu1_ok):
            # Listen for any status text message
            msg = self.master.recv_match(type='STATUSTEXT', blocking=True, timeout=5)
            
            if msg and msg.get_srcSystem() == sysid:
                text = msg.text
                # ArduPilot specific log strings
                if "EKF3 IMU0 is using GPS" in text:
                    imu0_ok = True
                    logger.info(f"   -> [{sysid}] IMU0 aligned with GPS")
                if "EKF3 IMU1 is using GPS" in text:
                    imu1_ok = True
                    logger.info(f"   -> [{sysid}] IMU1 aligned with GPS")
                
            
        logger.info(f"[{sysid}] EKF READY: Both IMUs are using GPS.")
    def exec_takeoff(self, sysid):
        global TOTAL_MESSAGES
        tag = f"[{sysid}]"

        log_metric("TAKEOFF_START", active_sysid=sysid)

        logger.info(f"{tag} LAUNCH SEQUENCE INITIATED")
        
        self.master.mav.command_long_send(sysid, 1, 176, 0, 1, 4, 0, 0, 0, 0, 0) # Mode
        log_metric("MODE_GUIDED_SENT", active_sysid=sysid)
        logger.info(f"{tag} Mode -> GUIDED")
        
        self.master.mav.command_long_send(sysid, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0) # Arm
        log_metric("ARM_SENT", active_sysid=sysid)
        logger.info(f"{tag} Arming Motors...")
        
        self.master.mav.command_long_send(sysid, 1, 22, 0, 0, 0, 0, 0, 0, 0, ALT_TARGET) # Takeoff
        log_metric("TAKEOFF_SENT", active_sysid=sysid, extra=f"alt={ALT_TARGET}")
        logger.info(f"{tag} Takeoff Command Sent. Climbing to {ALT_TARGET}m...")
        
        # Wait for altitude using the ID filter
        while True:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            TOTAL_MESSAGES += 1
            if msg and msg.get_srcSystem() == sysid:
                alt = msg.relative_alt / 1000.0
                if alt > (ALT_TARGET * 0.95): 
                    logger.info(f"{tag} Altitude Reached ({alt:.1f}m).")
                    break
        log_metric("TAKEOFF_COMPLETE", active_sysid=sysid, extra=f"alt~={ALT_TARGET}")

    def manage_handover(self):
        global TOTAL_MESSAGES
        old_id = self.active_sysid
        new_id = self.backup_sysid
        
        print("\n" + "="*50)
        logger.warning(f"HANDOVER TRIGGERED: RETIRING UAV {old_id}")
        print("="*50 + "\n")

        
        log_metric("HANDOVER_TRIGGER", active_sysid=old_id, extra=f"to={new_id}")
        
        # RTL Active Drone
        logger.info(f"[{old_id}] Sending RTL Command...")
        self.master.mav.command_long_send(old_id, 1, 20, 0, 0, 0, 0, 0, 0, 0, 0)
        log_metric("UAV1_RTL_SENT", active_sysid=old_id)
        maybe_fail("AFTER_RTL")

        logger.info(f"[{old_id}] Waiting for landing & disarm...")
        while True:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True)
            TOTAL_MESSAGES += 1
            if msg and msg.get_srcSystem() == old_id:
                if not (msg.base_mode & 128): 
                    logger.info(f"[{old_id}] DISARM CONFIRMED.")
                    break 
        
        log_metric("UAV1_DISARM_CONFIRMED", active_sysid=old_id)
        
        print("\n" + "-"*30)
        logger.info(f"ACTIVATING UAV {new_id}")
        print("-"*30 + "\n")
        log_metric("UAV2_TAKEOFF_START", active_sysid=new_id)
        # Launch Backup
        self.exec_takeoff(new_id)
        self.active_sysid = new_id
        self.handover_done = True
        log_metric("HANDOVER_COMPLETE", active_sysid=new_id)

def main():

    global TOTAL_MESSAGES
    if os.path.exists("experiment_metrics.csv"): os.remove("experiment_metrics.csv")
    ctrl = RelayController()
    ctrl.init_comms()

    # Get geometry based on Drone 1
    h_lat, h_lon = fetch_position(ctrl.master, ctrl.active_sysid)
    wps = []
    
    # Simple Geometry helper
    def get_off(lat, lon, n, e):
        d_lat = n / 6378137.0
        d_lon = e / (6378137.0 * math.cos(math.pi * lat / 180))
        return (lat + (d_lat * 180/math.pi), lon + (d_lon * 180/math.pi))

    try:
        with open(WAYPOINTS_FILENAME, "r") as f:
            for row in csv.reader(f):
                if row: wps.append(get_off(h_lat, h_lon, float(row[0]), float(row[1])))
    except FileNotFoundError: return
    log_metric(
        "MISSION_CONFIG",
        active_sysid=ctrl.active_sysid,
        extra=f"laps={LAPS_TOTAL},wps={len(wps)},alt={ALT_TARGET},batt_crit={BATT_CRIT}"
    )

    ctrl.exec_takeoff(ctrl.active_sysid)
    log_metric("PHASE_ENTER_PATROL", active_sysid=ctrl.active_sysid)
    
    for lap in range(LAPS_TOTAL):
        print("\n")
        logger.info(f"=== STARTING LAP {lap+1} ===")
        log_metric("LAP_START", active_sysid=ctrl.active_sysid, lap=lap + 1)
        
        for i, (wp_lat, wp_lon) in enumerate(wps):
            wp_idx = i + 1
            lap_idx = lap + 1

            # Check battery specific to ACTIVE drone
            if not ctrl.handover_done:
                batt = get_battery(ctrl.master, ctrl.active_sysid)
                if batt < BATT_CRIT:
                    logger.warning(f"[{ctrl.active_sysid}] BATTERY CRITICAL ({batt}%).")
                    log_metric(
                        "BATTERY_CRITICAL",
                        active_sysid=ctrl.active_sysid,
                        lap=lap_idx,
                        wp=wp_idx,
                        extra=f"batt={batt}"
                    )
                    ctrl.manage_handover()

            # Send Move Command
            logger.info(f"[{ctrl.active_sysid}] Moving to WP {i+1}...")
            ctrl.master.mav.command_int_send(
                ctrl.active_sysid, 1, 6, 192, 0, 0, -1, 0, 0, 0, 
                int(wp_lat * 1e7), int(wp_lon * 1e7), int(ALT_TARGET)
            )

            log_metric(
                "MOVE_SENT",
                active_sysid=ctrl.active_sysid,
                lap=lap_idx,
                wp=wp_idx,
                extra=f"lat={wp_lat:.7f},lon={wp_lon:.7f},alt={ALT_TARGET}"
            )
            if lap_idx==FAIL_LAP and wp_idx==FAIL_WP:
                maybe_fail("AFTER_SEND")

            #Arrival + loiter
            track_arrival(ctrl.master, ctrl.active_sysid, wp_lat, wp_lon)
            log_metric("ARRIVED", active_sysid=ctrl.active_sysid, lap=lap_idx, wp=wp_idx)

            log_metric("LOITER_START", active_sysid=ctrl.active_sysid, lap=lap_idx, wp=wp_idx, extra="sec=5")
            wait_veh_time(ctrl.master, ctrl.active_sysid, 5.0)
            log_metric("LOITER_COMPLETE", active_sysid=ctrl.active_sysid, lap=lap_idx, wp=wp_idx)
        
        log_metric("LAP_END", active_sysid=ctrl.active_sysid, lap=lap + 1)

    logger.info("MISSION COMPLETE. Sending RTL to final vehicle.")
    log_metric("MISSION_COMPLETE", active_sysid=ctrl.active_sysid)

    ctrl.master.mav.command_long_send(ctrl.active_sysid, 1, 20, 0, 0, 0, 0, 0, 0, 0, 0)
    log_metric("RTL_SENT", active_sysid=ctrl.active_sysid)
    # logger.info(f"Total Messages Received: {TOTAL_MESSAGES}")

    ctrl.master.close()

    

if __name__ == '__main__':
    main()
