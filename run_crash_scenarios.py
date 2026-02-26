import subprocess
import time
import os
import argparse
import csv
from pathlib import Path

# Configuration
SITL_IMAGE = "ardupilot-sitl:latest"
SITL_COMMAND = "docker compose -f docker/compose/compose.sitl-drones.yaml"
MISSION_IMAGE = "mission-container:latest"
# MISSION_COMMAND = "docker compose -f docker/compose/compose.mission.yaml"

CHECKPOINTS_DIR = f"{os.getcwd()}/experiment_logs"
ENV_FILE = "mission/mission.env"
MISSION_CONTAINER_LOGFILE = "experiment_logs/container_logs/mission_container.log"
DRONE_CONTAINER_LOGFILE = "experiment_logs/container_logs/ardupilot_output.log"
CRASH_RECOVERY_TIME = 40

def run_cmd(cmd, detach=False, log_file=None):
    if detach:
        # Open the file in write mode
        out = open(log_file, 'a') if log_file else None
        # Start the process and send output to the file
        return subprocess.Popen(cmd, shell=True, stdout=out, stderr=out)
    return subprocess.run(cmd, shell=True, check=True)

def run_mission_timewindow_crash(mission_cmd, crash_times):
    

    crash_times.sort()

    if len(crash_times) != len(set(crash_times)):
        raise SystemExit("Error: crash times must be unique (no duplicates).")

    # Convert absolute times -> per-phase survival durations (deltas)
    # Example: [60,120,180] -> [60,60,60]
    deltas = []
    prev = 0
    for t in crash_times:
        deltas.append(t - prev)
        prev = t

    # Schedule: crash phases, then one final run to completion (no crash)
    crash_schedule = deltas + [None] 

    for i, interval in enumerate(crash_schedule):
        is_last = (i == len(crash_schedule) - 1)

        if interval is None:
            print("\nStarting Mission (No Crash Scheduled)")
        else:
            # interval can be 0 if the user asked for a crash at t=0 (immediate crash)
            print(f"\nStarting Mission (Crash after {interval}s) ---")

        # Start the mission container
        mission_proc = run_cmd(mission_cmd, detach=True, log_file=MISSION_CONTAINER_LOGFILE)

        if interval is None:
            # Final run: wait until mission exits naturally
            mission_proc.wait()
        else:
            # Crash phase: wait 'interval' seconds then kill and restart in next loop iteration
            if interval > 0:
                time.sleep(interval)

            print(F"Killing mission_container\nWait {CRASH_RECOVERY_TIME} seconds before restarting container")
            time.sleep(1)
            run_cmd("docker kill mission_container")
            # subprocess.run("docker kill mission_container", shell=True, capture_output=True)
            time.sleep(CRASH_RECOVERY_TIME)

def run_mission_phase_crash(mission_cmd, phase_args):

    defaults = [None, "2", "3"]
    # Slice phase_args to prevent index errors, then pad with defaults
    params = (phase_args + defaults[len(phase_args):])[:3]
    
    phase_name, lap_str, wp_str = params

    # 2. Final validation/conversion
    try:
        lap_idx = int(lap_str)
        wp_idx = int(wp_str)
    except ValueError:
        print("Error: Lap and Waypoint must be integers.")
        return

    print(f"\n--- [Phase-Based Fault Injection] ---")
    print(f"Targeting: Phase='{phase_name}', Lap={lap_idx}, WP={wp_idx}")

    # 3. Inject flags into the command
    injection_flags = [
        f"-e FAIL_ENABLE=1 ",
        f"-e FAIL_PHASE={phase_name} ",
        f"-e FAIL_LAP={lap_idx} ",
        f"-e FAIL_WP={wp_idx} "
    ]

    # Insert flags into mission_cmd
    cmd_parts = mission_cmd.split()
    image_name = cmd_parts[-1]
    base_args = cmd_parts[:-1] # Everything except the image name
    
    # Reassemble: [docker, run, ..., -e flags, image_name]
    final_cmd_list = base_args + injection_flags + [image_name]
    final_cmd = " ".join(final_cmd_list)

    # print(final_cmd)
    
    # --- Execution Logic ---
    print(f"Starting Run 1: Execution until internal crash...")
    proc = run_cmd(final_cmd, detach=True, log_file=MISSION_CONTAINER_LOGFILE)
    exit_code = proc.wait()
    if exit_code == 0:
        print(f"Run 1 exited with code {exit_code}. Crash injeciton failed...")
        return
    print(f"Run 1 exited with code {exit_code}. Starting Recovery Run...")
    time.sleep(CRASH_RECOVERY_TIME)

    proc_recovery = run_cmd(final_cmd, detach=True, log_file=MISSION_CONTAINER_LOGFILE)
    final_exit = proc_recovery.wait()
    
    if final_exit == 0:
        print("SUCCESS: Mission recovered and completed.")
    else:
        print(f"FAILURE: Recovery run exited with code {final_exit}.")

def generate_functional_report(file_path):
    # Metrics based on Table 2
    metrics = {
        "Distinct waypoint arrivals": 0,
        "Wrapper send forwarded": 0,
        "Wrapper send suppressed (replay)": 0,
        "Wrapper recv served in replay": 0,
        "Waypoint arrivals by drone1/drone2": {1: 0, 2: 0},
        "Wrapper recv served in live": 0
    }

    with open(file_path, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or len(row) < 7: continue
            
            # Format: State, Timestamp, Mode, Direction, Function, Source, Details
            # Example row: NA, 123, LIVE, RECV, recv_match, LIVE, type=HEARTBEAT src=1
            mode = row[2]       # LIVE or REPLAY
            direction = row[3]  # RECV or SEND
            method = row[4]
            source = row[5]     # LIVE, REPLAY, or TX
            details = row[6]

            # 1. Count Replay metrics
            if mode == "REPLAY":
                if direction == "RECV":
                    metrics["Wrapper recv served in replay"] += 1
                if direction == "SEND":
                    metrics["Wrapper send suppressed (replay)"] += 1
            
            # 2. Count Forwarded sends
            if direction == "SEND" and source == "TX":
                metrics["Wrapper send forwarded"] += 1
                if method == "command_int_send":
                    try:
                        # Split by 'sys=' and then take the first part before the comma
                        sys_id_str = details.split("sys=")[1].split(",")[0]
                        sys_id = int(sys_id_str)
                        
                        # Increment arrival counts
                        metrics["Distinct waypoint arrivals"] += 1
                        if sys_id in metrics["Waypoint arrivals by drone1/drone2"]:
                            metrics["Waypoint arrivals by drone1/drone2"][sys_id] += 1
                    except (IndexError, ValueError):
                        pass

            if mode == "LIVE":
                if direction == "RECV":
                    metrics["Wrapper recv served in live"] += 1
            
    return metrics

def main():
    
    #Cleanup old logfiles
    run_cmd("rm -rf experiment_logs/container_logs/*")
    run_cmd("rm -rf experiment_logs/checkpoint/*")
    run_cmd("rm -rf experiment_logs/logfiles/*")
    run_cmd("rm -rf experiment_logs/mission_logs/*")
    run_cmd("rm -rf experiment_logs/wrapper_logs/*")


    print(f"Starting SITL UAV1 and UAV2...")
    # Outcome F1: Drone swarm availability
    sitl_cmd = (f"{SITL_COMMAND} up")
    run_cmd(sitl_cmd, True, DRONE_CONTAINER_LOGFILE)
    # run_cmd(sitl_cmd)
    print("Waiting 10s for SITL initialization...")
    time.sleep(10)

    print(f"Launching Mission Container")
    

    mission_cmd = (
        f"docker run -it --rm --name mission_container --network host --privileged "
        f"-v {CHECKPOINTS_DIR}:/mnt/checkpoints --env-file {ENV_FILE} {MISSION_IMAGE}"
    )

    parser = argparse.ArgumentParser()
    # Accept anything as a positional argument
    parser.add_argument(
        "args",
        nargs="*",
        help="Either timed crashes (60 120) or phase crash (AFTER_SEND 1 4)"
    )
    parsed = parser.parse_args()

    # 2. Branching Logic
    if not parsed.args:
        # No arguments: standard run
        print("Standard run: No failures scheduled.")
        run_mission_timewindow_crash(mission_cmd, [])
    
    elif parsed.args[0].isdigit():
        # Case A: Timed Crashes (e.g., "60 120")
        # Convert the strings to integers for the timed function
        timed_crashes = [int(x) for x in parsed.args]
        print(f"Executing Timed Crash Scenario at: {timed_crashes}s")
        run_mission_timewindow_crash(mission_cmd, timed_crashes)
    
    else:
        # Case B: Phase Crashes (e.g., "AFTER_SEND 1 4")
        # Branch to your other function (Phase, Lap, WP)
        print(f"Executing Phase-based Failure Scenario: {parsed.args}")
        run_mission_phase_crash(mission_cmd, parsed.args)

    # --- CLEANUP SECTION ---
    print("\n--- [Outcome] Evaluation complete. Cleaning up... ---")
    run_cmd(f"{SITL_COMMAND} down")

    # * Print summary
    # Define the directory path
    log_dir = Path("experiment_logs/wrapper_logs/")

    # Ensure the directory exists
    if log_dir.exists() and log_dir.is_dir():
        # Iterate through all .csv files in the directory
        for log_file in log_dir.glob("*.csv"):
            try:
                # Generate the report for the current file
                res = generate_functional_report(str(log_file))
                
                # Extract drone data
                d1 = res["Waypoint arrivals by drone1/drone2"][1]
                d2 = res["Waypoint arrivals by drone1/drone2"][2]
                drone_dist = f"{d1}/{d2}"

                # Print Header with filename
                print(f"\nREPORT FOR: {log_file.name}")
                print("=" * 50)
                print(f"{'Functional Indicator':<35} | {'Value':<10}")
                print("-" * 50)
                
                # Print metrics
                print(f"{'Distinct waypoint arrivals':<35} | {res['Distinct waypoint arrivals']}")
                print(f"{'Waypoint arrivals by drone1/drone2':<35} | {drone_dist}")
                print(f"{'Wrapper send forwarded':<35} | {res['Wrapper send forwarded']}")
                print(f"{'Wrapper send suppressed (replay)':<35} | {res['Wrapper send suppressed (replay)']}")
                print(f"{'Wrapper recv served in replay':<35} | {res['Wrapper recv served in replay']}")
                print(f"{'Wrapper recv served in live':<35} | {res['Wrapper recv served in live']}")
                print("=" * 50)
                
            except Exception as e:
                print(f"Error processing {log_file.name}: {e}")
    else:
        print(f"Directory {log_dir} not found.")
    
    

if __name__ == "__main__":
    main()