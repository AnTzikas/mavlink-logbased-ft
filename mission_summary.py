import subprocess
import time
import os
import argparse
import csv
from pathlib import Path

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
                print(f"{'Waypoint arrivals by veh1/veh2':<35} | {drone_dist}")
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