import csv
import collections

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

# Run the analysis
res = generate_functional_report("container_logs/wrapper_logs/wrapper_log_1772051679.csv")

# # Print the Table
# print("\n" + "="*45)
# print(f"{'Functional Indicator':<35} | {'Value':<7}")
# print("-" * 45)
# print(f"{'Distinct waypoint arrivals':<35} | {res['Distinct waypoint arrivals']}")
# print(f"{'Wrapper send forwarded':<35} | {res['Wrapper send forwarded']}")
# print(f"{'Wrapper send suppressed (replay)':<35} | {res['Wrapper send suppressed (replay)']}")
# print(f"{'Wrapper recv served in replay':<35} | {res['Wrapper recv served in replay']}")
# print("="*45)

# Format the drone distribution string (e.g., "11/7")
d1 = res["Waypoint arrivals by drone1/drone2"][1]
d2 = res["Waypoint arrivals by drone1/drone2"][2]
drone_dist = f"{d1}/{d2}"

print("\n" + "="*50)
print(f"{'Functional Indicator':<35} | {'Value':<10}")
print("-" * 50)
print(f"{'Distinct waypoint arrivals':<35} | {res['Distinct waypoint arrivals']}")
print(f"{'Waypoint arrivals by drone1/drone2':<35} | {drone_dist}")
print(f"{'Wrapper send forwarded':<35} | {res['Wrapper send forwarded']}")
print(f"{'Wrapper send suppressed (replay)':<35} | {res['Wrapper send suppressed (replay)']}")
print(f"{'Wrapper recv served in replay':<35} | {res['Wrapper recv served in replay']}")
print(f"{'Wrapper recv served in live':<35} | {res['Wrapper recv served in live']}")
print("="*50)