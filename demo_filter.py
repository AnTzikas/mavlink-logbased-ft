#!/usr/bin/env python3
"""
demo_filter.py  —  MAVLink wrapper log filter for demo presentation
Usage:  tail -f <evidence_log_path> | python3 demo_filter.py

Log line format (from _wrapper_log):
    {run_id},{ts_us},{mode},{direction},{api},{outcome},{summary},{extra}

Example lines:
    NA,1772717596266124,LIVE,SEND,command_int_send,TX,CMD_INT(sys=1 cmd=192 frame=6),
    NA,1772717595924271,LIVE,RECV,recv_match,LIVE,type=GLOBAL_POSITION_INT src=2,

Output rhythm:
    [SEND] [LIVE] TX  command_int_send  CMD_INT(sys=1 cmd=192 frame=6)
      ↳ recv: GLOBAL_POSITION_INT src=1 ×14   GLOBAL_POSITION_INT src=2 ×12
"""

import sys
import re
from collections import defaultdict

# ── ANSI colors ────────────────────────────────────────────────────────────
RESET      = "\033[0m"
BOLD       = "\033[1m"
DIM        = "\033[2m"

# SEND — most prominent
SEND_LIVE      = f"{BOLD}\033[92m"    # Bright green   → LIVE send
SEND_REPLAY    = f"{BOLD}\033[96m"    # Bright cyan    → REPLAY send

# Mode tags
TAG_LIVE       = f"\033[92m"          # Green
TAG_REPLAY     = f"\033[96m"          # Cyan

# RECV summary — subtle, not competing with sends
RECV_LABEL     = f"{DIM}\033[37m"     # Dim grey       → "↳ recv:"
RECV_TYPE      = "\033[97m"           # Bright white   → message type name
RECV_COUNT     = "\033[33m"           # Yellow         → ×count

# Outcome tag colors
OUTCOME_TX         = f"\033[92m"      # Green
OUTCOME_REPLAY     = f"\033[96m"      # Cyan
OUTCOME_SUPPRESSED = f"\033[91m"      # Red
OUTCOME_INTENT     = f"\033[95m"      # Magenta

# ── CSV field indices (0-based) ────────────────────────────────────────────
F_RUN_ID    = 0
F_TS        = 1
F_MODE      = 2
F_DIRECTION = 3
F_API       = 4
F_OUTCOME   = 5
F_SUMMARY   = 6
F_EXTRA     = 7

# ── RECV summary type extraction ───────────────────────────────────────────
RE_TYPE_SRC = re.compile(r"type=(\S+)\s+src=(\d+)")

# ── RECV mode colors in summary ────────────────────────────────────────────
RECV_LIVE_COUNT   = f"\033[92m"   # Green  → live count
RECV_REPLAY_COUNT = f"\033[96m"   # Cyan   → replay count

# ── State ──────────────────────────────────────────────────────────────────
# { "GLOBAL_POSITION_INT src=1": {"LIVE": 3, "REPLAY": 11} }
recv_buffer = defaultdict(lambda: defaultdict(int))


def flush_recv_summary():
    """Print buffered recv counts (with LIVE/REPLAY breakdown) then clear."""
    if not recv_buffer:
        return

    # Sort by total count descending
    sorted_keys = sorted(
        recv_buffer.items(),
        key=lambda x: -(x[1]["LIVE"] + x[1]["REPLAY"])
    )

    parts = []
    for msg_key, modes in sorted_keys:
        live_n   = modes["LIVE"]
        replay_n = modes["REPLAY"]

        # Build the mode breakdown — skip zeros
        breakdown = []
        if live_n > 0:
            breakdown.append(f"{RECV_LIVE_COUNT}live x{live_n}{RESET}")
        if replay_n > 0:
            breakdown.append(f"{RECV_REPLAY_COUNT}replay x{replay_n}{RESET}")

        parts.append(
            f"{RECV_TYPE}{msg_key}{RESET} "
            f"({' / '.join(breakdown)})"
        )

    # print(f"  {RECV_LABEL}-> recv:{RESET}  " + "   ".join(parts))
    print(f"  {RECV_LABEL}-> recv:{RESET}")
    for part in parts:
        print(f"      {part}")
    recv_buffer.clear()


def color_outcome(outcome):
    if outcome == "TX":
        return f"{OUTCOME_TX}{outcome}{RESET}"
    elif outcome == "REPLAY":
        return f"{OUTCOME_REPLAY}{outcome}{RESET}"
    elif outcome == "SUPPRESSED":
        return f"{OUTCOME_SUPPRESSED}{outcome}{RESET}"
    elif outcome == "INTENT":
        return f"{OUTCOME_INTENT}{outcome}{RESET}"
    return outcome


def format_send(mode, api, outcome, summary):
    color = SEND_LIVE if mode == "LIVE" else SEND_REPLAY
    tag   = f"{TAG_LIVE}[LIVE]{RESET}" if mode == "LIVE" else f"{TAG_REPLAY}[REPLAY]{RESET}"
    return (
        f"{color}[SEND]{RESET} {tag} "
        f"{color_outcome(outcome)}  "
        f"{BOLD}{api}{RESET}  "
        f"{summary}"
    )


# ── Main loop ──────────────────────────────────────────────────────────────
try:
    for raw_line in sys.stdin:
        line = raw_line.strip()
        if not line:
            continue

        # Split into exactly 8 fields (extra may contain commas — limit split)
        fields = line.split(",", 7)
        if len(fields) < 7:
            # Malformed or non-log line — pass through
            flush_recv_summary()
            print(line)
            continue

        mode      = fields[F_MODE]
        direction = fields[F_DIRECTION]
        api       = fields[F_API]
        outcome   = fields[F_OUTCOME]
        summary   = fields[F_SUMMARY].strip()

        if direction == "SEND":
            flush_recv_summary()
            print(format_send(mode, api, outcome, summary))

        elif direction == "RECV":
            m = RE_TYPE_SRC.search(summary)
            if m:
                key = f"{m.group(1)} src={m.group(2)}"
                recv_buffer[key][mode] += 1

        else:
            # Any other direction — pass through as-is
            flush_recv_summary()
            print(line)

except KeyboardInterrupt:
    flush_recv_summary()
    sys.exit(0)