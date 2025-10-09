import os
import re
import csv
import datetime
import sys
import shutil
from decimal import Decimal
from collections import defaultdict

script_dir = os.path.dirname(os.path.abspath(__file__))
BASE_DIR = os.path.join(script_dir, "../log")
OUTPUT_BASE = os.path.join(script_dir, "../log")

# time patterns to match various timestamp formats
TIME_PATTERNS = [
    r"\[\d{10}(?:\.\d+)?\]:",                           # [1759720581.602674838]:
    r"\b\d{10}(?:\.\d+)?\b",                            # 1759720581.602674838
    r"\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}[.,]?\d*",     # 2025-06-06 14:16:22,341
]

# key-value pair pattern [status]:[12.76]
KEYVALUE_PATTERN = re.compile(r"\[([a-zA-Z0-9_]+)\]:\[(\-?\d+(?:\.\d+)?)\]")

# UUID raw log path match
UUID_PATTERN = re.compile(r"^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$", re.I)

def is_uuid_folder(name):
    return bool(UUID_PATTERN.match(name))

# Extract a timestamp from one log line, return unix timestamp and the rest of the line.
def extract_timestamp(line):
    line = re.sub(r'\x1b\[[0-9;]*m', '', line)
    for pattern in TIME_PATTERNS:
        match = re.search(pattern, line)
        if match:
            text = match.group(0).replace(",", ".")
            try:
                # Unix timestamp [1759720581.602674838] 1759720581.602674838
                if text.replace(".", "").isdigit() or text.strip("[]:").replace(".", "").isdigit():
                    content = line.replace(match.group(0), "")
                    text = text.strip("[]:")
                    return Decimal(text), content

                # Normal datetime
                else:
                    dt = datetime.datetime.fromisoformat(text)
                    content = line.replace(match.group(0), "")
                    return Decimal(str(dt.timestamp())), content
            except Exception:
                pass
    return None, line

# Read all .log files in a directory, extract unix timestamps, and return sorted logs.
def collect_logs_in_dir(log_dir):
    entries = []
    for root, _, files in os.walk(log_dir):
        for file in files:
            if file.endswith(".log"):
                path = os.path.join(root, file)
                with open(path, "r", encoding="utf-8", errors="ignore") as f:
                    for line in f:
                        ts, log_line = extract_timestamp(line)
                        if ts:
                            entries.append((ts, log_line))
    entries.sort(key=lambda x: x[0])  # sort by timestamp
    return entries

def extract_keyvalues(entries):
    """
    extract key-value pairs from log entries.
    return a dict:
    {
        key1: [(timestamp, value1), (timestamp, value2), ...],
        key2: [...]
    }
    """
    key_map = defaultdict(list)

    for ts, line in entries:
        for match in KEYVALUE_PATTERN.finditer(line):
            key, value = match.groups()
            try:
                key_map[key].append((
                    Decimal(str(ts)),          # high precision timestamp
                    Decimal(str(value))        # high precision value
                ))
            except Exception:
                pass

    return key_map

# Process one ROS log directory
def process_log_dir(log_dir):

    print(f"Processing {log_dir} ...")
    entries = collect_logs_in_dir(log_dir)

    if not entries:
        print(f" No valid timestamps found in {log_dir}")
        return

    # Create output directory based on start time
    start_time = entries[0][0]
    seconds = int(start_time)
    output_name = str(datetime.datetime.fromtimestamp(seconds))
    output_dir = os.path.join(OUTPUT_BASE, output_name)
    os.makedirs(output_dir, exist_ok=True)

    # merge all logs into one file with relative timestamps
    merged_path = os.path.join(output_dir, f"{output_name}.log")
    with open(merged_path, "w", encoding="utf-8") as out:
        for i, (ts, line) in enumerate(entries):
            delta = ts - start_time
            entries[i] = (delta, line)
            out.write(f"[{delta:11.6f}s] {line}\n") # timestamp output format: [   0.000000s]

    # Extract key-value pairs
    key_map = extract_keyvalues(entries)

    # collect all unique timestamps
    all_timestamps = sorted({
        ts for values in key_map.values() for ts, _ in values
    }, key=lambda x: Decimal(x))

    # initialize CSV header
    keys = sorted(key_map.keys())
    header = ["timestamp"] + keys

    # construct a lookup for values by timestamp
    value_lookup = {
        key: {ts: val for ts, val in values}
        for key, values in key_map.items()
    }

    # write to CSV
    csv_path = os.path.join(output_dir, f"{output_name}.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(header)  # write header

        for ts in all_timestamps:
            row = [str(ts)]  # first column is timestamp
            for key in keys:
                val = value_lookup[key].get(ts, "")
                row.append(str(val) if val != "" else "")
            writer.writerow(row)

    print(f"Done: {output_dir}")


def main():
    if not os.path.exists(BASE_DIR):
        print(f"Base directory not found: {BASE_DIR}")
        return

    os.makedirs(OUTPUT_BASE, exist_ok=True)

    for entry in sorted(os.listdir(BASE_DIR)):
        path = os.path.join(BASE_DIR, entry)
        if os.path.isdir(path) and is_uuid_folder(entry):
            process_log_dir(path)

if __name__ == "__main__":
    main()
