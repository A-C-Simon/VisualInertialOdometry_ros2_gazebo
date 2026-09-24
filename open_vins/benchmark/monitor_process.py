#!/usr/bin/env python3
"""Sample CPU time and RSS for one Linux process."""

import argparse
import json
import os
import time
from pathlib import Path


def sample(pid: int):
    stat = Path(f"/proc/{pid}/stat").read_text().split()
    status = Path(f"/proc/{pid}/status").read_text().splitlines()
    rss_kib = int(next(line.split()[1] for line in status if line.startswith("VmRSS:")))
    return int(stat[13]) + int(stat[14]), rss_kib


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("pid", type=int)
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    ticks_per_second = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
    started = time.monotonic()
    samples = []
    try:
        first_ticks, first_rss = sample(args.pid)
        samples.append(first_rss)
    except (FileNotFoundError, ProcessLookupError):
        raise SystemExit(f"process {args.pid} is not running")

    last_ticks = first_ticks
    while True:
        time.sleep(args.interval)
        try:
            last_ticks, rss = sample(args.pid)
            samples.append(rss)
        except (FileNotFoundError, ProcessLookupError, StopIteration):
            break

    elapsed = time.monotonic() - started
    cpu_seconds = (last_ticks - first_ticks) / ticks_per_second
    result = {
        "pid": args.pid,
        "elapsed_seconds": round(elapsed, 3),
        "cpu_seconds": round(cpu_seconds, 3),
        "mean_cpu_percent": round(100.0 * cpu_seconds / elapsed, 2),
        "mean_rss_mib": round(sum(samples) / len(samples) / 1024.0, 2),
        "peak_rss_mib": round(max(samples) / 1024.0, 2),
        "samples": len(samples),
    }
    output = json.dumps(result, indent=2)
    if args.output:
        args.output.write_text(output + "\n")
    print(output)


if __name__ == "__main__":
    main()
