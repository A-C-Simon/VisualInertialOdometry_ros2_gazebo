#!/usr/bin/env python3
"""Run one owned process group and persist wait4 CPU/RSS even on Ctrl-C."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

output = Path(sys.argv[1])
command = sys.argv[2:]
started = time.monotonic()
child = None
stopping = False
stage = 0

def stop(signum, frame):
    global stopping
    if child is not None and not stopping:
        stopping = True
        try: os.killpg(child.pid, signal.SIGINT)
        except ProcessLookupError: pass
        signal.alarm(8)

def escalate(signum, frame):
    global stage
    if child is not None:
        try: os.killpg(child.pid, signal.SIGTERM if stage == 0 else signal.SIGKILL)
        except ProcessLookupError: pass
        if stage == 0:
            stage = 1
            signal.alarm(3)

signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
signal.signal(signal.SIGALRM, escalate)
try:
    child = subprocess.Popen(command, start_new_session=True)
    _, status, usage = os.wait4(child.pid, 0)
    child.returncode = os.waitstatus_to_exitcode(status)
    signal.alarm(0)
    elapsed = time.monotonic() - started
    data = dict(command=command, wall_seconds=elapsed, cpu_seconds=usage.ru_utime+usage.ru_stime,
                cpu_percent_one_core=100*(usage.ru_utime+usage.ru_stime)/elapsed,
                peak_rss_mib=usage.ru_maxrss/1024, exit_code=child.returncode,
                forced_shutdown=bool(stage))
except Exception as error:
    data = dict(command=command, error=str(error))
output.write_text(json.dumps(data, indent=2)+'\n')
sys.exit(0 if stopping else data.get('exit_code', 1))
