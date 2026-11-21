"""Measure foreign CPU load (per process, over a window) for stage7k timing gates. Read-only.

Two snapshots of every process's accumulated CPU time (PowerShell Get-Process) `window`
seconds apart; the Idle pseudo-process and the excluded process ids are ignored. The foreign
fraction is the summed CPU-time delta divided by window x logical processors.
Usage: python tools/stage7k_idle.py [--window 20]  (prints JSON)
"""
from pathlib import Path
import argparse
import json
import os
import subprocess
import time

QUERY = "Get-Process | Select-Object Id,ProcessName,CPU | ConvertTo-Json -Compress"


def snapshot():
    child = subprocess.Popen(['powershell', '-NoProfile', '-Command', QUERY], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    out, err = child.communicate()
    assert child.returncode == 0, err
    data = json.loads(out)
    return {(p['Id'], p['ProcessName']): float(p['CPU'] or 0.) for p in data}, child.pid


def measure(window=20., exclude=()):
    first, pid1 = snapshot()
    start = time.perf_counter()
    time.sleep(window)
    second, pid2 = snapshot()
    elapsed = time.perf_counter()-start
    ignore = {os.getpid(), pid1, pid2, *exclude}
    deltas = []
    for key, value in second.items():
        pid, name = key
        if name == 'Idle' or pid in ignore or key not in first:
            continue
        delta = value-first[key]
        if delta > 0:
            deltas.append(dict(pid=pid, name=name, cpu_seconds=delta))
    deltas.sort(key=lambda x: -x['cpu_seconds'])
    cpus = os.cpu_count() or 1
    total = sum(d['cpu_seconds'] for d in deltas)
    return dict(window_s=elapsed, logical_cpus=cpus, foreign_cpu_seconds=total, foreign_fraction=total/(elapsed*cpus),
                top=deltas[:8], utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--window', type=float, default=20.)
    print(json.dumps(measure(parser.parse_args().window), indent=1))
