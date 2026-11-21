"""Run an existing CTest suite with diagnostic destinations isolated from history."""
import argparse
from pathlib import Path
import subprocess
import json
import os
import sys
import time

sys.dont_write_bytecode = True
from diagnostic_isolation import OutputGuard, ctest_executable, prepare, stop_children, tracked_popen, close_process_job

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--regex')
    parser.add_argument('--prepare-only', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    record = prepare(args.build, args.output)
    (args.output/'redirect.json').write_text(json.dumps(record, indent=2), encoding='utf-8')
    if args.prepare_only:
        return
    command = [ctest_executable(args.build), '--test-dir', str(args.output.resolve()), '--output-on-failure']
    if args.regex:
        command += ['-R', args.regex]
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
    guard = OutputGuard(args.output, args.output/'isolation.json').start()
    stopped = []
    try:
        with (args.output/'ctest.log').open('x', encoding='utf-8') as log:
            child = tracked_popen(command, stdout=log, stderr=subprocess.STDOUT, env=env)
            while child.poll() is None and not guard.violated:
                time.sleep(.1)
            if guard.violated:
                stopped = stop_children([child])
            code = child.wait()
            close_process_job(child)
    finally:
        isolation = guard.finish()
    (args.output/'process-results.json').write_text(json.dumps(dict(returncode=code, isolation_clean=isolation['clean'],
        stopped_process_trees=stopped), indent=2), encoding='utf-8')
    print((args.output/'ctest.log').read_text(encoding='utf-8'), flush=True)
    raise SystemExit(code if isolation['clean'] else 1)


if __name__ == '__main__':
    main()
