"""Run the frozen desktop/CPU/OFF CTest suites into a new stage7k output (current layout).

Equivalent of tools/stage7h_ctest.py + tools/stage7d_ctest.py for outputs that live under
build/diagnostics/stage7k. For each build the generated tests/CTestTestfile.cmake is copied
with every `<build>/verification` path (current layout) and any historical
`<project>/build/diagnostics/` path redirected into <output>/<variant>; the copy is rejected
unless, after removing the destination itself, no historical diagnostics path and no
`<build>/verification` path remains (case-insensitive: Windows drive letters may differ).
redirect.json records the substitution counts. Suites run in parallel with --parallel.
"""
from pathlib import Path
import argparse
import json
import os
import re
import subprocess
import sys
import time

sys.dont_write_bytecode = True
from diagnostic_isolation import OutputGuard, ctest_executable, prepare, stop_children, tracked_popen, close_process_job

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--parallel', action='store_true')
    parser.add_argument('--regex', help='Smoke tests only: ctest -R pattern')
    parser.add_argument('--prepare-only', action='store_true', help='Validate redirection without starting CTest')
    parser.add_argument('--allowed-output-root', type=Path, help='Whole new validation directory, when called by a driver')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    ctest = ctest_executable(ROOT/'build/vs-release')
    env = os.environ.copy(); env['PATH'] = str(Path(ctest).parent)+os.pathsep+env['PATH']
    env['PYTHONDONTWRITEBYTECODE'] = '1'
    redirect, work, results = {}, [], {}
    start = time.perf_counter()
    for variant in ('vs-release', 'cpu-release', 'cpu-no-physics'):
        destination = args.output/variant
        destination.mkdir()
        redirect[variant] = prepare(Path('build')/variant, destination)
    (args.output/'redirect.json').write_text(json.dumps(redirect, indent=2), encoding='utf-8')
    if args.prepare_only:
        return
    guard = OutputGuard(args.allowed_output_root or args.output, args.output/'isolation.json').start()
    stopped = []
    try:
        for variant in redirect:
            destination = args.output/variant
            log = (destination/'ctest.log').open('x', encoding='utf-8')
            command = [ctest, '--test-dir', str(destination.resolve()), '--output-on-failure', *(['-R', args.regex] if args.regex else [])]
            process = tracked_popen(command, stdout=log, stderr=subprocess.STDOUT, env=env)
            work.append((variant, process, log))
            if not args.parallel:
                while process.poll() is None and not guard.violated:
                    time.sleep(.1)
                if guard.violated:
                    stopped = stop_children([p for _, p, _ in work]); break
                results[variant] = process.wait(); log.close()
        while any(p.poll() is None for _, p, _ in work) and not guard.violated:
            time.sleep(.1)
        if guard.violated:
            stopped += stop_children([p for _, p, _ in work])
    finally:
        for variant, process, log in work:
            results[variant] = process.wait(); log.close()
            close_process_job(process)
        isolation = guard.finish()
    (args.output/'process-results.json').write_text(json.dumps(dict(returncodes=results, wall_seconds=time.perf_counter()-start,
        parallel=args.parallel, isolation_clean=isolation['clean'], stopped_process_trees=stopped), indent=2), encoding='utf-8')
    print(results, flush=True)
    if any(results.values()) or not isolation['clean']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
