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
import sys

sys.dont_write_bytecode = True
from diagnostic_isolation import ctest_executable, prepare, run_ctest_suites

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
    redirect, destinations = {}, {}
    for variant in ('vs-release', 'cpu-release', 'cpu-no-physics'):
        destination = args.output/variant
        destination.mkdir()
        redirect[variant] = prepare(Path('build')/variant, destination)
        destinations[variant] = destination
    (args.output/'redirect.json').write_text(json.dumps(redirect, indent=2), encoding='utf-8')
    if args.prepare_only:
        return
    result = run_ctest_suites(ctest_executable(ROOT/'build/vs-release'), destinations, args.output,
                              regex=args.regex, parallel=args.parallel, allowed_output=args.allowed_output_root)
    (args.output/'process-results.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(result['returncodes'], flush=True)
    if result['error']:
        print(result['error'], file=sys.stderr)
    if any(code != 0 for code in result['returncodes'].values()) or not result['isolation_clean'] or result['error']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
