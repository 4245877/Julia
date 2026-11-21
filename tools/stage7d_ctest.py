"""Run an existing CTest suite with diagnostic destinations isolated from history."""
import argparse
from pathlib import Path
import json
import sys

sys.dont_write_bytecode = True
from diagnostic_isolation import ctest_executable, prepare, run_ctest_suites

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
    result = run_ctest_suites(ctest_executable(args.build), {'suite': args.output}, args.output, regex=args.regex)
    code = result.pop('returncodes')['suite']
    result['returncode'] = code
    (args.output/'process-results.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    if (log := args.output/'ctest.log').is_file():
        print(log.read_text(encoding='utf-8', errors='replace'), flush=True)
    if result['error']:
        print(result['error'], file=sys.stderr)
    raise SystemExit(code if code is not None and result['isolation_clean'] and not result['error'] else 1)


if __name__ == '__main__':
    main()
