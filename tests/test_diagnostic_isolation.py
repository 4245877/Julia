"""Safe persistent isolation fixtures; no full CTest matrix or physical experiment."""
from pathlib import Path
import argparse
import json
import os
import subprocess
import sys
import time
from unittest import mock

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from diagnostic_isolation import OutputGuard, prepare, run_ctest_suites, stop_children, tracked_popen


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    out = args.output.resolve(); out.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[1]
    records = {}
    for drive in ('C:', 'c:'):
        cwd = drive+root.as_posix()[2:]
        destination = out/('upper' if drive == 'C:' else 'lower')
        child = subprocess.run([sys.executable, '-B', 'tools/stage7k_ctest.py', '--prepare-only', '--output', str(destination)],
                               cwd=cwd, capture_output=True, text=True)
        assert child.returncode == 0, child.stderr
        records[drive] = json.loads((destination/'redirect.json').read_text())
        assert {v: r['validated_output_arguments'] for v, r in records[drive].items()} == {
            'vs-release': 66, 'cpu-release': 29, 'cpu-no-physics': 14}
    # The original historical layout, with the destination itself under diagnostics,
    # must also be rewritten exactly once for either drive case.
    historical = 'add_test([=[fixture]=] "C:/Julia/build/cpu-release/tests/julia_test_core.exe" "C:/Julia/build/diagnostics/stage5/cpu")\n'
    for drive in ('C:', 'c:'):
        dest = out/('historical-upper' if drive == 'C:' else 'historical-lower')
        records[str(dest.name)] = prepare(root/'build/cpu-release', dest, root=Path(drive+root.as_posix()[2:]), source_text=historical)
        assert (dest/'CTestTestfile.cmake').read_text().count('/artifacts/stage5/cpu') == 1
    for label, text in [('zero', 'add_test([=[pure]=] "C:/Julia/build/cpu-release/tests/julia_test_core.exe")\n'),
                        ('foreign', historical+'add_test([=[bad]=] "C:/Julia/build/cpu-release/tests/julia_test_core.exe" "C:/another/unsafe.csv")\n'),
                        ('foreign_exe', historical+'add_test([=[bad]=] "C:/Julia/build/cpu-release/tests/julia_test_core.exe" "C:/another/unsafe.exe")\n')]:
        try:
            prepare(root/'build/cpu-release', out/label, source_text=text)
        except RuntimeError as error:
            records[label] = dict(rejected=True, error=str(error))
        else:
            raise AssertionError(f'{label} did not fail closed')
    # A hidden writer runs only inside a synthetic workspace. It never touches real
    # historical diagnostics. Keep the fixture and the invalidated observation.
    fixture = out/'hidden-writer-fixture'; fixture.mkdir()
    allowed = fixture/'allowed'; allowed.mkdir()
    rogue = fixture/'compiled-in-output.csv'; rogue.write_text('old\n')
    guard = OutputGuard(allowed, allowed/'isolation.json', root=fixture).start()
    code = "import pathlib,time; pathlib.Path('compiled-in-output.csv').write_text('new\\n'); time.sleep(20)"
    child = tracked_popen([sys.executable, '-B', '-c', code], cwd=fixture)
    deadline = time.monotonic()+5
    while not guard.violated and time.monotonic() < deadline:
        time.sleep(.05)
    stopped = stop_children([child]); child.wait()
    assert stopped and all(x['stopped'] for x in stopped)
    result = guard.finish()
    assert not result['clean'] and result['outside_events'] and not result['monitoring_errors'], result
    records['hidden_writer'] = dict(detected=True, stopped_process_trees=stopped, outside_events=len(result['outside_events']))
    clean_root = out/'clean-fixture'; clean_root.mkdir()
    clean_out = clean_root/'allowed'; clean_out.mkdir()
    guard = OutputGuard(clean_out, clean_out/'isolation.json', root=clean_root).start()
    for n in range(100):
        path = clean_out/f'checkpoint-{n}.txt'
        path.write_text('permitted\n')
        path.unlink()
    result = guard.finish()
    assert result['clean'] and result['observed_events'] > 0, result
    records['permitted_writer'] = dict(clean=True, events=result['observed_events'])
    partial = out/'partial-suites'
    suites = {name: partial/name for name in ('started', 'unavailable')}
    for directory in suites.values():
        directory.mkdir(parents=True)
    child = None

    def launch(_command, **kwargs):
        nonlocal child
        if child is not None:
            raise FileNotFoundError('fixture: second suite could not start')
        child = tracked_popen([sys.executable, '-B', '-c', 'import time; time.sleep(20)'], **kwargs)
        return child

    with mock.patch('diagnostic_isolation.tracked_popen', side_effect=launch):
        result = run_ctest_suites(sys.executable, suites, partial, parallel=True)
    assert result['isolation_clean'] and result['error'] and result['stopped_process_trees'], result
    assert result['returncodes'] == {'started': 1, 'unavailable': None}, result
    records['partial_suites'] = result
    (out/'certificate.json').write_text(json.dumps(dict(pass_all=True, checks=records), indent=2), encoding='utf-8')
    print('PASS: both drive cases, current/historical layouts, incomplete redirect rejection, actual hidden writer detection and scoped stop, clean writer')


if __name__ == '__main__':
    main()
