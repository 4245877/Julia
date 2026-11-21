"""Resumable fixed post-freeze stage7k validation. Never tunes or rebuilds.

  python tools/stage7k_final.py init --output <new dir> --manifest <frozen runtime manifest>
  python tools/stage7k_final.py run --output <dir> [--through N]
  python tools/stage7k_final.py retry --output <dir> --label <step> --reason "<why>"
  python tools/stage7k_final.py summarize --output <dir>

plan.json fixes every command, its output path and order before any native process.
Each step writes started-NN.json before launch and appends a result (exit code, wall
time, executable SHA256, UTC start/end) to process-results.json. The frozen runtime
manifest is checked before and after every step. Successful steps are never rerun.
A started step without a result is an interruption: its partial output is preserved
in place and recorded, and the step reruns into a new '-rK' output path; dependants
use the resolved path. A completed failing step is rerun only by an explicit 'retry'
with a recorded reason. Performance steps run last, alone, after a process gate.
"""
from pathlib import Path
import argparse
import datetime
import hashlib
import json
import os
import re
import subprocess
import sys
import time

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))
from diagnostic_isolation import OutputGuard, stop_children, tracked_popen, close_process_job


def digest(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            value.update(block)
    return value.hexdigest()


def now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def write(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False), encoding='utf-8')


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def manifest_check(path):
    entries = read(path)
    bad = [item['path'] for item in entries if not Path(item['path']).is_file() or digest(item['path']) != item['sha256']]
    assert not bad, ('Frozen runtime changed', bad)
    return len(entries)


def steps_template():
    py = sys.executable
    cpu = str(Path('build/cpu-release/apps/julia_body_physics/julia_body_physics.exe').resolve())
    test = str(Path('build/cpu-release/tests/julia_test_stage7k.exe').resolve())
    coord = [cpu, '--case', 'coord-shift', '--bounded-solver', 'optimized']
    s = []
    def add(label, command, output=None, kind='native'):
        s.append(dict(label=label, command=command, output=output, kind=kind))
    # Close the isolation incident with one full matrix before other acceptance work.
    add('full-tests', [py, 'tools/stage7k_ctest.py', '--output', '@out:full-tests', '--parallel',
                       '--allowed-output-root', '@validation-root'], 'tests', 'suites')
    add('new-parity', [py, 'tools/verify_stage7k.py', '--output', '@out:new-parity'], 'parity', 'native-parity')
    add('legacy-load-parity', [py, 'tools/verify_stage7j.py', '--output', '@out:legacy-load-parity'], 'legacy-load-parity', 'native-parity')
    for label, extra in (('shift-pulse', ['com-shift', '--ticks', '6480', '--bounded-solver', 'optimized', '--com-pulses', 'on']),
                         ('retarget', ['com-retarget', '--ticks', '2160', '--bounded-solver', 'optimized']),
                         ('yaw', ['com-shift', '--ticks', '6480', '--bounded-solver', 'optimized', '--yaw', '.73', '--com-pulses', 'on'])):
        add(f'legacy-com-{label}', [cpu, '--case', *extra, '--output', f'@out:legacy-com-{label}'], f'legacy-com-cli/optimized-{label}-headless')
    for label, mode in (('quiet', 'cycle'), ('pulse', 'cycle'), ('retarget', 'retarget')):
        add(f'audit-{label}', [py, 'tools/stage7k_audit.py', '--run', f'@sub:new-parity/{label}-headless', '--output', f'@out:audit-{label}',
                               '--mode', mode, '--require-goals', '--plot'], f'independent-{label}', 'reader')
    for label, extra in (('yaw', ['--yaw', '.73', '--coord-pulses', 'on']), ('step32', ['--substeps', '32']),
                         ('soft008', ['--contact-time', '.008']), ('soft012', ['--contact-time', '.012'])):
        add(f'run-{label}', [*coord, *extra, '--output', f'@out:run-{label}'], label)
        add(f'audit-{label}', [py, 'tools/stage7k_audit.py', '--run', f'@out:run-{label}', '--output', f'@out:audit-{label}',
                               '--mode', 'cycle', '--require-goals', '--plot'], f'independent-{label}', 'reader')
    for scenario, extra in (('quiet', []), ('pulse', ['--coord-pulses', 'on'])):
        add(f'run-comonly-{scenario}', [*coord, *extra, '--load-task', 'off', '--output', f'@out:run-comonly-{scenario}'], f'comonly-{scenario}')
        add(f'run-loadonly-{scenario}', [*coord, *extra, '--com-task', 'off', '--output', f'@out:run-loadonly-{scenario}'], f'loadonly-{scenario}')
        add(f'causal-{scenario}', [py, 'tools/stage7k_audit.py', '--run', f'@sub:new-parity/{scenario}-headless', '--output', f'@out:causal-{scenario}',
                                   '--mode', 'cycle', '--skip-dynamics', '--plot', '--compare', f'com-only=@out:run-comonly-{scenario}',
                                   '--compare', f'load-only=@out:run-loadonly-{scenario}'], f'causal-{scenario}', 'reader')
    for label, extra in (('disable', ['--stand-event', 'disable']), ('support-loss', ['--stand-event', 'lift']), ('weak', ['--effort-scale', '.5'])):
        add(f'negative-{label}', [*coord, '--ticks', '960', *extra, '--output', f'@out:negative-{label}'], f'negative-{label}')
    add('negatives-audit', [py, 'tools/stage7k_negatives.py', '--disable', '@out:negative-disable', '--support-loss', '@out:negative-support-loss',
                            '--weak', '@out:negative-weak', '--output', '@out:negatives-audit'], 'negatives-audit', 'reader')
    add('feasibility-current', [py, 'tools/stage7k_feasibility.py', '--alpha0-from-run', '@sub:new-parity/quiet-headless', '--output', '@out:feasibility-current',
                                '--case', 'A_x+3_alpha+0.05=@sub:new-parity/quiet-headless::2399::0.05',
                                '--case', 'B_x-3_alpha-0.05=@sub:new-parity/quiet-headless::5039::-0.05',
                                '--case', 'C_z+3_alpha+0.05=@sub:new-parity/quiet-headless::6959::0.05',
                                '--case', 'return_0_alpha0=@sub:new-parity/quiet-headless::8399::0',
                                '--case', 'retarget_z+3_alpha-0.05=@sub:new-parity/retarget-headless::2159::-0.05',
                                '--case', 'retarget_cancel_stop=@sub:new-parity/retarget-headless::5999::-0.05'], 'feasibility-current', 'reader')
    add('legacy-stage7h', [py, 'tools/stage7k_regression.py', '--stage7h-tests', '@out:full-tests', '--output', '@out:legacy-stage7h'], 'legacy-stage7h', 'reader')
    add('accepted-regression', [py, 'tools/stage7k_regression.py', '--tests', '@out:full-tests', '--load-parity', '@out:legacy-load-parity',
                                '--com-run', 'shift-pulse=@out:legacy-com-shift-pulse', '--com-run', 'retarget=@out:legacy-com-retarget',
                                '--com-run', 'yaw=@out:legacy-com-yaw', '--output', '@out:accepted-regression'], 'accepted-regression', 'reader')
    add('independent-qp', [py, 'tools/stage7i_qp_audit.py', '--corpus', '@sub:full-tests/cpu-release/verification/artifacts/stage7k/corpus-suite/corpus.bin',
                           '--output', '@out:independent-qp'], 'independent-qp', 'reader')
    for index in (1, 2):
        add(f'performance-{index}', [test, f'@out:performance-{index}', 'perf'], f'performance-{index}', 'performance')
    return s


def resolve(plan, results, token):
    """'@out:label' -> resolved output dir of that step; '@sub:label/x' -> dir/x; '@parent:label' -> parent."""
    def output(label):
        retry = [r for r in results if r['label'] == label and r.get('returncode') == 0]
        if retry:
            return retry[-1]['resolved_output']
        step = next(x for x in plan['steps'] if x['label'] == label)
        attempts = [r for r in results if r['label'] == label]
        return step['output_path'] if not attempts else f"{step['output_path']}-r{len(attempts)}"
    if token == '@validation-root':
        return plan['output_root']
    if not isinstance(token, str) or '@' not in token:
        return token
    def one(text):
        kind, rest = text[1:].split(':', 1)
        if kind == 'out':
            return output(rest)
        if kind == 'parent':
            return str(Path(output(rest)).parent)
        label, sub = rest.split('/', 1)
        return str(Path(output(label))/sub)
    if token.startswith('@'):
        return one(token)
    head, tail = token.split('=@', 1) if '=@' in token else (None, None)
    if head is not None:
        if '::' in tail:
            first, others = tail.split('::', 1)
            return f'{head}={one("@"+first)}::{others}'
        return f'{head}={one("@"+tail)}'
    return token


def gate():
    query = subprocess.run(['powershell', '-NoProfile', '-Command',
                            "Get-Process | Where-Object { $_.ProcessName -match 'julia|cmake|ctest|ninja|^cl$|^link$|MSBuild' } | Select-Object Id,ProcessName,Path | ConvertTo-Json"],
                           capture_output=True, text=True)
    assert query.returncode == 0, query.stderr
    processes = json.loads(query.stdout) if query.stdout.strip() else []
    if isinstance(processes, dict):
        processes = [processes]
    python = subprocess.run(['powershell', '-NoProfile', '-Command',
                             f"Get-CimInstance Win32_Process -Filter \"Name='python.exe'\" -ErrorAction Stop | Where-Object {{ $_.ProcessId -ne {os.getpid()} -and $_.ParentProcessId -ne {os.getpid()} }} | Select-Object ProcessId,CommandLine | ConvertTo-Json"],
                            capture_output=True, text=True)
    if python.returncode == 0:
        others = json.loads(python.stdout) if python.stdout.strip() else []
        if isinstance(others, dict):
            others = [others]
    else:
        # Without command lines every other python.exe counts as competing (conservative).
        listing = subprocess.run(['powershell', '-NoProfile', '-Command',
                                  f"Get-Process python -ErrorAction SilentlyContinue | Where-Object {{ $_.Id -ne {os.getpid()} }} | Select-Object Id,ProcessName | ConvertTo-Json"],
                                 capture_output=True, text=True)
        assert listing.returncode == 0, ('Could not enumerate competing Python processes', listing.stderr)
        others = json.loads(listing.stdout) if listing.stdout.strip() else []
        if isinstance(others, dict):
            others = [others]
    return processes, others, python.returncode, python.stderr.strip()


def idle_gate(out, step, threshold=.10, wait=60., limit=12*3600.):
    """Block until foreign CPU load over a 20 s window is <= threshold; record every sample."""
    from stage7k_idle import measure
    samples, start = [], time.perf_counter()
    while True:
        sample = measure(20.)
        samples.append(sample)
        write(out/f"idle-gate-{step['index']:02}-{step['label']}.json", dict(threshold=threshold, samples=samples))
        if sample['foreign_fraction'] <= threshold:
            return dict(passed=True, threshold=threshold, samples=len(samples), waited_s=time.perf_counter()-start, last=sample)
        if time.perf_counter()-start > limit:
            raise SystemExit(f"Idle gate timeout before {step['label']}")
        top = sample['top'][0]['name'] if sample['top'] else '-'
        print('IDLE-WAIT', step['index'], step['label'], round(sample['foreign_fraction'], 3), top, flush=True)
        time.sleep(wait)


def run_step(out, plan, results, step, reason=None):
    attempts = [r for r in results if r['label'] == step['label']]
    resolved_output = step['output_path'] if not attempts else f"{step['output_path']}-r{len(attempts)}"
    command = [resolve(plan, results, token) for token in step['command']]
    assert not any(isinstance(t, str) and t.startswith('@') for t in command), ('Unresolved template', command)
    destination = Path(resolved_output)
    assert not destination.exists(), ('Output must be new', destination)
    assert digest(plan['manifest']) == plan['manifest_sha256'], 'Frozen manifest itself changed'
    manifest_check(plan['manifest'])
    entry = dict(index=step['index'], label=step['label'], kind=step['kind'], attempt=len(attempts)+1, command=command,
                 resolved_output=resolved_output, utc=now(), executable=command[0],
                 executable_sha256=digest(command[0]) if Path(command[0]).is_file() else None, retry_reason=reason)
    if step['kind'] in ('suites', 'performance'):
        entry['idle_gate'] = idle_gate(out, step)
    if step['kind'] == 'performance':
        processes, others, code, error = gate()
        entry['performance_gate'] = dict(competing_native_or_build=processes, other_python=others, python_query_returncode=code, python_query_error=error)
        assert not processes and not others, ('Competing own workloads before performance', processes, others)
    # An idle gate can wait for hours. Recheck immediately before launch, so files
    # changed during that wait cannot be executed under the earlier freeze check.
    assert digest(plan['manifest']) == plan['manifest_sha256'], 'Frozen manifest itself changed during gate'
    manifest_check(plan['manifest'])
    if step.get('reuse'):
        reuse = step['reuse']
        assert step['kind'] == 'native-parity', 'Only complete deterministic parity evidence can be inherited'
        for item in read(reuse['artifacts_manifest']):
            assert Path(item['path']).stat().st_size == item['bytes'] and digest(item['path']) == item['sha256'], item['path']
        for path, expected in reuse['input_hashes'].items():
            assert digest(path) == expected, ('Inherited evidence input changed', path)
        entry.update(returncode=0, wall_seconds=0, completed_utc=now(), status='reused-success',
                     executed=False, resolved_output=reuse['source_result']['resolved_output'], reuse=reuse)
        results.append(entry)
        write(out/'process-results.json', results)
        print('REUSED', step['index'], step['label'], reuse['source_validation'], flush=True)
        return 0
    started = out/f"started-{step['index']:02}-a{entry['attempt']}.json"
    write(started, entry)
    log = out/f"process-{step['index']:02}-{step['label']}-a{entry['attempt']}.log"
    print('START', step['index'], step['label'], 'attempt', entry['attempt'], flush=True)
    guard = OutputGuard(out, out/f"isolation-{step['index']:02}-a{entry['attempt']}.json").start()
    start = time.perf_counter()
    stopped, launch_error, code = [], None, None
    try:
        with log.open('x', encoding='utf-8') as stream:
            child = tracked_popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                     env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1'))
            while child.poll() is None and not guard.violated:
                time.sleep(.1)
            if guard.violated:
                stopped = stop_children([child])
            code = child.wait()
            close_process_job(child)
    except OSError as error:
        launch_error = repr(error)
    finally:
        isolation = guard.finish()
    entry.update(returncode=code if isolation['clean'] and launch_error is None else 1,
                 child_returncode=code, wall_seconds=time.perf_counter()-start, completed_utc=now(), log=str(log),
                 isolation_report=str(guard.report), isolation_clean=isolation['clean'], stopped_process_trees=stopped,
                 launch_error=launch_error, executed=True)
    results.append(entry)
    write(out/'process-results.json', results)
    manifest_check(plan['manifest'])
    print('END', step['index'], step['label'], entry['returncode'], round(entry['wall_seconds'], 1), flush=True)
    return entry['returncode']


def interrupted(out, results):
    """Started files without results are preserved interruptions."""
    done = {(r['index'], r['attempt']) for r in results}
    found = []
    for path in sorted(out.glob('started-*.json')):
        entry = read(path)
        if (entry['index'], entry['attempt']) not in done:
            found.append(entry)
    return found


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('init', 'run', 'retry', 'summarize'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--manifest', type=Path)
    parser.add_argument('--through', type=int)
    parser.add_argument('--label')
    parser.add_argument('--reason')
    parser.add_argument('--reuse-success', action='append', default=[], help='init: LABEL=existing validation directory; complete deterministic parity only')
    parser.add_argument('--dry-run', action='store_true', help='run: print resolved commands only; launches nothing, writes nothing')
    args = parser.parse_args()
    out = args.output
    if args.action == 'run' and args.dry_run:
        plan = read(out/'plan.json'); results = read(out/'process-results.json')
        for step in plan['steps']:
            command = [resolve(plan, results, token) for token in step['command']]
            assert not any(isinstance(t, str) and t.startswith('@') for t in command), command
            print(step['index'], step['label'], step['kind'], ' '.join(command), flush=True)
        return
    if args.action == 'init':
        out.mkdir(parents=True, exist_ok=False)
        assert args.manifest and args.manifest.is_file()
        frozen = manifest_check(args.manifest)
        steps = steps_template()
        for index, step in enumerate(steps):
            step['index'] = index
            step['output_path'] = str(out/step.pop('output'))
        for spec in args.reuse_success:
            label, folder = spec.split('=', 1)
            source = Path(folder)
            original_plan = read(source/'plan.json')
            source_results = read(source/'process-results.json')
            result = next(r for r in reversed(source_results) if r['label'] == label and r.get('returncode') == 0)
            step = next(s for s in steps if s['label'] == label)
            assert step['kind'] == 'native-parity'
            original_manifest = read(original_plan['manifest'])
            # Every native executable/DLL and physical source must be identical. Tool-only
            # changes are explicitly retained in the new manifest change map.
            native = [i for i in original_manifest if i['path'].endswith(('.exe', '.dll')) or
                      i['path'].startswith(('libs/physics/', 'apps/julia_body_physics/'))]
            assert native
            assert all(digest(i['path']) == i['sha256'] for i in native), 'Inherited native/physics version differs'
            artifacts = [dict(path=p.as_posix(), bytes=p.stat().st_size, sha256=digest(p))
                         for p in sorted(Path(result['resolved_output']).rglob('*')) if p.is_file()]
            assert artifacts
            artifact_manifest = out/f'inherited-{label}-files.json'
            write(artifact_manifest, artifacts)
            inputs = [source/'plan.json', source/'process-results.json', Path(original_plan['manifest']), artifact_manifest]
            step['reuse'] = dict(source_validation=str(source), source_result=result, artifacts_manifest=str(artifact_manifest),
                                input_hashes={str(p): digest(p) for p in inputs},
                                identical_native_and_physics_files=len(native), executed_by_new_plan=False)
        plan = dict(schema='stage7k-final-v2', utc=now(), output_root=str(out.resolve()), manifest=str(args.manifest), manifest_sha256=digest(args.manifest), frozen_files=frozen,
                    sequential=True, steps=steps, policy=__doc__.strip(),
                    postprocessing=['suite counts', 'accepted histories', 'independent actual certificates', 'KKT', 'causality', 'negatives',
                                    'static LP', 'GPU/callback parity', 'performance two runs p95<5ms', 'frozen runtime check'])
        write(out/'plan.json', plan)
        write(out/'process-results.json', [])
        print('INIT', len(steps), 'steps; frozen files', frozen, flush=True)
        return
    plan = read(out/'plan.json')
    assert digest(plan['manifest']) == plan['manifest_sha256'], 'Frozen manifest itself changed'
    # Byte-range locks are released by Windows when a driver exits; a stale file does
    # not block continuation, while a live second driver is rejected before launch.
    lock = None
    if args.action in ('run', 'retry'):
        import msvcrt
        lock = (out/'driver.lock').open('a+b')
        if lock.tell() == 0:
            lock.write(b'0'); lock.flush()
        lock.seek(0)
        try:
            msvcrt.locking(lock.fileno(), msvcrt.LK_NBLCK, 1)
        except OSError:
            raise SystemExit('Another live driver owns this validation plan')
    results = read(out/'process-results.json')
    pending = interrupted(out, results)
    if pending:
        ledger = out/'interruptions.json'
        known = read(ledger) if ledger.exists() else []
        for entry in pending:
            if all((k['index'], k['attempt']) != (entry['index'], entry['attempt']) for k in known):
                known.append(dict(entry, status='interrupted; partial output preserved in place; never counted as success', recorded_utc=now()))
                results.append(dict(entry, returncode=None, status='interrupted', completed_utc=None))
        write(ledger, known)
        write(out/'process-results.json', results)
    if args.action == 'retry':
        assert args.label and args.reason, 'Explicit failing step label and reason required'
        step = next(x for x in plan['steps'] if x['label'] == args.label)
        previous = [r for r in results if r['label'] == args.label]
        assert previous and previous[-1].get('returncode') not in (0,), 'Only a failed/interrupted step may be retried'
        code = run_step(out, plan, results, step, reason=args.reason)
        raise SystemExit(code)
    if args.action == 'run':
        for step in plan['steps']:
            if args.through is not None and step['index'] > args.through:
                break
            attempts = [r for r in results if r['label'] == step['label']]
            if any(r.get('returncode') == 0 for r in attempts):
                continue
            if attempts and attempts[-1].get('status') != 'interrupted':
                raise SystemExit(f"Step {step['label']} failed; investigate, then use 'retry' with a reason")
            if step['kind'] == 'performance':
                remaining = [x['label'] for x in plan['steps'] if x['kind'] != 'performance' and not any(r['label'] == x['label'] and r.get('returncode') == 0 for r in results)]
                assert not remaining, ('All other steps must finish before performance', remaining)
            code = run_step(out, plan, results, step)
            if code:
                raise SystemExit(code)
        return
    from stage7k_summary import summarize
    summary = summarize(out, plan, results)
    write(out/'summary.json', summary)
    print('SUMMARY pass_all=', summary['pass_all'], 'failures=', summary['failures'], flush=True)
    if not summary['pass_all']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
