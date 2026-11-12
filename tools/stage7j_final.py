"""Sequential, fixed post-freeze stage7j validation. Never tunes or rebuilds.

Run only after tuning is finished and source/binaries have been frozen. The final
performance process runs after all other subprocesses have completed. Reports and
the final delivery/evidence manifests are completed afterwards.
"""
from pathlib import Path
import argparse
import csv
import datetime
import hashlib
import json
import re
import subprocess
import sys
import time

from stage7j_manifest import digest
from verify_stage7c import compare


def json_write(path, value):
    path.write_text(json.dumps(value, indent=2), encoding='utf-8')


def manifest_check(path):
    entries = json.loads(path.read_text())
    bad = [item['path'] for item in entries if not Path(item['path']).is_file()
           or digest(Path(item['path'])) != item['sha256']]
    assert not bad, ('Frozen runtime changed', bad)
    return len(entries)


def accepted_history(tests, output):
    """Stage7i optimized histories, in addition to stage7h checked by old helper."""
    records = []
    for variant in ('cpu-release', 'vs-release'):
        old = Path('build/diagnostics/stage7i/final-validation-01/tests')/variant/'artifacts/stage7i/optimized-suite'
        current = tests/variant/'artifacts/stage7i/optimized-suite'
        assert old.is_dir() and current.is_dir()
        for path in sorted(old.rglob('*.csv')):
            if 'performance' in path.name:
                continue
            rel = path.relative_to(old)
            count, fields = compare(path, current/rel)
            records.append(dict(variant=variant, file=rel.as_posix(), rows=count, fields=fields))
    assert records
    json_write(output, records)
    return len(records)


def read_csv(path):
    with path.open(newline='') as stream:
        return list(csv.DictReader(stream))


def causal_summary(output):
    from stage7j_audit import compare as independent_compare
    result = {}
    for label in ('quiet', 'pulse'):
        enabled = output/'parity'/f'{label}-headless'
        disabled = output/f'{label}-task-off'
        assert (enabled/'model.xml').read_bytes() == (disabled/'model.xml').read_bytes()
        on = read_csv(enabled/'load-v1.csv'); off = read_csv(disabled/'load-v1.csv')
        assert len(on) == len(off)
        # Commands/reference must remain identical despite different measured loads.
        fields = ('tick', 'serial', 'command', 'reference', 'reference_v', 'reference_a', 'alpha0')
        assert all([a[k] for k in fields] == [b[k] for k in fields] for a, b in zip(on, off))
        a = read_csv(output/f'independent-{label}'/'reconstructed-v1.csv')
        b = read_csv(output/f'independent-{label}-task-off'/'reconstructed-v1.csv')
        by_tick = {int(r['tick']): r for r in b}
        intervals = []
        for serial in (1, 2, 3):
            records = [r for r in a if int(r['serial']) == serial][-240:]
            assert len(records) == 240
            valid = all(r['actual'] not in ('', 'None') and by_tick[int(r['tick'])]['actual'] not in ('', 'None') for r in records)
            intervals.append(dict(serial=serial, start_tick=int(records[0]['tick']), end_tick=int(records[-1]['tick']),
                                  statistic='arithmetic mean of last240 completed-tick samples; no hold claim', samples=240,
                                  endpoint_span_s=(int(records[-1]['tick'])-int(records[0]['tick']))/120.,
                                  valid=valid, command=float(records[-1]['command']),
                                  enabled_mean_alpha=sum(float(r['actual']) for r in records)/240 if valid else None,
                                  disabled_mean_alpha=sum(float(by_tick[int(r['tick'])]['actual']) for r in records)/240 if valid else None,
                                  causal_alpha_difference=sum(float(r['actual'])-float(by_tick[int(r['tick'])]['actual']) for r in records)/240 if valid else None))
        result[label] = dict(independent_comparison=independent_compare(enabled, disabled, output/f'independent-{label}', output/f'independent-{label}-task-off'),
                             identical_model=True, identical_commands_and_reference=True, completed_tick_intervals=intervals)
    return result


def negative_summary(output):
    import math
    from stage7j_audit import audit
    result = {}
    for label in ('disable', 'support-loss', 'weak'):
        folder = output/f'negative-{label}'
        stand = read_csv(folder/'stand-v1.csv')
        terminals = [r for r in stand if int(r['tick']) > 0 and r['phase'] in ('lost', 'disabled')]
        assert terminals, label
        terminal = terminals[0]; tick = int(terminal['tick'])
        assert terminal['evaluations'] == stand[-1]['evaluations'], label
        assert all(float(r['applied_Nm']) == 0 for r in read_csv(folder/'joints.csv') if int(r['tick']) > tick), label
        states = {int(r['tick']): r for r in read_csv(folder/'state.csv')}
        subsequent = [r for n, r in sorted(states.items()) if n > tick]
        assert len(subsequent) >= 24, ('Too few post-terminal completed ticks', label)
        assert all(int(r['epoch']) == int(states[tick]['epoch'])
                   and int(r['world_steps'])-int(states[tick]['world_steps']) == (int(r['tick'])-tick)*16
                   and float(r['time_s']) > float(states[tick]['time_s']) for r in subsequent), label
        pose_fields = ('root_x_m', 'root_y_m', 'root_z_m', 'qw', 'qx', 'qy', 'qz')
        physical_change = max(abs(float(r[field])-float(states[tick][field]))
                              for r in subsequent for field in pose_fields)
        assert physical_change > 1e-8, ('Physical state stopped after termination', label)
        # Reuse only the independent raw reader, never Julia's controller. It
        # reconstructs each foot-floor force/moment and validity from raw points,
        # then checks actual alpha/invalid exports. Falling after termination may
        # intentionally fail stand geometry guards and no_lost; those are reported.
        destination = output/f'independent-negative-{label}'
        destination.mkdir(exist_ok=False)
        independent = audit(folder, destination, include_dynamics=False, goal_mode='none')
        required = ('raw_reconstruction', 'actual_friction', 'moments', 'reference')
        assert all(independent['checks'][key] for key in required), (label, independent['checks'])
        json_write(destination/'certificate.json', independent)
        loads = read_csv(folder/'load-v1.csv')
        invalid_after = [int(r['tick']) for r in loads if int(r['tick']) >= tick and r['actual_valid'] == '0']
        assert all(math.isnan(float(r['actual'])) for r in loads if r['actual_valid'] == '0'), label
        if label == 'support-loss':
            assert invalid_after, 'Support loss must produce independently invalid actual alpha'
            assert set(invalid_after).issubset(independent['invalid_actual_ticks'])
        result[label] = dict(tick=tick, phase=terminal['phase'], reason=terminal['reason'], subsequent_qp=0, subsequent_active_torque=0,
                             proven_infeasible=terminal['reason'] == 'infeasible',
                             completed_post_terminal_ticks=len(subsequent), physical_pose_change=physical_change,
                             no_reset=True, continued_16_substeps_per_tick=True,
                             invalid_actual_post_terminal_ticks=len(invalid_after),
                             first_invalid_post_terminal_tick=min(invalid_after) if invalid_after else None,
                             independent_raw_force_alpha=True, independent_peaks=independent['peaks'])
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--manifest', type=Path, default=Path('build/diagnostics/stage7j/frozen-runtime.json'))
    parser.add_argument('--legacy-parity', action='store_true', help='Also rerun all18 old real CLI/GLFW comparisons')
    parser.add_argument('--plan-only', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    py = sys.executable
    cpu = str(Path('build/cpu-release/apps/julia_body_physics/julia_body_physics.exe'))
    newtest = str(Path('build/cpu-release/tests/julia_test_stage7j.exe'))
    steps = []
    def add(label, command): steps.append(dict(label=label, command=command))
    add('full-tests', [py, 'tools/stage7h_ctest.py', '--output', str(args.output/'tests'), '--parallel'])
    add('legacy-stage7h', [py, 'tools/stage7i_regression.py', '--tests', str(args.output/'tests'), '--output', str(args.output/'legacy-stage7h')])
    add('new-parity', [py, 'tools/verify_stage7j.py', '--output', str(args.output/'parity')])
    if args.legacy_parity:
        add('legacy-parity', [py, 'tools/verify_stage7i.py', '--output', str(args.output/'legacy-parity')])
    for label in ('quiet', 'pulse', 'retarget'):
        command = [py, 'tools/stage7j_audit.py', '--run', str(args.output/'parity'/f'{label}-headless'), '--output', str(args.output/f'independent-{label}'), '--require-goals', '--plot']
        if label == 'retarget': command.extend(['--goal-mode', 'final'])
        add('audit-'+label, command)
    add('independent-qp', [py, 'tools/stage7i_qp_audit.py', '--corpus', str(args.output/'tests/cpu-release/artifacts/stage7j/corpus-suite/corpus.bin'), '--output', str(args.output/'independent-qp')])
    base = [cpu, '--case', 'load-shift', '--bounded-solver', 'optimized']
    for label, extra in (
        ('yaw', ['--yaw', '.73', '--load-pulses', 'on']),
        ('step32', ['--substeps', '32']),
        ('soft008', ['--contact-time', '.008']),
        ('soft012', ['--contact-time', '.012']),
        ('quiet-task-off', ['--load-task', 'off']),
        ('pulse-task-off', ['--load-task', 'off', '--load-pulses', 'on']),
    ):
        add('run-'+label, [*base, *extra, '--output', str(args.output/label)])
        command = [py, 'tools/stage7j_audit.py', '--run', str(args.output/label), '--output', str(args.output/f'independent-{label}'), '--plot', '--require-goals']
        if label in ('quiet-task-off', 'pulse-task-off'): command.extend(['--goal-mode', 'none'])
        add('audit-'+label, command)
    for label, extra in (
        ('disable', ['--stand-event', 'disable']),
        ('support-loss', ['--stand-event', 'lift']),
        ('weak', ['--effort-scale', '.5']),
    ):
        add('negative-'+label, [*base, '--ticks', '960', *extra, '--output', str(args.output/f'negative-{label}')])
    # All other native runs/readers finish before this CPU measurement starts.
    add('performance', [newtest, str(args.output/'performance'), 'perf'])
    json_write(args.output/'plan.json', dict(manifest=str(args.manifest), sequential=True, steps=steps,
                                            postprocessing=['accepted-stage7i-histories', 'causal-raw-alpha', 'negative-zero-torque', 'performance-floor-quantiles', 'frozen-runtime-check'],
                                            evidence_manifest='Generate only after completing final reports; not inside this validation driver'))
    if args.plan_only:
        print('Saved fixed validation plan; no native processes launched')
        return
    frozen_count = manifest_check(args.manifest)
    results = []
    for index, step in enumerate(steps):
        manifest_check(args.manifest)
        start = time.perf_counter()
        entry = dict(index=index, **step, utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
        executable = Path(step['command'][0])
        entry['executable_sha256'] = hashlib.sha256(executable.read_bytes()).hexdigest() if executable.is_file() else None
        print('START', index, step['label'], flush=True)
        with (args.output/f'process-{index:02}-{step["label"]}.log').open('w', encoding='utf-8') as log:
            run = subprocess.run(step['command'], stdout=log, stderr=subprocess.STDOUT)
        entry.update(returncode=run.returncode, wall_seconds=time.perf_counter()-start)
        results.append(entry); json_write(args.output/'process-results.json', results)
        manifest_check(args.manifest)
        print('END', index, run.returncode, flush=True)
    summary = dict(frozen_runtime_files=frozen_count, subprocesses=results, checks={}, failures=[])
    def check(label, function):
        try: summary['checks'][label] = function()
        except Exception as error: summary['failures'].append(dict(check=label, error=repr(error)))
    check('stage7i_accepted_exact_csv', lambda: accepted_history(args.output/'tests', args.output/'accepted-stage7i-histories.json'))
    check('causality', lambda: causal_summary(args.output))
    check('negatives', lambda: negative_summary(args.output))
    def performance():
        from stage7i_performance import analyze
        result = analyze(args.output/'performance')
        result['target_met'] = result['tick_us']['p95'] < 5000
        result['hard_real_time'] = False
        json_write(args.output/'performance-summary.json', result)
        return result
    check('performance', performance)
    def suites():
        result = {}
        for variant in ('vs-release', 'cpu-release', 'cpu-no-physics'):
            text = (args.output/'tests'/variant/'ctest.log').read_text()
            match = re.search(r'100% tests passed, 0 tests failed out of (\d+)', text)
            assert match, variant
            result[variant] = int(match.group(1))
        return result
    check('full_suites', suites)
    def retarget():
        result = json.loads((args.output/'independent-retarget/certificate.json').read_text())
        assert not result['lost'] and len(result['commands']) == 4
        assert all(result['checks'][key] for key in ('raw_reconstruction', 'actual_friction', 'moments', 'reference', 'guards', 'no_lost'))
        assert result['commands'][2]['first_two_seconds'] and result['commands'][3]['first_two_seconds']
        return dict(cancel=result['commands'][2], return_to_alpha0=result['commands'][3])
    check('retarget_cancel_return', retarget)
    def unchanged_model():
        baseline = Path('build/diagnostics/stage7i/final-validation-01/parity/optimized-shift-pulse-headless/model.xml')
        nominal = args.output/'parity/quiet-headless/model.xml'
        assert baseline.read_bytes() == nominal.read_bytes()
        return dict(identical=True, sha256=digest(nominal))
    check('accepted_nominal_model', unchanged_model)
    summary['all_processes_passed'] = all(result['returncode'] == 0 for result in results)
    summary['pass_all'] = summary['all_processes_passed'] and not summary['failures'] and summary['checks'].get('performance', {}).get('target_met', False)
    summary['OS_input'] = 'unverified; internal callbacks and real GLFW are separate evidence'
    manifest_check(args.manifest)
    json_write(args.output/'summary.json', summary)
    print('Fixed validation finished; pass_all=', summary['pass_all'], flush=True)
    if not summary['pass_all']: raise SystemExit(1)


if __name__ == '__main__':
    main()
