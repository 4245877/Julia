"""Aggregate stage7k final validation from plan, process results and independent certificates.

Reads only files produced by the fixed plan (and accepted stage7i/stage7j evidence for
model identity); never launches native processes. Every check either passes or is listed
under failures. OS input delivery is reported separately as unverified.
"""
from pathlib import Path
import json
import re
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def gpu_stats(folder):
    text = (Path(folder)/'gpu.txt').read_text()
    values = {}
    for name in ('render_submit_us', 'GPU_elapsed_us', 'palette_upload_us'):
        match = re.search(name+r' mean=([0-9.e+-]+) p95=([0-9.e+-]+) max=([0-9.e+-]+)', text)
        if match:
            values[name] = dict(mean=float(match.group(1)), p95=float(match.group(2)), max=float(match.group(3)))
    return dict(GL_errors_zero='GL_errors=0' in text, callbacks='callback_checks=PASS' in text, query_synchronized='query_synchronized=true' in text,
                OS_input='OS_input=not-tested' in text, gpu=text.splitlines()[0], **values)


def summarize(out, plan, results):
    out = Path(out)
    last = {}
    for r in results:
        last[r['label']] = r
    path = {label: Path(r['resolved_output']) for label, r in last.items()}
    summary = dict(schema='stage7k-final-summary-v1', plan_steps=len(plan['steps']), checks={}, failures=[],
                   OS_input='UNVERIFIED: no OS-level input delivery tool was used; internal GLFW callback fixture is separate evidence',
                   hard_real_time=False)
    def check(name, function):
        try:
            summary['checks'][name] = function()
        except Exception as error:  # every failure is reported, none is hidden
            summary['failures'].append(dict(check=name, error=repr(error)))
    def processes():
        missing = [s['label'] for s in plan['steps'] if s['label'] not in last or last[s['label']].get('returncode') != 0]
        assert not missing, missing
        interrupted = [r for r in results if r.get('status') == 'interrupted']
        return dict(steps=len(plan['steps']), all_returncode_zero=True, attempts=len(results), interrupted_attempts=len(interrupted),
                    retries=[dict(label=r['label'], attempt=r['attempt'], reason=r.get('retry_reason')) for r in results if r['attempt'] > 1])
    check('fixed_plan_processes', processes)
    def isolation():
        executed = [r for r in results if r.get('executed') and r.get('returncode') == 0]
        reused = [r for r in results if r.get('status') == 'reused-success']
        assert len(executed)+len(reused) == len(plan['steps'])
        reports = [read(r['isolation_report']) for r in executed]
        assert all(r['clean'] and not r['outside_events'] and not r['protected_inventory_changes'] and not r['monitoring_errors'] for r in reports)
        suite = read(path['full-tests']/'isolation.json')
        assert suite['clean']
        from stage7k_final import digest
        for r in reused:
            reuse = r['reuse']
            assert all(digest(p) == h for p, h in reuse['input_hashes'].items())
            assert all(digest(i['path']) == i['sha256'] for i in read(reuse['artifacts_manifest']))
        return dict(actual_write_audited_steps=len(executed), reused_complete_parity_steps=len(reused),
                    suite_clean=True, observed_events=sum(r['observed_events'] for r in reports),
                    protected_inventory_files=suite['protected_inventory_files'], outside_events=0,
                    protected_inventory_changes=0, monitoring_errors=0,
                    reused_policy='Inherited evidence is individually hashed; its original run predates the new write watcher')
    check('diagnostic_output_isolation', isolation)
    def accepted_integrity():
        data = read(out/'accepted-evidence-check.json')
        assert data['pass_all'] and all(x['payload_pass_all'] for x in data['stages'].values())
        assert data['stages']['stage7i']['verified'] == 7298
        assert data['stages']['stage7j']['verified'] == 8906
        return data
    check('accepted_evidence_preservation', accepted_integrity)
    def preflight():
        data = read(out/'isolation-preflight.json')
        assert data['pass_all']
        return data
    check('isolation_drive_case_and_hidden_writer', preflight)
    def equivalence():
        data = read(out/'postprocess/rebuild-equivalence.json')
        assert data['pass_all']
        return data
    check('unchanged_physical_result_equivalence', equivalence)
    def suites():
        counts = {}
        for variant in ('vs-release', 'cpu-release', 'cpu-no-physics'):
            text = (path['full-tests']/variant/'ctest.log').read_text(encoding='utf-8', errors='replace')
            match = re.search(r'100% tests passed, 0 tests failed out of (\d+)', text)
            assert match, variant
            counts[variant] = int(match.group(1))
        return counts
    check('full_suites', suites)
    def legacy_stage7h():
        data = read(path['legacy-stage7h']/'comparison.json')
        records = data['records'] if isinstance(data, dict) else data
        assert records and (not isinstance(data, dict) or data['pass_all'])
        return dict(exact_csv_comparisons=len(records), by_group=data.get('by_group') if isinstance(data, dict) else None,
                    rows=data.get('rows') if isinstance(data, dict) else None)
    check('legacy_stage7h_histories', legacy_stage7h)
    def accepted():
        data = read(path['accepted-regression']/'comparison.json')
        assert data['pass_all'] and data['comparisons'] > 0
        groups = {}
        for r in data['records']:
            key = r['group'].split(':')[0]
            groups[key] = groups.get(key, 0)+1
        return dict(exact_csv_comparisons=data['comparisons'], by_kind=groups, rows=data['rows'], new_only_files=data['new_only_files'])
    check('accepted_stage7i_stage7j_histories', accepted)
    def parity(label, scenarios):
        records = read(path[label]/'comparisons.json')
        assert len(records) == 9 and {(r['scenario'], r['fps']) for r in records} == {(s, f) for s in scenarios for f in (30, 60, 144)}
        gpu = {f"{r['scenario']}-{r['fps']}": gpu_stats(path[label]/f"{r['scenario']}-{r['fps']}") for r in records}
        assert all(g['GL_errors_zero'] and g['callbacks'] and g['query_synchronized'] for g in gpu.values())
        return dict(comparisons=len(records), csv_files_per_comparison=len(records[0]['counts']), gpu=gpu)
    check('new_cli_glfw_parity', lambda: parity('new-parity', ('quiet', 'pulse', 'retarget')))
    check('legacy_load_cli_glfw_parity', lambda: parity('legacy-load-parity', ('quiet', 'pulse', 'retarget')))
    def certificate(label, required=True):
        c = read(path[label]/'certificate.json')
        assert c['pass_all'] or not required, (label, c['checks'])
        brief = dict(pass_all=c['pass_all'], checks=c['checks'], alpha0=c['alpha0'], baseline_endpoint_mean_alpha=c['baseline_endpoint_mean_alpha'],
                     peaks=c['peaks'], references={k: v for k, v in c['references'].items() if k != 'switch_ticks'}, pulses=c['pulses'], lost=c['lost'],
                     commands=c['commands'], dynamics=c.get('independent_inverse_dynamics'), quadrature=c.get('independent_quadrature'))
        return brief
    for label in ('audit-quiet', 'audit-pulse', 'audit-retarget', 'audit-yaw', 'audit-step32', 'audit-soft008', 'audit-soft012'):
        check('actual_'+label.split('-', 1)[1], lambda label=label: certificate(label))
    def qp():
        c = read(path['independent-qp']/'certificate.json')
        assert c['pass_all'] and c['problems'] >= 300
        return dict(problems=c['problems'], solutions=c['solutions_checked'], maxima=c['maxima'], scenarios=c['scenarios'], corpus_sha256=c['corpus_sha256'])
    check('independent_QP_KKT', qp)
    def causal(label):
        c = read(path[label]/'certificate.json')
        assert c['checks']['joint_actual_goals'] and len(c['causality']) == 2
        return c['causality']
    check('causality_quiet', lambda: causal('causal-quiet'))
    check('causality_pulse', lambda: causal('causal-pulse'))
    def negatives():
        c = read(path['negatives-audit']/'certificate.json')
        assert c['pass_all']
        return {k: v for k, v in c.items() if k != 'pass_all'}
    check('negatives', negatives)
    def feasibility():
        c = read(path['feasibility-current']/'certificate.json')
        assert c['all_feasible']
        return dict(all_feasible=True, max_minimum_peak_utilization=c['max_minimum_peak_utilization'], alpha0=c['alpha0'],
                    cases=[dict(label=x['label'], tick=x['tick'], alpha=x['requested_alpha'], range=[x['alpha_min'], x['alpha_max']],
                                minimum_peak_utilization=x['minimum_peak_utilization'], com_offset_mm=[1000*v for v in x['pose_com_offset_from_initial_xz_m']])
                           for x in c['cases']])
    check('static_feasibility_current', feasibility)
    def performance():
        from stage7i_performance import analyze
        runs = {}
        for label in ('performance-1', 'performance-2'):
            result = analyze(path[label])
            assert result['metadata']['warmup_ticks'] == 480
            assert result['metadata']['measured_ticks'] == result['tick_us']['count'] >= 2400
            result['target_met'] = result['tick_us']['p95'] < 5000
            runs[label] = result
        assert all(r['target_met'] for r in runs.values())
        perf = [r for r in results if r['kind'] == 'performance' and r.get('returncode') == 0]
        assert [r['label'] for r in perf] == ['performance-1', 'performance-2']
        assert perf[0]['completed_utc'] < perf[1]['utc']
        assert all(r.get('executed') and r['isolation_clean'] and r['idle_gate']['passed'] and
                   not r['performance_gate']['competing_native_or_build'] and not r['performance_gate']['other_python'] for r in perf)
        return runs
    check('performance', performance)
    def model():
        nominal = path['new-parity']/'quiet-headless'/'model.xml'
        accepted = Path('build/diagnostics/stage7j/final-validation-01/parity/quiet-headless/model.xml')
        stage7i = Path('build/diagnostics/stage7i/final-validation-01/parity/optimized-shift-pulse-headless/model.xml')
        assert nominal.read_bytes() == accepted.read_bytes() == stage7i.read_bytes()
        return dict(identical_to_stage7j_and_stage7i=True, bytes=len(nominal.read_bytes()))
    check('unchanged_nominal_model', model)
    def frozen():
        from stage7k_final import manifest_check
        return dict(frozen_runtime_files=manifest_check(plan['manifest']))
    check('frozen_runtime_unchanged', frozen)
    summary['pass_all'] = not summary['failures']
    return summary
