"""Read-only aggregate of frozen stage7j evidence; never launches native code.

Run after the fixed final driver, independent geometry readers, and performance
measurement have finished. This reader independently rechecks immutable inputs,
exact legacy histories, raw timing order statistics, and actual holds after the
reference has completely stopped. It writes only its new --output directory.
"""
from pathlib import Path
import argparse
import csv
import difflib
import hashlib
import json
import math
import re
import xml.etree.ElementTree as ET


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def rows(path):
    with path.open(newline='', encoding='utf-8-sig') as stream:
        yield from csv.DictReader(stream)


def write(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False), encoding='utf-8')


def exact_csv(left, right):
    """Exact text comparison of deterministic values; excludes *_us only."""
    with left.open(newline='', encoding='utf-8-sig') as ls, right.open(newline='', encoding='utf-8-sig') as rs:
        a, b = csv.DictReader(ls), csv.DictReader(rs)
        assert a.fieldnames == b.fieldnames, ('CSV schema', left, right)
        fields = [f for f in a.fieldnames if not f.endswith('_us')]
        count = 0
        while True:
            x, y = next(a, None), next(b, None)
            assert (x is None) == (y is None), ('CSV length', right, count)
            if x is None:
                break
            assert None not in x and None not in y and all(v is not None for v in x.values()) and all(v is not None for v in y.values())
            assert all(x[f] == y[f] for f in fields), ('CSV value', right, count, [f for f in fields if x[f] != y[f]])
            count += 1
    return dict(rows=count, deterministic_fields=len(fields))


def frozen(root):
    manifest = root/'frozen-runtime.json'
    data = read(manifest)
    assert len(data) == 340, len(data)
    bad = [r['path'] for r in data if not Path(r['path']).is_file()
           or Path(r['path']).stat().st_size != r['bytes'] or digest(Path(r['path'])) != r['sha256']]
    assert not bad, bad
    baseline = read(root/'baseline/summary.json')
    assert baseline['runtime-manifest.json']['matched'] == baseline['runtime-manifest.json']['total'] == 325
    assert baseline['evidence-manifest.json']['matched'] == baseline['evidence-manifest.json']['total'] == 7298
    assert baseline['snapshot_files'] == 336
    return dict(files=len(data), manifest_sha256=digest(manifest), baseline=baseline)


def suites(validation):
    result = {}
    for variant, expected in (('vs-release', 74), ('cpu-release', 35), ('cpu-no-physics', 22)):
        log = validation/'tests'/variant/'ctest.log'
        text = log.read_text(encoding='utf-8-sig')
        match = re.search(r'100% tests passed, 0 tests failed out of (\d+)', text)
        assert match and int(match.group(1)) == expected, variant
        cache = Path('build')/variant/'CMakeCache.txt'
        config = cache.read_text()
        assert 'CMAKE_BUILD_TYPE:STRING=Release' in config and 'JULIA_WARNINGS_AS_ERRORS:BOOL=ON' in config
        seconds = re.search(r'Total Test time \(real\) =\s*([\d.]+)', text)
        result[variant] = dict(passed=expected, seconds=float(seconds.group(1)), log_sha256=digest(log))
    return result


def histories(validation, output):
    records = []
    counts = dict(accepted_stage7h=0, accepted_stage7i_optimized=0, current_reference_optimized=0)
    for variant in ('cpu-release', 'vs-release'):
        old = Path('build/diagnostics/stage7h/final-tests-03')/variant/'artifacts'
        current = validation/'tests'/variant/'artifacts'
        for p in sorted(old.rglob('*.csv')):
            rel = p.relative_to(old)
            if not rel.parts[0].startswith('stage7') or 'performance' in p.name:
                continue
            records.append(dict(group='accepted_stage7h', variant=variant, path=rel.as_posix(), **exact_csv(p, current/rel)))
            counts['accepted_stage7h'] += 1
        old = Path('build/diagnostics/stage7i/final-validation-01/tests')/variant/'artifacts/stage7i/optimized-suite'
        for p in sorted(old.rglob('*.csv')):
            if 'performance' in p.name:
                continue
            rel = p.relative_to(old)
            records.append(dict(group='accepted_stage7i_optimized', variant=variant, path=rel.as_posix(), **exact_csv(p, current/'stage7i/optimized-suite'/rel)))
            counts['accepted_stage7i_optimized'] += 1
        reference = current/'stage7h/cpu'
        for p in sorted(reference.rglob('*.csv')):
            if 'performance' in p.name:
                continue
            rel = p.relative_to(reference)
            records.append(dict(group='current_reference_optimized', variant=variant, path=rel.as_posix(), **exact_csv(p, current/'stage7i/optimized-suite'/rel)))
            counts['current_reference_optimized'] += 1
    assert counts['accepted_stage7h'] == 916 and counts['accepted_stage7i_optimized'] == 168, counts
    assert counts['current_reference_optimized'] >= 168, counts
    inherited = read(validation/'legacy-stage7h/comparison.json')
    direct = read(validation/'accepted-stage7i-histories.json')
    assert len(inherited) == counts['accepted_stage7h']+counts['current_reference_optimized']
    assert len(direct) == counts['accepted_stage7i_optimized']
    write(output/'exact-histories.json', records)
    return dict(**counts, accepted_physical_csv_total=1084, all_comparisons=len(records), timing_fields_excluded='*_us only; performance workloads separately measured')


def intervals(data, mask):
    best = []; current = []; first = None
    for r, good in zip(data, mask):
        if good:
            current = current+[r] if not current or int(r['tick']) == int(current[-1]['tick'])+1 else [r]
            if len(current) > len(best):
                best = current[:]
            if len(current) >= 241 and first is None:
                first = dict(start_tick=int(current[-241]['tick']), end_tick=int(current[-1]['tick']), duration_s=2.)
        else:
            current = []
    if not best:
        return dict(completed_ticks=0, duration_s=0., first_two_seconds=None)
    return dict(start_tick=int(best[0]['tick']), end_tick=int(best[-1]['tick']),
                start_s=float(best[0]['time_s']), end_s=float(best[-1]['time_s']),
                completed_ticks=len(best), duration_s=(int(best[-1]['tick'])-int(best[0]['tick']))/120.,
                first_two_seconds=first,
                actual_alpha_range=[min(float(r['actual']) for r in best), max(float(r['actual']) for r in best)],
                minimum_left_N=min(float(r['left_N']) for r in best), minimum_right_N=min(float(r['right_N']) for r in best),
                maximum_command_error=max(float(r['command_error']) for r in best),
                maximum_com_error_m=max(float(r['com_error_m']) for r in best), maximum_speed_m_s=max(float(r['speed_m_s']) for r in best))


def actual_run(validation, label, run, geometry, output):
    certificate = read(validation/f'independent-{label}/certificate.json')
    required = ('raw_reconstruction', 'actual_friction', 'moments', 'reference', 'guards', 'no_lost', 'pulse_recovery')
    assert all(certificate['checks'][key] for key in required), (label, certificate['checks'])
    assert not certificate['lost']
    if label != 'retarget':
        assert certificate['pass_all'] and certificate['checks']['selected_actual_goals']
    for name, value in certificate['raw_sha256'].items():
        assert digest(run/name) == value, (label, name)
    for name in ('independent_inverse_dynamics', 'independent_quadrature'):
        assert name in certificate, (label, name)
    dynamic = certificate['independent_inverse_dynamics']
    assert dynamic['root_residual_N_Nm'] < 1e-6 and dynamic['torque_reconstruction_Nm'] < 1e-10
    assert dynamic['contact_acceleration_residual'] < 1e-6 and dynamic['corner_friction_violation_N'] < 1e-7
    quad = certificate['independent_quadrature']
    assert quad['integral_error'] < 1e-10 and quad['residual_reconstruction_error'] < 1e-10
    geo = read(geometry/'certificate.json')
    assert geo['pass_all'] and all(geo['checks'].values()), (label, geo['checks'])
    assert geo['mujoco_version'] == '3.3.7' and geo['original_mass_kg'] == 55.
    assert geo['model_sha256'] == digest(run/'model.xml')
    for name, value in geo['files_sha256'].items():
        assert digest(run/name) == value, (label, 'geometry input', name)
    raw = {int(r['tick']): r for r in rows(run/'load-v1.csv')}
    data = list(rows(validation/f'independent-{label}/reconstructed-v1.csv'))
    assert len(raw) == len(data) == geo['samples_including_initial']
    selected = (3, 4) if label == 'retarget' else (1, 2, 3)
    assert sorted({int(r['serial']) for r in data}-{0}) == ([1, 2, 3, 4] if label == 'retarget' else [1, 2, 3])
    goals = []
    for serial in selected:
        samples = [r for r in data if int(r['serial']) == serial]
        assert samples
        meets = [r['meets_all'] == 'True' for r in samples]
        stopped = []
        for r in samples:
            source = raw[int(r['tick'])]
            stopped.append(abs(float(source['reference'])-float(source['command'])) <= 1e-12
                           and abs(float(source['reference_v'])) <= 1e-12 and abs(float(source['reference_a'])) <= 1e-12)
        before = intervals(samples, meets)
        post = intervals(samples, [a and b for a, b in zip(meets, stopped)])
        assert post['duration_s'] >= 2. and post['first_two_seconds'], (label, serial, post)
        final = samples[-1]
        assert final['meets_all'] == 'True', (label, serial, final)
        expected = None if label == 'retarget' and serial == 3 else (0. if label == 'retarget' or serial == 3 else .05 if serial == 1 else -.05)
        if expected is not None:
            assert abs(float(final['command'])-float(final['alpha0'])-expected) < 1e-12
        goals.append(dict(serial=serial, alpha0=float(final['alpha0']), command=float(final['command']),
                          final_actual=float(final['actual']), final_error=float(final['command_error']),
                          final_left_N=float(final['left_N']), final_right_N=float(final['right_N']),
                          final_com_error_m=float(final['com_error_m']), final_speed_m_s=float(final['speed_m_s']),
                          raw_criteria_hold=before, after_reference_stop_hold=post))
    write(output/f'actual-goals-{label}.json', goals)
    return dict(actual_goals=goals, peaks=certificate['peaks'], dynamics=dynamic, quadrature=quad,
                cop_margins=certificate['cop_margins'], geometry=dict(peaks=geo['peaks'], feet=geo['feet']),
                pulses=certificate['pulses'], baseline=certificate['baseline'],
                certificate_sha256=digest(validation/f'independent-{label}/certificate.json'), geometry_certificate_sha256=digest(geometry/'certificate.json'))


def qp_certificate(validation):
    path = validation/'independent-qp/certificate.json'; c = read(path)
    assert c['pass_all'] and c['problems'] >= 480 and c['solutions_checked'] == 2*c['problems'] and len(c['scenarios']) == 2
    maximum = c['maxima']
    for key, threshold in dict(objective_relative=1e-8, primal_difference=1e-5, equality=1e-6,
                               root_dynamic_scaled_100=1e-6, contact_acceleration=1e-6, inequality=1e-7,
                               projected_stationarity=1e-5, complementarity=1e-5, dual_negative=1e-8).items():
        assert maximum[key] <= threshold, (key, maximum[key])
    corpus = validation/'tests/cpu-release/artifacts/stage7j/corpus-suite/corpus.bin'
    assert c['corpus_sha256'] == digest(corpus) and corpus.open('rb').read(8) == b'J7IQP001'
    return c


def performance(validation):
    path = validation/'performance'
    samples = list(rows(path/'performance.csv')); calls = list(rows(path/'calls.csv'))
    assert len(samples) >= 2400 and len(calls) == 4*len(samples)
    assert all(int(r['evaluations']) == 4 and int(r['physical_steps']) == 16 and r['phase'] == 'holding' for r in samples)
    grouped = {}
    for r in calls:
        grouped.setdefault(int(r['tick']), []).append(int(r['call']))
    assert all(v == [0, 1, 2, 3] for v in grouped.values()) and len(grouped) == len(samples)
    values = sorted(float(r['tick_us']) for r in samples)
    result = dict(count=len(values), mean=sum(values)/len(values), p95=values[len(values)*95//100],
                  p99=values[len(values)*99//100], max=values[-1], over_8333_333=sum(x > 1e6/120 for x in values),
                  hard_real_time=False, no_own_concurrent_work='Final driver is sequential; performance last, confirmed by process results')
    assert result['p95'] < 5000
    inherited = read(validation/'performance-summary.json')['tick_us']
    assert result['p95'] == inherited['p95'] and result['p99'] == inherited['p99'] and result['max'] == inherited['max']
    assert abs(result['mean']-inherited['mean']) < 1e-8
    processes = read(validation/'process-results.json')
    assert processes[-1]['label'] == 'performance' and all(r['returncode'] == 0 for r in processes)
    result['metadata'] = read(path/'metadata.json')
    return result


def model_and_sources(root, validation, dependencies):
    baseline = root/'baseline/snapshot'
    old_model = Path('build/diagnostics/stage7i/final-validation-01/parity/optimized-shift-pulse-headless/model.xml')
    model = validation/'parity/quiet-headless/model.xml'
    assert model.read_bytes() == old_model.read_bytes()
    xml = ET.parse(model).getroot()
    bodies = list(xml.find('worldbody').iter('body')); inertials = list(xml.find('worldbody').iter('inertial'))
    assert len(bodies) == 20 and abs(sum(float(x.attrib['mass']) for x in inertials)-55.) < 1e-12
    assert len(list(xml.find('worldbody').iter('freejoint'))) == 1 and xml.find('worldbody').find('.//freejoint').attrib['name'] == 'root'
    actuators = list(xml.find('actuator')); assert len(actuators) == 41 and all(a.attrib.get('joint') != 'root' for a in actuators)
    equality = xml.find('equality'); assert equality is None or len(list(equality)) == 0
    assert abs(float(xml.find('option').attrib['timestep'])-1/1920) < 1e-15
    for file in ('headless-dependencies.txt', 'mujoco-dependencies.txt'):
        text = (dependencies/file).read_text(encoding='utf-8-sig')
        assert re.search(r'Image has the following dependencies', text) and not re.search(r'opengl|glfw|glad', text, re.I)
    exact = []
    for name in ('PreparedStand.cpp', 'StandCom.cpp', 'SupportObservation.cpp', 'BoundedQp.cpp', 'BoundedQpReference.cpp', 'BoundedQpOptimized.cpp', 'BoundedQp.hpp', 'BoundedQpTiming.hpp'):
        path = Path('libs/physics/src')/name
        assert (baseline/path).read_bytes() == path.read_bytes(), name
        exact.append(dict(path=path.as_posix(), sha256=digest(path)))
    body = Path('libs/physics/src/BodyRig.cpp').read_text(); old_body = (baseline/'libs/physics/src/BodyRig.cpp').read_text()
    assert body.split('    BodyCase parseBodyCase')[0] == old_body.split('    BodyCase parseBodyCase')[0]
    assert body.split('    Definition bodyBenchDefinition')[1] == old_body.split('    Definition bodyBenchDefinition')[1]
    control = Path('libs/physics/src/StandControl.cpp').read_text()
    control = control.replace('        status.load.enabled=settings.loadCommands; status.load.taskEnabled=settings.loadTask;\n', '')
    control = control.replace('        if(settings_.loadCommands) advanceLoad(dt);\n', '')
    for phase in ('Idle', 'Failed'):
        control = control.replace(f' status.load.phase=ComPhase::{phase}; status.load.integral=status.load.feedback=status.load.achievedSeconds=0;', '')
    assert control == (baseline/'libs/physics/src/StandControl.cpp').read_text(), 'Old tasks/gains/guards changed'
    bounded = Path('libs/physics/src/StandBounded.cpp').read_text().replace('#include <limits>\n', '')
    def remove_block(text, opening):
        start = text.index(opening); brace = text.index('{', start); depth = 1; end = brace+1
        while depth:
            depth += (text[end] == '{')-(text[end] == '}'); end += 1
        if text[end:end+1] == '\n':
            end += 1
        return text[:start]+text[end:]
    bounded = remove_block(bounded, '        if(settings_.loadCommands&&settings_.loadTask&&status.load.baselineValid)')
    bounded = remove_block(bounded, '        if(settings_.loadCommands)')
    assert bounded == (baseline/'libs/physics/src/StandBounded.cpp').read_text(), 'Existing QP construction/hard bounds changed'
    additions = ''.join(line[1:]+'\n' for line in difflib.unified_diff((baseline/'libs/physics/src/PhysicsWorld.cpp').read_text().splitlines(), Path('libs/physics/src/PhysicsWorld.cpp').read_text().splitlines()) if line.startswith('+') and not line.startswith('+++'))
    assert not re.search(r'\b(qpos|qvel|xfrc_applied|mj_resetData|setRoot)\b', additions)
    new_controller = Path('libs/physics/src/StandLoad.cpp').read_text()
    assert not re.search(r'\b(qpos|qvel|xfrc_applied|applyWrench|mj_step|mj_forward|setRoot)\b', new_controller)
    return dict(model_identical_to_stage7i=True, model_sha256=digest(model), mass_kg=55., segments=20, actuators=41,
                root_actuators=0, equality_constraints=0, nominal_physics_hz=1920,
                unchanged_source_files=exact, body_rig_physical_and_scenario_execution_sections_exact=True,
                old_stand_tasks_gains_and_guards_exact=True, old_QP_construction_and_hard_bounds_exact=True,
                no_new_live_qv_or_external_force_write=True,
                cpu_no_opengl_dependencies=True, dependencies_folder=str(dependencies))


def negative_certificates(validation):
    result = {}
    for label in ('disable', 'support-loss', 'weak'):
        folder = validation/f'negative-{label}'
        c = read(validation/f'independent-negative-{label}/certificate.json')
        assert all(c['checks'][key] for key in ('raw_reconstruction', 'actual_friction', 'moments', 'reference'))
        for name, value in c['raw_sha256'].items():
            assert digest(folder/name) == value
        stand = list(rows(folder/'stand-v1.csv'))
        terminal = next(r for r in stand if int(r['tick']) > 0 and r['phase'] in ('lost', 'disabled'))
        tick = int(terminal['tick'])
        assert all(r['evaluations'] == terminal['evaluations'] for r in stand if int(r['tick']) >= tick)
        assert all(float(r['applied_Nm']) == 0 for r in rows(folder/'joints.csv') if int(r['tick']) > tick)
        states = {int(r['tick']):r for r in rows(folder/'state.csv')}
        after = [r for n,r in sorted(states.items()) if n > tick]
        assert len(after) >= 24 and all(r['epoch'] == states[tick]['epoch'] and int(r['world_steps'])-int(states[tick]['world_steps']) == (int(r['tick'])-tick)*16 for r in after)
        change = max(abs(float(r[key])-float(states[tick][key])) for r in after for key in ('root_x_m', 'root_y_m', 'root_z_m', 'qw', 'qx', 'qy', 'qz'))
        assert change > 1e-8
        invalid = []
        for r in rows(folder/'load-v1.csv'):
            if r['actual_valid'] == '0':
                assert math.isnan(float(r['actual']))
                if int(r['tick']) >= tick:
                    invalid.append(int(r['tick']))
        if label == 'support-loss':
            assert invalid and set(invalid).issubset(c['invalid_actual_ticks'])
        result[label] = dict(tick=tick, phase=terminal['phase'], reason=terminal['reason'], subsequent_QP=0,
                             subsequent_active_torque_Nm=0, completed_post_terminal_ticks=len(after), physical_pose_change=change,
                             invalid_actual_after_terminal_ticks=len(invalid), proven_infeasible=terminal['reason'] == 'infeasible',
                             post_fall_stand_guards=c['checks']['guards'], no_lost=c['checks']['no_lost'], raw_force_alpha_reconstruction=True)
    return result


def causality(validation):
    result = {}
    for label in ('quiet', 'pulse'):
        on = validation/'parity'/f'{label}-headless'; off = validation/f'{label}-task-off'
        assert (on/'model.xml').read_bytes() == (off/'model.xml').read_bytes()
        a, b = list(rows(on/'load-v1.csv')), list(rows(off/'load-v1.csv'))
        assert len(a) == len(b)
        keys = ('tick', 'serial', 'alpha0', 'command', 'reference', 'reference_v', 'reference_a')
        assert all(all(x[k] == y[k] for k in keys) for x,y in zip(a,b))
        d = read(validation/f'independent-{label}-task-off/certificate.json')
        assert d['pass_all'] and all(d['checks'].values()) and d['goal_mode'] == 'none'
        raw_on = list(rows(validation/f'independent-{label}/reconstructed-v1.csv'))
        raw_off = {int(r['tick']):r for r in rows(validation/f'independent-{label}-task-off/reconstructed-v1.csv')}
        commands = []
        for serial in (1,2,3):
            r = [r for r in raw_on if int(r['serial']) == serial][-1]; control = raw_off[int(r['tick'])]
            on_change = float(r['actual'])-float(r['alpha0']); off_change = float(control['actual'])-float(control['alpha0'])
            commands.append(dict(serial=serial, command=float(r['command']), alpha0=float(r['alpha0']),
                                 enabled_actual=float(r['actual']), disabled_actual=float(control['actual']),
                                 enabled_change=on_change, disabled_change=off_change, attributable_alpha_difference=on_change-off_change,
                                 enabled_left_N=float(r['left_N']), disabled_left_N=float(control['left_N']),
                                 attributable_left_N=float(r['left_N'])-float(control['left_N'])))
        result[label] = dict(identical_physical_model=True, identical_commands_reference=True, commands=commands)
    return result


def gpu(validation, output):
    parity = read(validation/'parity/comparisons.json'); assert len(parity) == 9
    result = []
    for c in parity:
        label, fps = c['scenario'], c['fps']; assert label in ('quiet', 'pulse', 'retarget') and fps in (30, 60, 144)
        path = validation/'parity'/f'{label}-{fps}'
        ticks = 2880 if label == 'retarget' else 7200
        text = (path/'gpu.txt').read_text()
        assert 'GL_errors=0' in text and 'callback_checks=PASS' in text and 'query_synchronized=true' in text
        assert f'steps={ticks*16} ' in text and f'renders={(ticks*fps+119)//120} ' in text
        assert 'OS_input=not-tested' in text
        statistics = {}
        for name in ('render_submit_us', 'GPU_elapsed_us', 'palette_upload_us'):
            match = re.search(re.escape(name)+r' mean=([\d.eE+-]+) p95=([\d.eE+-]+) max=([\d.eE+-]+)', text)
            assert match, (path, name)
            statistics[name] = dict(zip(('mean', 'p95', 'max'), map(float, match.groups())))
        counts = {}
        headless = validation/'parity'/f'{label}-headless'
        for source in sorted(headless.glob('*.csv')):
            counts[source.name] = exact_csv(source, path/source.name)
        assert counts['load-v1.csv']['rows'] == ticks+1
        result.append(dict(scenario=label, scheduler_hz=fps, ticks=ticks, statistics=statistics, exact_csv=counts,
                           callbacks='Internal callback fixture PASS', OS_input='Unverified; no OS key-delivery evidence'))
    assert {(r['scenario'], r['scheduler_hz']) for r in result} == {(s, f) for s in ('quiet', 'pulse', 'retarget') for f in (30, 60, 144)}
    write(output/'gpu-and-parity.json', result)
    return result


def budget(root):
    value = read(root/'budget.json')
    assert len(value['variants']) == 3 and len(value['runs']) == 6
    assert value['variant_limit'] == 8 and value['exploratory_native_process_limit'] == 40
    assert [r['number'] for r in value['runs']] == list(range(1, 7))
    assert sum(r['returncode'] != 0 for r in value['runs']) == 1
    return dict(variants=3, variant_limit=8, exploratory_native_processes=6, exploratory_limit=40,
                failed_exploratory_processes=1, fixed_final_validation='Separate after freeze; no tuning', ledger_sha256=digest(root/'budget.json'))


def static_feasibility(root):
    result = {}
    for label in ('feasibility-measured', 'feasibility-current'):
        c = read(root/label/'certificate.json')
        assert c['all_three_targets_static_feasible'] and not c['dynamic_control_proven']
        assert c['mujoco_version'] == '3.3.7' and c['mass_kg'] == 55. and all(x == 0 for x in c['qvel'])
        witnesses = []
        for r in c['witnesses']:
            assert r['feasible'] and r['scipy_status'] == 0 and r['peak_utilization'] <= .950000001
            assert r['root_residual'] < 1e-6 and r['inequality_violation'] < 1e-7
            force = r['force_floor_frame_N']
            assert all(force[k+1] >= -1e-7 and abs(force[k])+abs(force[k+2]) <= .6*force[k+1]+1e-7 for k in range(0, 24, 3))
            assert r['left_N'] >= 30-1e-7 and r['right_N'] >= 30-1e-7
            assert abs(r['alpha']-r['left_N']/(r['left_N']+r['right_N'])) < 1e-12
            assert all(abs(tau)/cap <= .950000001 for tau, cap in zip(r['torque_Nm'], r['torque_caps_Nm']))
            witnesses.append({key:r[key] for key in ('label', 'requested_alpha', 'alpha', 'left_N', 'right_N', 'peak_utilization', 'root_residual', 'inequality_violation')})
        result[label] = dict(normal_range=c['normal_range'], alpha0=c['alpha0'], tick=c['tick'], witnesses=witnesses,
                             dynamic_control_proven=False, certificate_sha256=digest(root/label/'certificate.json'))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--validation', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--geometry-root', type=Path, help='Contains <label>/certificate.json for seven positive runs')
    parser.add_argument('--dependencies', type=Path, default=Path('build/diagnostics/stage7j/dependencies-fallback'))
    args = parser.parse_args(); args.output.mkdir(parents=True, exist_ok=False)
    root = Path('build/diagnostics/stage7j'); validation = args.validation
    geometry = args.geometry_root or validation/'actual-geometry'
    result = dict(reader_sha256=digest(Path(__file__)), validation=str(validation), checks={}, failures=[],
                  OS_input='Unverified; real GLFW rendering and internal callbacks are separate', hard_real_time=False)
    def check(label, fn):
        try:
            result['checks'][label] = fn()
        except Exception as error:
            result['failures'].append(dict(check=label, error=repr(error)))
    def driver():
        c = read(validation/'summary.json'); assert c['pass_all'] and c['all_processes_passed'] and not c['failures']
        assert all(x['returncode'] == 0 for x in c['subprocesses'])
        return dict(pass_all=True, subprocesses=len(c['subprocesses']), summary_sha256=digest(validation/'summary.json'))
    check('fixed_driver', driver)
    check('frozen_runtime_and_accepted_baseline', lambda:frozen(root))
    check('full_suites', lambda:suites(validation))
    check('exact_accepted_histories', lambda:histories(validation, args.output))
    check('independent_QP_KKT', lambda:qp_certificate(validation))
    for label in ('quiet', 'pulse', 'yaw', 'step32', 'soft008', 'soft012', 'retarget'):
        run = validation/'parity'/f'{label}-headless' if label in ('quiet', 'pulse', 'retarget') else validation/label
        check('actual_'+label, lambda label=label,run=run:actual_run(validation,label,run,geometry/label,args.output))
    check('performance', lambda:performance(validation))
    check('model_parameters_sources_dependencies', lambda:model_and_sources(root, validation, args.dependencies))
    check('negative_semantics_and_invalid_actual', lambda:negative_certificates(validation))
    check('actual_causality', lambda:causality(validation))
    check('real_GPU_callbacks_exact_parity', lambda:gpu(validation, args.output))
    check('budget', lambda:budget(root))
    check('static_feasibility', lambda:static_feasibility(root))
    result['pass_all'] = not result['failures']
    write(args.output/'summary.json', result)
    text = ('PASS' if result['pass_all'] else 'FAIL')+' independent stage7j aggregate: '+str(len(result['checks']))+' checks; '+str(len(result['failures']))+' failures.\n'
    text += 'Actual holds after reference completion checked separately from raw-criteria holds; all completed ticks, endpoint span>=2s.\nOS input delivery: UNVERIFIED. Hard real-time: not claimed.\n'
    if result['failures']:
        text += json.dumps(result['failures'], ensure_ascii=False, indent=2)+'\n'
    (args.output/'audit.txt').write_text(text, encoding='utf-8'); print(text)
    if not result['pass_all']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
