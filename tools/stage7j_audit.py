"""Independent stage7j reader: raw contact reactions, actual holds and dynamics.

Imports neither Julia control nor its QP solver. Filtered alpha and QP alpha
are reported separately and never count as measured success. Every completed
tick of each reported hold satisfies all frozen criteria.
"""
import os
os.environ['OPENBLAS_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'
from collections import defaultdict
from pathlib import Path
import argparse
import csv
import hashlib
import json
import math
import xml.etree.ElementTree as ET
import numpy as np


def rows(path):
    with path.open(newline='', encoding='utf-8-sig') as stream:
        for row in csv.DictReader(stream):
            assert None not in row and all(v is not None for v in row.values()), path
            yield row


def vector(row, names):
    return np.array([float(row[n]) for n in names.split()])


def number(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (ValueError, TypeError):
        return None


def norm(value):
    return float(np.linalg.norm(value))


def turn(a, b):
    return float(a[0]*b[1]-a[1]*b[0])


def longest(mask, ticks, times):
    best = 0
    current = 0
    first = None
    best_start = best_end = None
    for i, good in enumerate(mask):
        current = current+1 if good and (not i or ticks[i] == ticks[i-1]+1) else (1 if good else 0)
        if current > best:
            best = current; best_start = ticks[i-current+1]; best_end = ticks[i]
        if current >= 241 and first is None:
            first = dict(start_tick=ticks[i-240], end_tick=ticks[i], start_s=times[i-240], end_s=times[i], completed_ticks=241, duration_s=2.)
    return dict(best_completed_ticks=best, best_hold_s=max(0, best-1)/120., best_start_tick=best_start, best_end_tick=best_end, first_two_seconds=first)


def audit(path, output, include_dynamics=True, goal_mode='cycle'):
    states = {int(r['tick']): r for r in rows(path/'state.csv')}
    loads = {int(r['tick']): r for r in rows(path/'load-v1.csv')}
    stands = {int(r['tick']): r for r in rows(path/'stand-v1.csv')}
    dynamic_rows = {int(r['tick']): r for r in rows(path/'dynamics-v1.csv')}
    assert states.keys() == loads.keys() == stands.keys() == dynamic_rows.keys()
    support = defaultdict(dict)
    for r in rows(path/'support-v1.csv'):
        support[int(r['tick'])][r['side']] = r
    assert support.keys() == states.keys()
    regions = defaultdict(list)
    for r in rows(path/'support-regions-v1.csv'):
        if r['owner'] in ('LeftFoot', 'RightFoot'):
            regions[(int(r['tick']), r['owner'], r['region'])].append(vector(r, 'x_m y_m z_m')[[0, 2]])
    raw = {tick: {side: dict(force=np.zeros(3), moment=np.zeros(3), active=0, records=0) for side in ('LeftFoot', 'RightFoot')} for tick in states}
    peaks = dict(force_reconstruction_N=0., moment_reconstruction_Nm=0., cop_reconstruction_m=0., actual_friction_violation_N=0.,
                 normal_export_error_N=0., actual_alpha_export_error=0., published_com_error_m=0., published_speed_error_m_s=0.,
                 actual_tau_cap=0., joint_limit_rad=0., saturation_run_s=0., slip_m_s=0., penetration_m=0., foot_displacement_m=0.)
    for c in rows(path/'support-points-v1.csv'):
        tick = int(c['tick'])
        for side in ('LeftFoot', 'RightFoot'):
            s = support[tick][side]
            if {c['geom1'], c['geom2']} != {s['geom'], s['surface']}:
                continue
            rr = raw[tick][side]; rr['records'] += 1
            if c['active'] != '1' or float(c['fn_N']) <= 1e-9:
                continue
            rr['active'] += 1
            sign = 1 if c['geom2'] == s['geom'] else -1
            f = sign*vector(c, 'f2x_N f2y_N f2z_N')
            moment = sign*vector(c, 't2x_Nm t2y_Nm t2z_Nm')
            rr['force'] += f
            rr['moment'] += np.cross(vector(c, 'x_m y_m z_m')-vector(s, 'ox_m oy_m oz_m'), f)+moment
            cone = math.hypot(float(c['ft1_N'])/float(c['mu1']), float(c['ft2_N'])/float(c['mu2']))-float(c['fn_N'])
            peaks['actual_friction_violation_N'] = max(peaks['actual_friction_violation_N'], cone)
    reconstructed = []
    initial_origins = {side: vector(support[min(states)][side], 'ox_m oy_m oz_m') for side in ('LeftFoot', 'RightFoot')}
    max_predicted_error = 0.
    cop_margins = {side: dict(sole_min_margin_m=None, loaded_min_margin_m=None) for side in ('LeftFoot', 'RightFoot')}
    invalid_actual_ticks = []
    for tick in sorted(states):
        s, l, st = states[tick], loads[tick], stands[tick]
        forces = []
        valid = dynamic_rows[tick]['valid'] == '1'
        for side in ('LeftFoot', 'RightFoot'):
            foot, rr = support[tick][side], raw[tick][side]
            assert rr['active'] == int(foot['active']) and rr['records'] == int(foot['records'])
            f, moment = rr['force'], rr['moment']
            forces.append(float(f[1]))
            peaks['force_reconstruction_N'] = max(peaks['force_reconstruction_N'], norm(f-vector(foot, 'fx_N fy_N fz_N')))
            peaks['moment_reconstruction_Nm'] = max(peaks['moment_reconstruction_Nm'], norm(moment-vector(foot, 'mx_Nm my_Nm mz_Nm')))
            peaks['normal_export_error_N'] = max(peaks['normal_export_error_N'], abs(float(foot['normal_N'])-f[1]))
            valid &= (foot['enabled'] == '1' and foot['floor'] == '1' and foot['loaded'] == '1' and rr['active'] > 0 and f[1] >= 30. and np.isfinite(f).all()
                      and foot['near'] == '1' and foot['cop_reason'] == 'available' and foot['combined_cop_reason'] == 'available'
                      and foot['other_active'] == '0' and float(foot['slip_m_s']) <= .05)
            if foot['cop_reason'] == 'available':
                origin = vector(foot, 'ox_m oy_m oz_m')
                plane = np.array([0., float(foot['com_projection_y_m']), 0.])
                transported = moment+np.cross(origin-plane, f)
                cop = plane+np.cross([0., 1., 0.], transported)/f[1]
                peaks['cop_reconstruction_m'] = max(peaks['cop_reconstruction_m'], norm(cop-vector(foot, 'cop_x_m cop_y_m cop_z_m')))
                for region in ('sole', 'loaded'):
                    vertices = regions[(tick, side, region)]
                    if len(vertices) >= 3:
                        margin = min(turn(vertices[(k+1) % len(vertices)]-a, cop[[0, 2]]-a)/norm(vertices[(k+1) % len(vertices)]-a) for k, a in enumerate(vertices))
                        field = region+'_min_margin_m'
                        old = cop_margins[side][field]
                        cop_margins[side][field] = margin if old is None else min(old, margin)
            for key, field in (('slip_m_s', 'slip_m_s'), ('penetration_m', 'penetration_m')):
                peaks[key] = max(peaks[key], float(foot[field]))
            displacement = vector(foot, 'ox_m oy_m oz_m')-initial_origins[side]
            peaks['foot_displacement_m'] = max(peaks['foot_displacement_m'], norm(displacement[[0, 2]]))
        total = sum(forces)
        valid = bool(valid and math.isfinite(total) and total >= 60.)
        actual = forces[0]/total if valid else None
        exported = number(l['actual'])
        # Independent validity is intentionally independent of drive phase: Disable
        # may retain valid measured support while it ceases active control.
        assert (l['actual_valid'] == '1') == valid, (tick, l['actual_valid'], valid, forces)
        assert (exported is not None) == valid, (tick, l['actual'])
        if valid:
            peaks['actual_alpha_export_error'] = max(peaks['actual_alpha_export_error'], abs(actual-exported))
        else:
            invalid_actual_ticks.append(tick)
        peaks['normal_export_error_N'] = max(peaks['normal_export_error_N'], abs(float(l['left_fn'])-forces[0]), abs(float(l['right_fn'])-forces[1]))
        mass = float(dynamic_rows[tick]['mass_kg'])
        actual_com = vector(s, 'com_x_m com_y_m com_z_m')
        initial_com = vector(l, 'initial_com_x initial_com_y initial_com_z')
        velocity = vector(s, 'px_kg_m_s py_kg_m_s pz_kg_m_s')/mass
        com_error = norm((actual_com-initial_com)[[0, 2]])
        speed = norm(velocity[[0, 2]])
        peaks['published_com_error_m'] = max(peaks['published_com_error_m'], abs(com_error-float(l['com_error_m'])))
        peaks['published_speed_error_m_s'] = max(peaks['published_speed_error_m_s'], abs(speed-float(l['speed_m_s'])))
        alpha0, command = number(l['alpha0']), number(l['command'])
        error = abs(actual-command) if actual is not None and command is not None else None
        delta = actual-alpha0 if actual is not None and alpha0 is not None else None
        command_delta = command-alpha0 if command is not None and alpha0 is not None else None
        directional = delta is not None and (abs(command_delta) < .049999 or delta*math.copysign(1., command_delta) >= .04)
        good = valid and error is not None and error <= .01 and directional and com_error <= .0005 and speed <= .001
        good &= st['phase'] not in ('lost', 'disabled') and support[tick]['LeftFoot']['other_active'] == '0'
        predicted = number(l['predicted'])
        if l['predicted_valid'] == '1':
            fl, fr = float(st['left_pred_fy']), float(st['right_pred_fy'])
            assert predicted is not None and fl+fr > 0
            max_predicted_error = max(max_predicted_error, abs(predicted-fl/(fl+fr)))
        else:
            assert predicted is None
        reconstructed.append(dict(tick=tick, time_s=float(s['time_s']), serial=int(l['serial']), alpha0=alpha0, command=command,
                                  reference=number(l['reference']), predicted=predicted, actual=actual, filtered=number(l['filtered']),
                                  left_N=forces[0], right_N=forces[1], actual_valid=valid, command_error=error, delta_from_alpha0=delta,
                                  com_error_m=com_error, speed_m_s=speed, meets_all=bool(good)))
    caps = {int(x.attrib['name'][5:]): float(x.attrib['ctrlrange'].split()[1]) for x in ET.parse(path/'model.xml').getroot().find('actuator')}
    for r in rows(path/'drive-v1.csv'):
        peaks['actual_tau_cap'] = max(peaks['actual_tau_cap'], abs(float(r['actual_Nm']))/caps[int(r['coordinate'])])
        peaks['saturation_run_s'] = max(peaks['saturation_run_s'], float(r['saturation_run_s']))
    for r in rows(path/'joints.csv'):
        peaks['joint_limit_rad'] = max(peaks['joint_limit_rad'], float(r['peak_limit_rad']))
    commands = []
    for serial in sorted({r['serial'] for r in reconstructed}-{0}):
        data = [r for r in reconstructed if r['serial'] == serial]
        summary = longest([r['meets_all'] for r in data], [r['tick'] for r in data], [r['time_s'] for r in data])
        final = data[-1]
        good = [r for r in data if r['meets_all']]
        summary.update(serial=serial, phase=loads[final['tick']]['phase'], command=final['command'], alpha0=final['alpha0'],
                       start_tick=data[0]['tick'], end_tick=final['tick'], final_actual=final['actual'], final_error=final['command_error'],
                       final_left_N=final['left_N'], final_right_N=final['right_N'], final_com_error_m=final['com_error_m'], final_speed_m_s=final['speed_m_s'],
                       good_actual_range=[min(r['actual'] for r in good), max(r['actual'] for r in good)] if good else None,
                       worst_error_in_best_hold=max((r['command_error'] for r in data if summary['best_start_tick'] is not None and summary['best_start_tick'] <= r['tick'] <= summary['best_end_tick']), default=None))
        commands.append(summary)
    baseline = [r for r in reconstructed if 240 <= r['tick'] <= 479]
    baseline_alphas = [r['actual'] for r in baseline if r['actual'] is not None]
    measured_alpha0 = sum(baseline_alphas)/len(baseline_alphas) if baseline_alphas else None
    reference_v = [abs(float(r['reference_v'])) for r in loads.values()]
    reference_a = [abs(float(r['reference_a'])) for r in loads.values()]
    lost = [dict(tick=tick, reason=r['reason']) for tick, r in stands.items() if r['phase'] == 'lost'][:1]
    result = dict(schema='stage7j-independent-v1', run=str(path), independent_raw_contact_method=True,
                  reader_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), peaks=peaks,
                  ticks=len(states)-1, invalid_actual_ticks=invalid_actual_ticks, commands=commands, lost=lost,
                  baseline=dict(start_tick=240, end_tick=479, valid_completed_ticks=len(baseline_alphas), endpoint_mean_alpha=measured_alpha0,
                                min_alpha=min(baseline_alphas) if baseline_alphas else None, max_alpha=max(baseline_alphas) if baseline_alphas else None,
                                substep_weighted_controller_alpha0=next((number(r['alpha0']) for r in loads.values() if r['baseline_valid'] == '1'), None)),
                  max_reference_speed=max(reference_v), max_reference_acceleration=max(reference_a), max_predicted_reconstruction_error=max_predicted_error)
    result['cop_margins'] = cop_margins
    # Pulses must be exactly the previously accepted 3N x .1s; recovery uses all
    # actual conditions relative to the active final command and a full 2s hold.
    active_pulses, group = [], []
    for r in reconstructed:
        force = vector(states[r['tick']], 'external_fx_N external_fy_N external_fz_N')
        if norm(force) > 1e-10:
            if group and r['tick'] != group[-1][0]+1:
                active_pulses.append(group); group = []
            group.append((r['tick'], force))
        elif group:
            active_pulses.append(group); group = []
    if group:
        active_pulses.append(group)
    result['pulses'] = []
    for group in active_pulses:
        start, end = group[0][0], group[-1][0]
        pulse_serial = next(r['serial'] for r in reconstructed if r['tick'] == end)
        after = [r for r in reconstructed if end < r['tick'] and r['serial'] == pulse_serial]
        recovery = longest([r['meets_all'] for r in after], [r['tick'] for r in after], [r['time_s'] for r in after])
        window = recovery['first_two_seconds']
        impulse = np.sum([f for _, f in group], axis=0)/120.
        result['pulses'].append(dict(start_tick=start, end_tick=end, ticks=len(group), impulse_Ns=impulse.tolist(),
                                     recovery_start_delay_s=(window['start_tick']-end)/120. if window else None, first_hold=window))
    selected_goals = ((len(commands) == 3 and all(c['first_two_seconds'] is not None for c in commands)) if goal_mode == 'cycle'
                      else bool(commands and commands[-1]['first_two_seconds'] is not None and abs(commands[-1]['command']-commands[-1]['alpha0']) < 1e-10) if goal_mode == 'final' else True)
    result['goal_mode'] = goal_mode
    result['checks'] = dict(raw_reconstruction=max(peaks[k] for k in ('force_reconstruction_N', 'moment_reconstruction_Nm', 'cop_reconstruction_m', 'normal_export_error_N', 'actual_alpha_export_error')) < 1e-10,
                            actual_friction=peaks['actual_friction_violation_N'] < 1e-7,
                            moments=peaks['actual_tau_cap'] <= 1.000000001,
                            reference=max(reference_v) <= .020000001 and max(reference_a) <= .040000001,
                            guards=peaks['joint_limit_rad'] <= .005 and peaks['saturation_run_s'] <= .1 and peaks['slip_m_s'] <= .05 and peaks['penetration_m'] <= .004 and peaks['foot_displacement_m'] <= .01,
                            selected_actual_goals=selected_goals,
                            no_lost=not lost,
                            pulse_recovery=all((goal_mode == 'none' or p['first_hold'] is not None) and p['ticks'] == 12 and abs(norm(p['impulse_Ns'])-.3) < 1e-9 for p in result['pulses']))
    if include_dynamics:
        from stage7h_audit import dynamics
        result['independent_inverse_dynamics'] = dynamics(path)
        result['independent_quadrature'] = quadrature(path, dynamic_rows)
    with (output/'reconstructed-v1.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(reconstructed[0])); writer.writeheader(); writer.writerows(reconstructed)
    result['raw_sha256'] = {name: hashlib.sha256((path/name).read_bytes()).hexdigest() for name in ('state.csv', 'support-v1.csv', 'support-points-v1.csv', 'load-v1.csv', 'com-solver-v1.csv')}
    return result


def compare(on, off, on_output, off_output):
    a = list(rows(on_output/'reconstructed-v1.csv')); b = list(rows(off_output/'reconstructed-v1.csv'))
    assert len(a) == len(b)
    assert (on/'model.xml').read_bytes() == (off/'model.xml').read_bytes(), 'A/B must use identical physical models'
    for left, right in zip(rows(on/'load-v1.csv'), rows(off/'load-v1.csv')):
        assert left['task_enabled'] == '1' and right['task_enabled'] == '0'
        for field in ('tick', 'alpha0', 'command', 'reference', 'reference_v', 'reference_a', 'serial', 'initial_com_x', 'initial_com_y', 'initial_com_z'):
            assert left[field] == right[field], ('causality command mismatch', field, left['tick'])
    prefix = 0
    for left, right in zip(rows(on/'state.csv'), rows(off/'state.csv')):
        if int(left['tick']) <= 480:
            assert {k: v for k, v in left.items() if k != 'mapping_us'} == {k: v for k, v in right.items() if k != 'mapping_us'}
            prefix += 1
    result = dict(identical_model_xml=True, identical_commands_and_reference=True, identical_physical_prefix_completed_ticks=prefix, commands=[])
    for serial in sorted({r['serial'] for r in a}-{'0'}):
        ids = [i for i, r in enumerate(a) if r['serial'] == serial]
        last = ids[-1]; alpha_on, alpha_off = float(a[last]['actual']), float(b[last]['actual'])
        base_on, base_off = float(a[last]['alpha0']), float(b[last]['alpha0'])
        result['commands'].append(dict(serial=int(serial), command=float(a[last]['command']), on_actual=alpha_on, off_actual=alpha_off,
                                       on_change=alpha_on-base_on, off_change=alpha_off-base_off, incremental_actual_change=(alpha_on-base_on)-(alpha_off-base_off),
                                       on_left_N=float(a[last]['left_N']), off_left_N=float(b[last]['left_N']),
                                       incremental_left_N=float(a[last]['left_N'])-float(b[last]['left_N']),
                                       on_com_error_m=float(a[last]['com_error_m']), off_com_error_m=float(b[last]['com_error_m'])))
    return result


def plot(output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    data = list(rows(output/'reconstructed-v1.csv'))
    t = np.array([float(r['time_s']) for r in data])
    def series(field, scale=1.):
        return np.array([number(r[field]) if number(r[field]) is not None else np.nan for r in data])*scale
    fig, axes = plt.subplots(4, 1, figsize=(12, 11), sharex=True, constrained_layout=True)
    for field, label, style in (('command', 'final command', '--'), ('reference', 'C2 reference', ':'), ('predicted', 'QP predicted', '-.'), ('actual', 'raw actual MuJoCo', '-')):
        axes[0].plot(t, series(field), style, label=label, linewidth=1.3)
    axes[0].set_ylabel('Left normal fraction'); axes[0].legend(ncol=2)
    axes[1].plot(t, series('left_N'), label='left'); axes[1].plot(t, series('right_N'), label='right')
    axes[1].axhline(30, color='black', linestyle=':', label='minimum 30 N'); axes[1].set_ylabel('Raw normal force (N)'); axes[1].legend()
    axes[2].plot(t, series('com_error_m', 1000), label='horizontal COM error')
    axes[2].axhline(.5, color='black', linestyle=':', label='0.5 mm criterion'); axes[2].set_ylabel('COM error (mm)'); axes[2].legend()
    axes[3].plot(t, series('speed_m_s', 1000), label='horizontal COM speed')
    axes[3].axhline(1, color='black', linestyle=':', label='1 mm/s criterion'); axes[3].set_ylabel('COM speed (mm/s)'); axes[3].set_xlabel('Physical time (s)'); axes[3].legend()
    for axis in axes:
        axis.grid(alpha=.25)
    fig.savefig(output/'load-and-com.png', dpi=160)
    fig.savefig(output/'load-and-com.svg')
    plt.close(fig)


def quadrature(path, dynamics):
    iterator = iter(rows(path/'dynamics-substeps-v1.csv'))
    first = next(iterator); p0 = vector(first, 'p_x p_y p_z'); l0 = vector(first, 'L_x L_y L_z')
    linear = np.zeros(3); angular = np.zeros(3)
    max_integral = max_residual = 0.
    count = 0
    prior = float(first['time_s'])
    step = float(ET.parse(path/'model.xml').getroot().find('option').attrib['timestep'])
    per_tick = round(1/120/step)
    assert float(first['dt_s']) == 0.
    for r in iterator:
        count += 1; dt = float(r['dt_s'])
        assert abs(dt-step) < 1e-15 and abs(float(r['time_s'])-prior-dt) < 1e-11
        prior = float(r['time_s'])
        linear += .5*dt*(vector(r, 'force_before_x force_before_y force_before_z')+vector(r, 'force_x force_y force_z'))
        angular += .5*dt*(vector(r, 'moment_before_x moment_before_y moment_before_z')+vector(r, 'moment_x moment_y moment_z'))
        if count % per_tick == 0:
            d = dynamics[int(r['tick'])]
            max_integral = max(max_integral, norm(linear-vector(d, 'impulse_x impulse_y impulse_z')), norm(angular-vector(d, 'angular_impulse_x angular_impulse_y angular_impulse_z')))
            max_residual = max(max_residual, norm(vector(r, 'p_x p_y p_z')-p0-linear-vector(d, 'residual_p_x residual_p_y residual_p_z')),
                               norm(vector(r, 'L_x L_y L_z')-l0-angular-vector(d, 'residual_L_x residual_L_y residual_L_z')))
    assert count == (len(dynamics)-1)*per_tick and max(max_integral, max_residual) < 1e-10
    return dict(substeps=count, substeps_per_tick=per_tick, step_s=step, integral_error=max_integral, residual_reconstruction_error=max_residual,
                max_linear_residual_Ns=max(norm(vector(d, 'residual_p_x residual_p_y residual_p_z')) for d in dynamics.values()),
                max_angular_residual_Nms=max(norm(vector(d, 'residual_L_x residual_L_y residual_L_z')) for d in dynamics.values()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--require-goals', action='store_true')
    parser.add_argument('--goal-mode', choices=('cycle', 'final', 'none'), default='cycle', help='cycle requires all three goals; final requires final return (retarget/cancel); none reports comparator/negative checks')
    parser.add_argument('--skip-dynamics', action='store_true', help='Only for quick research reads; final audit includes dynamics')
    parser.add_argument('--compare-to', type=Path, help='Matched task-off run: verify physical prefix, commands and causal changes')
    parser.add_argument('--plot', action='store_true')
    args = parser.parse_args(); args.output.mkdir(parents=True, exist_ok=False)
    result = audit(args.run, args.output, not args.skip_dynamics, args.goal_mode)
    if args.compare_to:
        other = args.output/'task-off'; other.mkdir()
        control = audit(args.compare_to, other, False, 'none')
        (other/'certificate.json').write_text(json.dumps(control, indent=2), encoding='utf-8')
        result['causality'] = compare(args.run, args.compare_to, args.output, other)
    if args.plot:
        plot(args.output)
    result['pass_all'] = all(result['checks'].values())
    (args.output/'certificate.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps(result, indent=2))
    if args.require_goals and not result['pass_all']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
