"""Independent stage7k reader: joint COM/alpha holds from raw state, model and contacts.

Imports neither Julia control nor its QP solver. Actual COM and COM velocity are
recomputed in scratch MuJoCo 3.3.7 from the exported MJCF plus state.csv/joints.csv;
actual Fn and alpha from the raw foot-floor contact records. Every completed tick of
a reported hold satisfies ALL joint criteria simultaneously, after BOTH references
stopped. Exported controller values are compared, never trusted for success.
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
import sys
import xml.etree.ElementTree as ET
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
TICK = 1/120.
COM_ERROR, COM_SPEED, COM_DIRECTIONAL = .0005, .001, .002
ALPHA_ERROR, ALPHA_CHANGE, MIN_FN, HOLD = .01, .04, 30., 241


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


def runs(mask, ticks, times):
    """All maximal runs of consecutive completed ticks satisfying the mask."""
    result, start = [], None
    for i, good in enumerate(mask):
        if good and (start is None or ticks[i] != ticks[i-1]+1):
            if start is not None:
                result.append((start, i-1))
            start = i
        elif not good and start is not None:
            result.append((start, i-1)); start = None
    if start is not None:
        result.append((start, len(mask)-1))
    return [dict(start_tick=ticks[a], end_tick=ticks[b], start_s=times[a], end_s=times[b], completed_ticks=b-a+1,
                 endpoint_span_s=(ticks[b]-ticks[a])*TICK) for a, b in result]


def mujoco():
    sys.path.insert(0, str(Path('build/diagnostics/stage7g/python-deps').resolve()))
    import mujoco as mj
    assert mj.__version__ == '3.3.7'
    return mj


def reconstruct_state(path, mj):
    """Independent COM, COM velocity and foot-box geometry for every completed tick."""
    m = mj.MjModel.from_xml_path(str((path/'model.xml').resolve()))
    d = mj.MjData(m)
    assert (m.nq, m.nv, m.nu, m.neq) == (48, 47, 41, 0) and abs(float(m.body_mass.sum())-55.) < 1e-10
    root = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, 'root')
    body = int(m.jnt_bodyid[root]); rq, rv = int(m.jnt_qposadr[root]), int(m.jnt_dofadr[root])
    joints = [mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, f'j{k}') for k in range(41)]
    qa, va = m.jnt_qposadr[joints], m.jnt_dofadr[joints]
    limits = m.jnt_range[joints].copy()
    geoms = [mj.mj_name2id(m, mj.mjtObj.mjOBJ_GEOM, f'g{k}') for k in (16, 19)]
    floor = mj.mj_name2id(m, mj.mjtObj.mjOBJ_GEOM, 'floor')
    floor_y = float(d.geom_xpos[floor][1]) if floor >= 0 else 0.
    q = defaultdict(dict); v = defaultdict(dict)
    for r in rows(path/'joints.csv'):
        tick = int(r['tick']); k = int(r['coordinate'])
        q[tick][k] = float(r['q_rad']); v[tick][k] = float(r['v_rad_s'])
    jac = np.zeros((3, m.nv))
    result = {}
    initial_boxes = None
    for s in rows(path/'state.csv'):
        tick = int(s['tick'])
        d.qpos[rq:rq+3] = vector(s, 'root_x_m root_y_m root_z_m'); d.qpos[rq+3:rq+7] = vector(s, 'qw qx qy qz')
        d.qvel[rv:rv+3] = vector(s, 'vx_world_m_s vy_world_m_s vz_world_m_s'); d.qvel[rv+3:rv+6] = vector(s, 'wx_local_rad_s wy_local_rad_s wz_local_rad_s')
        assert len(q[tick]) == 41
        d.qpos[qa] = [q[tick][k] for k in range(41)]; d.qvel[va] = [v[tick][k] for k in range(41)]
        mj.mj_kinematics(m, d); mj.mj_comPos(m, d)
        com = d.subtree_com[body].copy()
        mj.mj_jacSubtreeCom(m, d, jac, body)
        velocity = jac@d.qvel
        boxes = []
        for g in geoms:
            rotation = d.geom_xmat[g].reshape(3, 3).copy(); center = d.geom_xpos[g].copy()
            corners = [center+rotation@np.array([sx*.045, -.025, sz*.085]) for sx in (-1, 1) for sz in (-1, 1)]
            boxes.append((center, rotation, [float(c[1]-floor_y) for c in corners]))
        if initial_boxes is None:
            initial_boxes = boxes
        displacement = max(norm((b[0]-i[0])[[0, 2]]) for b, i in zip(boxes, initial_boxes))
        angle = max(math.acos(float(np.clip((np.trace(i[1].T@b[1])-1.)*.5, -1., 1.))) for b, i in zip(boxes, initial_boxes))
        violation = float(np.max(np.maximum(0., np.maximum(limits[:, 0]-d.qpos[qa], d.qpos[qa]-limits[:, 1]))))
        result[tick] = dict(com=com, velocity=velocity, root_q=vector(s, 'qw qx qy qz'), exported_com=vector(s, 'com_x_m com_y_m com_z_m'),
                            exported_momentum=vector(s, 'px_kg_m_s py_kg_m_s pz_kg_m_s'), corner_heights=[b[2] for b in boxes],
                            box_displacement=displacement, box_angle=angle, joint_violation=violation)
    return result, float(m.body_mass.sum())


def raw_contacts(path, ticks):
    support = defaultdict(dict)
    for r in rows(path/'support-v1.csv'):
        support[int(r['tick'])][r['side']] = r
    assert set(support) == set(ticks)
    regions = defaultdict(list)
    for r in rows(path/'support-regions-v1.csv'):
        if r['owner'] in ('LeftFoot', 'RightFoot'):
            regions[(int(r['tick']), r['owner'], r['region'])].append(vector(r, 'x_m y_m z_m')[[0, 2]])
    raw = {tick: {side: dict(force=np.zeros(3), moment=np.zeros(3), active=0, records=0) for side in ('LeftFoot', 'RightFoot')} for tick in ticks}
    friction = 0.
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
            f = sign*vector(c, 'f2x_N f2y_N f2z_N'); moment = sign*vector(c, 't2x_Nm t2y_Nm t2z_Nm')
            rr['force'] += f
            rr['moment'] += np.cross(vector(c, 'x_m y_m z_m')-vector(s, 'ox_m oy_m oz_m'), f)+moment
            friction = max(friction, math.hypot(float(c['ft1_N'])/float(c['mu1']), float(c['ft2_N'])/float(c['mu2']))-float(c['fn_N']))
    return support, regions, raw, friction


def audit(path, output, mode='cycle', include_dynamics=True):
    mj = mujoco()
    coordinated = {int(r['tick']): r for r in rows(path/'coordinated-v1.csv')}
    stands = {int(r['tick']): r for r in rows(path/'stand-v1.csv')}
    states = {int(r['tick']): r for r in rows(path/'state.csv')}
    dynamic_rows = {int(r['tick']): r for r in rows(path/'dynamics-v1.csv')}
    ticks = sorted(states)
    assert set(coordinated) == set(stands) == set(states) == set(dynamic_rows), 'Every completed tick must be exported once'
    model, mass = reconstruct_state(path, mj)
    support, regions, raw, friction = raw_contacts(path, ticks)
    first = coordinated[ticks[0]]
    # Frozen placement frame and initial COM goal, recomputed from the reset state (tick 0).
    initial = model[ticks[0]]['com']
    q0 = model[ticks[0]]['root_q']
    w, x, y, z = q0
    frame = np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)], [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)], [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])
    exported_initial = vector(first, 'initial_com_x initial_com_y initial_com_z')
    placement = vector(first, 'placement_qw placement_qx placement_qy placement_qz')
    peaks = dict(com_reconstruction_m=0., com_velocity_vs_momentum_m_s=0., exported_com_error_m=0., exported_speed_m_s=0.,
                 exported_offset_m=0., exported_goal_com_m=0., force_reconstruction_N=0., moment_reconstruction_Nm=0., cop_reconstruction_m=0.,
                 normal_export_error_N=0., actual_alpha_export_error=0., actual_friction_violation_N=friction,
                 actual_tau_cap=0., joint_limit_rad=0., saturation_run_s=0., slip_m_s=0., penetration_m=0., foot_displacement_m=0.,
                 independent_box_displacement_m=0., independent_box_angle_rad=0., independent_joint_violation_rad=0.,
                 min_corner_height_m=float('inf'), max_corner_height_m=-float('inf'))
    initial_origins = {side: vector(support[ticks[0]][side], 'ox_m oy_m oz_m') for side in ('LeftFoot', 'RightFoot')}
    cop_margins = {side: dict(sole_min_margin_m=None, loaded_min_margin_m=None) for side in ('LeftFoot', 'RightFoot')}
    reconstructed = []
    invalid_actual_ticks = []
    for tick in ticks:
        c, st, mrow = coordinated[tick], stands[tick], model[tick]
        peaks['com_reconstruction_m'] = max(peaks['com_reconstruction_m'], norm(mrow['com']-mrow['exported_com']))
        peaks['com_velocity_vs_momentum_m_s'] = max(peaks['com_velocity_vs_momentum_m_s'], norm(mrow['velocity']-mrow['exported_momentum']/mass))
        peaks['independent_box_displacement_m'] = max(peaks['independent_box_displacement_m'], mrow['box_displacement'])
        peaks['independent_box_angle_rad'] = max(peaks['independent_box_angle_rad'], mrow['box_angle'])
        peaks['independent_joint_violation_rad'] = max(peaks['independent_joint_violation_rad'], mrow['joint_violation'])
        for heights in mrow['corner_heights']:
            peaks['min_corner_height_m'] = min(peaks['min_corner_height_m'], min(heights))
            peaks['max_corner_height_m'] = max(peaks['max_corner_height_m'], max(heights))
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
            valid &= (foot['enabled'] == '1' and foot['floor'] == '1' and foot['loaded'] == '1' and rr['active'] > 0 and f[1] >= MIN_FN and np.isfinite(f).all()
                      and foot['near'] == '1' and foot['cop_reason'] == 'available' and foot['combined_cop_reason'] == 'available'
                      and foot['other_active'] == '0' and float(foot['slip_m_s']) <= .05)
            if foot['cop_reason'] == 'available':
                origin = vector(foot, 'ox_m oy_m oz_m'); plane = np.array([0., float(foot['com_projection_y_m']), 0.])
                transported = moment+np.cross(origin-plane, f)
                cop = plane+np.cross([0., 1., 0.], transported)/f[1]
                peaks['cop_reconstruction_m'] = max(peaks['cop_reconstruction_m'], norm(cop-vector(foot, 'cop_x_m cop_y_m cop_z_m')))
                for region in ('sole', 'loaded'):
                    vertices = regions[(tick, side, region)]
                    if len(vertices) >= 3:
                        margin = min(turn(vertices[(k+1) % len(vertices)]-a, cop[[0, 2]]-a)/norm(vertices[(k+1) % len(vertices)]-a) for k, a in enumerate(vertices))
                        field = region+'_min_margin_m'; old = cop_margins[side][field]
                        cop_margins[side][field] = margin if old is None else min(old, margin)
            peaks['slip_m_s'] = max(peaks['slip_m_s'], float(foot['slip_m_s']))
            peaks['penetration_m'] = max(peaks['penetration_m'], float(foot['penetration_m']))
            peaks['foot_displacement_m'] = max(peaks['foot_displacement_m'], norm((vector(foot, 'ox_m oy_m oz_m')-initial_origins[side])[[0, 2]]))
        total = sum(forces)
        valid = bool(valid and math.isfinite(total) and total >= 2*MIN_FN)
        alpha = forces[0]/total if valid else None
        exported = number(c['alpha_actual'])
        assert (c['actual_valid'] == '1') == valid, (tick, c['actual_valid'], valid, forces)
        assert (exported is not None) == valid, (tick, c['alpha_actual'])
        if valid:
            peaks['actual_alpha_export_error'] = max(peaks['actual_alpha_export_error'], abs(alpha-exported))
        else:
            invalid_actual_ticks.append(tick)
        peaks['normal_export_error_N'] = max(peaks['normal_export_error_N'], abs(float(c['left_fn'])-forces[0]), abs(float(c['right_fn'])-forces[1]))
        com, velocity = mrow['com'], mrow['velocity']
        offset3 = frame.T@(com-initial); offset = offset3[[0, 2]]
        goal_offset = vector(c, 'goal_offset_x goal_offset_z')
        goal_world = initial+frame@np.array([goal_offset[0], 0., goal_offset[1]])
        peaks['exported_goal_com_m'] = max(peaks['exported_goal_com_m'], norm((goal_world-vector(c, 'goal_com_x goal_com_y goal_com_z'))[[0, 2]]))
        com_error = norm((com-goal_world)[[0, 2]]); speed = norm(velocity[[0, 2]])
        peaks['exported_com_error_m'] = max(peaks['exported_com_error_m'], abs(com_error-float(c['com_error_m'])))
        peaks['exported_speed_m_s'] = max(peaks['exported_speed_m_s'], abs(speed-float(c['speed_m_s'])))
        peaks['exported_offset_m'] = max(peaks['exported_offset_m'], norm(offset-vector(c, 'offset_x offset_z')))
        length = norm(goal_offset)
        directional = float(offset@goal_offset/length) if length > 0 else 0.
        alpha0, goal_alpha = number(c['alpha0']), number(c['alpha_goal'])
        serial, cancelled = int(c['serial']), c['cancelled'] == '1'
        error = abs(alpha-goal_alpha) if alpha is not None and goal_alpha is not None else None
        change = (alpha-alpha0)*math.copysign(1., goal_alpha-alpha0) if alpha is not None and alpha0 is not None and goal_alpha is not None else None
        full_com = length >= .0029999 and not cancelled
        full_alpha = goal_alpha is not None and alpha0 is not None and abs(goal_alpha-alpha0) >= .049999 and not cancelled
        reference = vector(c, 'com_ref_x com_ref_y com_ref_z'); ref_v = vector(c, 'com_ref_vx com_ref_vy com_ref_vz'); ref_a = vector(c, 'com_ref_ax com_ref_ay com_ref_az')
        alpha_ref = number(c['alpha_ref'])
        at_goal = (serial > 0 and norm(reference-vector(c, 'goal_com_x goal_com_y goal_com_z')) <= 1e-12 and norm(ref_v) <= 1e-12 and norm(ref_a) <= 1e-12
                   and alpha_ref is not None and goal_alpha is not None and abs(alpha_ref-goal_alpha) <= 1e-12
                   and abs(float(c['alpha_ref_v'])) <= 1e-12 and abs(float(c['alpha_ref_a'])) <= 1e-12)
        # Conservative: both the exported common-time flag and the exported reference values.
        # The values may reach the goal one tick before the accumulated time equals the duration.
        flag = c['stopped'] == '1' and serial > 0
        assert at_goal or not flag, (tick, 'reference flagged stopped away from its goal')
        stopped = bool(flag and at_goal)
        com_ok = com_error <= COM_ERROR and speed <= COM_SPEED and (not full_com or directional >= COM_DIRECTIONAL)
        alpha_ok = error is not None and error <= ALPHA_ERROR and (not full_alpha or change >= ALPHA_CHANGE)
        phase_ok = st['phase'] not in ('lost', 'disabled')
        good = bool(stopped and com_ok and alpha_ok and valid and phase_ok)
        reconstructed.append(dict(tick=tick, time_s=float(states[tick]['time_s']), serial=serial, cancelled=cancelled, stopped=stopped,
                                  goal_offset_x=goal_offset[0], goal_offset_z=goal_offset[1], goal_alpha=goal_alpha, alpha0=alpha0,
                                  ref_offset_x=float((frame.T@(reference-initial))[0]), ref_offset_z=float((frame.T@(reference-initial))[2]),
                                  alpha_reference=alpha_ref, alpha_predicted=number(c['alpha_predicted']),
                                  offset_x=float(offset[0]), offset_z=float(offset[1]), com_error_m=com_error, speed_m_s=speed, directional_m=directional,
                                  alpha_actual=alpha, alpha_error=error, alpha_change=change, left_N=forces[0], right_N=forces[1], actual_valid=valid,
                                  com_ok=bool(com_ok), alpha_ok=bool(alpha_ok), stand_phase=st['phase'], joint_good=good))
    assert peaks['com_reconstruction_m'] < 1e-9 and peaks['com_velocity_vs_momentum_m_s'] < 1e-9, peaks
    assert norm(initial-exported_initial) < 1e-9 and norm(np.abs(placement)-np.abs(q0)) < 1e-9, 'Initial goal/placement must equal the reset state'
    caps = {int(x.attrib['name'][5:]): float(x.attrib['ctrlrange'].split()[1]) for x in ET.parse(path/'model.xml').getroot().find('actuator')}
    for r in rows(path/'drive-v1.csv'):
        peaks['actual_tau_cap'] = max(peaks['actual_tau_cap'], abs(float(r['actual_Nm']))/caps[int(r['coordinate'])])
        peaks['saturation_run_s'] = max(peaks['saturation_run_s'], float(r['saturation_run_s']))
    for r in rows(path/'joints.csv'):
        peaks['joint_limit_rad'] = max(peaks['joint_limit_rad'], float(r['peak_limit_rad']))
    # References: limits and C2 consistency on every tick, including command switches.
    ref = [coordinated[t] for t in ticks]
    alpha0_values = {number(r['alpha0']) for r in ref if r['baseline_valid'] == '1'}
    assert len(alpha0_values) <= 1
    alpha0 = next(iter(alpha0_values)) if alpha0_values else None
    references = dict(max_com_speed_m_s=0., max_com_acceleration_m_s2=0., max_com_radius_m=0., max_alpha_speed=0., max_alpha_acceleration=0., max_alpha_excursion=0.,
                      max_position_trapezoid_residual_m=0., max_velocity_trapezoid_residual_m_s=0., max_alpha_trapezoid_residual=0., max_alpha_velocity_trapezoid_residual=0.,
                      switch_ticks=[])
    previous = None
    for r in ref:
        p = vector(r, 'com_ref_x com_ref_y com_ref_z'); pv = vector(r, 'com_ref_vx com_ref_vy com_ref_vz'); pa = vector(r, 'com_ref_ax com_ref_ay com_ref_az')
        references['max_com_speed_m_s'] = max(references['max_com_speed_m_s'], norm(pv))
        references['max_com_acceleration_m_s2'] = max(references['max_com_acceleration_m_s2'], norm(pa))
        references['max_com_radius_m'] = max(references['max_com_radius_m'], norm((p-initial)[[0, 2]]))
        av, aa, ar = float(r['alpha_ref_v']), float(r['alpha_ref_a']), number(r['alpha_ref'])
        references['max_alpha_speed'] = max(references['max_alpha_speed'], abs(av))
        references['max_alpha_acceleration'] = max(references['max_alpha_acceleration'], abs(aa))
        if ar is not None and alpha0 is not None:
            references['max_alpha_excursion'] = max(references['max_alpha_excursion'], abs(ar-alpha0))
        # The reference evolves only while the stand controller runs; after Disable/Lost it is frozen.
        active = stands[int(r['tick'])]['phase'] not in ('lost', 'disabled')
        if previous is not None and active and previous_active:
            q, qv, qa, br, bv, ba = previous
            references['max_position_trapezoid_residual_m'] = max(references['max_position_trapezoid_residual_m'], norm(p-q-.5*(pv+qv)*TICK))
            references['max_velocity_trapezoid_residual_m_s'] = max(references['max_velocity_trapezoid_residual_m_s'], norm(pv-qv-.5*(pa+qa)*TICK))
            if ar is not None and br is not None:
                references['max_alpha_trapezoid_residual'] = max(references['max_alpha_trapezoid_residual'], abs(ar-br-.5*(av+bv)*TICK))
                references['max_alpha_velocity_trapezoid_residual'] = max(references['max_alpha_velocity_trapezoid_residual'], abs(av-bv-.5*(aa+ba)*TICK))
            if r['serial'] != prior_serial:
                references['switch_ticks'].append(int(r['tick']))
        previous = (p, pv, pa, ar, av, aa); prior_serial = r['serial']; previous_active = active
    commands = []
    for serial in sorted({r['serial'] for r in reconstructed}-{0}):
        data = [r for r in reconstructed if r['serial'] == serial]
        all_runs = runs([r['joint_good'] for r in data], [r['tick'] for r in data], [r['time_s'] for r in data])
        best = max(all_runs, key=lambda x: x['completed_ticks'], default=None)
        first = next((x for x in all_runs if x['completed_ticks'] >= HOLD), None)
        stopped_ticks = [r for r in data if r['stopped']]
        summary = dict(serial=serial, start_tick=data[0]['tick'], end_tick=data[-1]['tick'], cancelled=data[-1]['cancelled'],
                       goal_offset_mm=[1000*data[-1]['goal_offset_x'], 1000*data[-1]['goal_offset_z']], goal_alpha=data[-1]['goal_alpha'], alpha0=data[-1]['alpha0'],
                       reference_stop_tick=stopped_ticks[0]['tick'] if stopped_ticks else None,
                       final=dict((k, data[-1][k]) for k in ('offset_x', 'offset_z', 'com_error_m', 'speed_m_s', 'directional_m', 'alpha_actual', 'alpha_error', 'alpha_change', 'left_N', 'right_N')),
                       joint_runs=len(all_runs), best=best, first_full_window=None if first is None else dict(first, completed_ticks_window=HOLD))
        if best is not None:
            inside = [r for r in data if best['start_tick'] <= r['tick'] <= best['end_tick']]
            summary['best_worst'] = dict(max_com_error_m=max(r['com_error_m'] for r in inside), max_speed_m_s=max(r['speed_m_s'] for r in inside),
                                         max_alpha_error=max(r['alpha_error'] for r in inside), min_directional_m=min(r['directional_m'] for r in inside),
                                         min_alpha_change=min(r['alpha_change'] for r in inside), min_left_N=min(r['left_N'] for r in inside),
                                         min_right_N=min(r['right_N'] for r in inside),
                                         mean_offset_mm=[1000*float(np.mean([r['offset_x'] for r in inside])), 1000*float(np.mean([r['offset_z'] for r in inside]))],
                                         mean_alpha=float(np.mean([r['alpha_actual'] for r in inside])))
        commands.append(summary)
    baseline = [r['alpha_actual'] for r in reconstructed if 240 <= r['tick'] <= 479 and r['alpha_actual'] is not None]
    lost = [dict(tick=t, reason=stands[t]['reason']) for t in ticks if stands[t]['phase'] == 'lost'][:1]
    # Pulses: exactly 3N x .1s; recovery = start of a full joint window after the pulse end.
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
    pulses = []
    for group in active_pulses:
        start, end = group[0][0], group[-1][0]
        serial = next(r['serial'] for r in reconstructed if r['tick'] == end)
        after = [r for r in reconstructed if r['tick'] > end and r['serial'] == serial]
        window = next((x for x in runs([r['joint_good'] for r in after], [r['tick'] for r in after], [r['time_s'] for r in after]) if x['completed_ticks'] >= HOLD), None)
        impulse = np.sum([f for _, f in group], axis=0)*TICK
        local = frame.T@impulse
        peak_error = max((r['com_error_m'] for r in after[:360]), default=None)
        pulses.append(dict(start_tick=start, end_tick=end, ticks=len(group), serial=serial, impulse_Ns=impulse.tolist(), impulse_placement_Ns=local.tolist(),
                           recovery_delay_s=None if window is None else (window['start_tick']-end)*TICK, recovered_window=window,
                           max_com_error_3s_after_m=peak_error))
    expected = dict(cycle=dict(required=[1, 2, 3, 4], main=[1, 2, 3]), retarget=dict(required=[3, 4, 6, 7], main=[3, 4]), none=dict(required=[], main=[]))[mode]
    found = {c['serial']: c for c in commands}
    goals = all(s in found and found[s]['first_full_window'] is not None for s in expected['required'])
    if mode == 'cycle':
        goals &= len(commands) == 4
    if mode == 'retarget':
        goals &= len(commands) == 7 and found.get(6, {}).get('cancelled', False)
    result = dict(schema='stage7k-independent-v1', run=str(path), mode=mode, reader_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  independent_com_method='scratch MuJoCo 3.3.7 subtree COM and jacSubtreeCom*qvel from exported MJCF, state.csv and joints.csv',
                  initial_com_goal_m=initial.tolist(), placement_quaternion=q0.tolist(), alpha0=alpha0,
                  baseline_endpoint_mean_alpha=sum(baseline)/len(baseline) if baseline else None, baseline_completed_ticks=len(baseline),
                  ticks=len(ticks)-1, invalid_actual_ticks=len(invalid_actual_ticks), first_invalid_actual_tick=invalid_actual_ticks[0] if invalid_actual_ticks else None,
                  peaks=peaks, cop_margins=cop_margins, references=references, commands=commands, pulses=pulses, lost=lost,
                  required_serials=expected['required'], main_serials=expected['main'])
    result['checks'] = dict(
        raw_reconstruction=max(peaks[k] for k in ('force_reconstruction_N', 'moment_reconstruction_Nm', 'cop_reconstruction_m', 'normal_export_error_N', 'actual_alpha_export_error')) < 1e-10,
        independent_com=peaks['com_reconstruction_m'] < 1e-9 and peaks['com_velocity_vs_momentum_m_s'] < 1e-9 and peaks['exported_offset_m'] < 1e-9 and peaks['exported_goal_com_m'] < 1e-12,
        actual_friction=peaks['actual_friction_violation_N'] < 1e-7,
        moments=peaks['actual_tau_cap'] <= 1.000000001,
        reference_limits=(references['max_com_speed_m_s'] <= .001+1e-9 and references['max_com_acceleration_m_s2'] <= .002+1e-9 and references['max_com_radius_m'] <= .005+1e-9
                          and references['max_alpha_speed'] <= .02+1e-9 and references['max_alpha_acceleration'] <= .04+1e-9 and references['max_alpha_excursion'] <= .05+1e-9),
        reference_c2=references['max_position_trapezoid_residual_m'] < 1e-8 and references['max_velocity_trapezoid_residual_m_s'] < 1e-7
                     and references['max_alpha_trapezoid_residual'] < 1e-7 and references['max_alpha_velocity_trapezoid_residual'] < 1e-6,
        guards=(peaks['joint_limit_rad'] <= .005 and peaks['saturation_run_s'] <= .1 and peaks['slip_m_s'] <= .05 and peaks['penetration_m'] <= .004
                and peaks['foot_displacement_m'] <= .01 and peaks['independent_box_displacement_m'] <= .01 and peaks['independent_box_angle_rad'] <= .05
                and peaks['independent_joint_violation_rad'] <= .005),
        joint_actual_goals=goals,
        no_lost=not lost,
        pulse_recovery=all(p['ticks'] == 12 and abs(norm(p['impulse_Ns'])-.3) < 1e-9 and p['recovery_delay_s'] is not None and p['recovery_delay_s'] <= 3. for p in pulses) if mode != 'none' else True)
    if mode == 'none':
        for key in ('joint_actual_goals', 'no_lost', 'guards'):
            result['checks'][key] = True
        result['note'] = 'comparator/negative reading: goal, guard and lost checks reported, not required'
    if include_dynamics:
        from stage7h_audit import dynamics
        from stage7j_audit import quadrature
        result['independent_inverse_dynamics'] = dynamics(path)
        result['independent_quadrature'] = quadrature(path, dynamic_rows)
    with (output/'reconstructed-v1.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(reconstructed[0])); writer.writeheader(); writer.writerows(reconstructed)
    result['raw_sha256'] = {name: hashlib.sha256((path/name).read_bytes()).hexdigest() for name in
                            ('model.xml', 'state.csv', 'joints.csv', 'support-v1.csv', 'support-points-v1.csv', 'coordinated-v1.csv', 'com-solver-v1.csv')}
    return result


def compare(base, other, base_output, other_output, label):
    """Matched causal variant: identical model/commands/references; differences only from tasks."""
    assert (base/'model.xml').read_bytes() == (other/'model.xml').read_bytes(), 'Variants must use identical physical models'
    fields = ('tick', 'serial', 'cancelled', 'stopped', 'duration_s', 'goal_offset_x', 'goal_offset_z', 'goal_com_x', 'goal_com_y', 'goal_com_z',
              'com_ref_x', 'com_ref_y', 'com_ref_z', 'com_ref_vx', 'com_ref_vz', 'alpha_goal', 'alpha_ref', 'alpha_ref_v', 'alpha_ref_a', 'alpha0',
              'initial_com_x', 'initial_com_y', 'initial_com_z')
    for left, right in zip(rows(base/'coordinated-v1.csv'), rows(other/'coordinated-v1.csv')):
        for field in fields:
            assert left[field] == right[field], ('causal command/reference mismatch', label, field, left['tick'])
    prefix = 0
    for left, right in zip(rows(base/'state.csv'), rows(other/'state.csv')):
        if int(left['tick']) <= 480:
            assert {k: v for k, v in left.items() if k != 'mapping_us'} == {k: v for k, v in right.items() if k != 'mapping_us'}, ('physical prefix differs', label, left['tick'])
            prefix += 1
    a = list(rows(base_output/'reconstructed-v1.csv')); b = list(rows(other_output/'reconstructed-v1.csv'))
    assert len(a) == len(b)
    result = dict(variant=label, identical_model_xml=True, identical_commands_and_references=True, identical_physical_prefix_completed_ticks=prefix, commands=[])
    def f(r, k):
        return float(r[k]) if r[k] not in ('', 'None') else None
    for serial in sorted({r['serial'] for r in a}-{'0'}):
        ids = [i for i, r in enumerate(a) if r['serial'] == serial]
        last = ids[-240:]
        def mean(rowsx, key):
            values = [f(r, key) for r in rowsx]
            return None if any(v is None for v in values) else sum(values)/len(values)
        entry = dict(serial=int(serial), goal_offset_mm=[1000*f(a[ids[-1]], 'goal_offset_x'), 1000*f(a[ids[-1]], 'goal_offset_z')], goal_alpha=f(a[ids[-1]], 'goal_alpha'),
                     endpoint_tick=int(a[ids[-1]]['tick']), statistic='endpoint and mean of the last 240 completed ticks of the command; not a hold claim')
        for name, data in (('coordinated', a), (label, b)):
            entry[name] = dict(endpoint_offset_mm=[1000*f(data[ids[-1]], 'offset_x'), 1000*f(data[ids[-1]], 'offset_z')], endpoint_alpha=f(data[ids[-1]], 'alpha_actual'),
                               endpoint_left_N=f(data[ids[-1]], 'left_N'), mean_offset_mm=[1000*mean([data[i] for i in last], 'offset_x'), 1000*mean([data[i] for i in last], 'offset_z')],
                               mean_alpha=mean([data[i] for i in last], 'alpha_actual'), endpoint_directional_mm=1000*f(data[ids[-1]], 'directional_m'),
                               endpoint_com_error_mm=1000*f(data[ids[-1]], 'com_error_m'), joint_good_ticks=sum(data[i]['joint_good'] == 'True' for i in ids))
        alpha0 = f(a[ids[-1]], 'alpha0')
        for name in ('coordinated', label):
            e = entry[name]
            e['endpoint_alpha_change'] = None if e['endpoint_alpha'] is None or alpha0 is None else e['endpoint_alpha']-alpha0
        result['commands'].append(entry)
    return result


def plot(output, title):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    data = list(rows(output/'reconstructed-v1.csv'))
    t = np.array([float(r['time_s']) for r in data])
    def series(field, scale=1.):
        return np.array([number(r[field]) if number(r[field]) is not None else np.nan for r in data])*scale
    good = np.array([r['joint_good'] == 'True' for r in data])
    fig, axes = plt.subplots(5, 1, figsize=(13, 15), sharex=True, constrained_layout=True)
    for axis, key, label in ((axes[0], 'x', 'placement X'), (axes[1], 'z', 'placement Z')):
        axis.plot(t, series('goal_offset_'+key, 1000), '--', label='terminal goal', linewidth=1.3)
        axis.plot(t, series('ref_offset_'+key, 1000), ':', label='C2 reference', linewidth=1.6)
        axis.plot(t, series('offset_'+key, 1000), '-', label='actual (independent MuJoCo COM)', linewidth=1.1)
        axis.set_ylabel(f'COM offset {label} (mm)'); axis.legend(ncol=3, fontsize=8)
    for field, label, style in (('goal_alpha', 'terminal goal', '--'), ('alpha_reference', 'C2 reference', ':'), ('alpha_predicted', 'QP predicted', '-.'), ('alpha_actual', 'raw actual (contacts)', '-')):
        axes[2].plot(t, series(field), style, label=label, linewidth=1.2)
    axes[2].set_ylabel('Left normal fraction alpha'); axes[2].legend(ncol=4, fontsize=8)
    axes[3].plot(t, series('left_N'), label='left'); axes[3].plot(t, series('right_N'), label='right')
    axes[3].axhline(30, color='black', linestyle=':', label='30 N'); axes[3].set_ylabel('Raw Fn (N)'); axes[3].legend(ncol=3, fontsize=8)
    axes[4].plot(t, series('com_error_m', 1000), label='COM error to goal (mm)')
    axes[4].plot(t, series('speed_m_s', 1000), label='COM speed (mm/s)')
    axes[4].plot(t, series('alpha_error', 100), label='alpha error x100')
    axes[4].axhline(.5, color='black', linestyle=':', label='0.5 mm'); axes[4].axhline(1, color='gray', linestyle=':', label='1 mm/s / 0.01 alpha')
    axes[4].set_ylim(0, 3); axes[4].set_ylabel('Errors'); axes[4].set_xlabel('Physical time (s)'); axes[4].legend(ncol=3, fontsize=8)
    for axis in axes:
        axis.grid(alpha=.25)
        start = None
        for i, flag in enumerate(good):
            if flag and start is None:
                start = i
            if (not flag or i == len(good)-1) and start is not None:
                axis.axvspan(t[start], t[i], color='green', alpha=.08, linewidth=0); start = None
    axes[0].set_title(title+' (green: every joint criterion met after both references stopped)')
    fig.savefig(output/'coordinated-goals.png', dpi=150)
    fig.savefig(output/'coordinated-goals.svg')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--mode', choices=('cycle', 'retarget', 'none'), default='cycle')
    parser.add_argument('--require-goals', action='store_true')
    parser.add_argument('--skip-dynamics', action='store_true', help='Quick research read only; final audits include dynamics')
    parser.add_argument('--compare', action='append', default=[], help='label=run_directory of a matched causal variant')
    parser.add_argument('--plot', action='store_true')
    args = parser.parse_args(); args.output.mkdir(parents=True, exist_ok=False)
    result = audit(args.run, args.output, args.mode, not args.skip_dynamics)
    result['causality'] = []
    for spec in args.compare:
        label, folder = spec.split('=', 1)
        other = args.output/f'variant-{label}'; other.mkdir()
        control = audit(Path(folder), other, 'none', False)
        (other/'certificate.json').write_text(json.dumps(control, indent=2), encoding='utf-8')
        result['causality'].append(compare(args.run, Path(folder), args.output, other, label))
    if args.plot:
        plot(args.output, args.run.name)
    result['pass_all'] = all(result['checks'].values())
    (args.output/'certificate.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps(dict(pass_all=result['pass_all'], checks=result['checks'], commands=[dict(serial=c['serial'], goal_mm=c['goal_offset_mm'], goal_alpha=c['goal_alpha'],
                                                                                     best=c['best'], first=c['first_full_window']) for c in result['commands']],
                          pulses=[dict(delay=p['recovery_delay_s']) for p in result['pulses']], lost=result['lost']), indent=1))
    if args.require_goals and not result['pass_all']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
