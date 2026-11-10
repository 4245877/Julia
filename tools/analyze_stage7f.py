"""Independent reader: reconstruct wrenches/CoP and integrate physical substeps.

Only stdlib; never imports the implementation or modifies simulation data.
"""
import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path


def rows(p):
    with p.open(newline='', encoding='utf-8-sig') as f:
        a = list(csv.DictReader(f))
    assert all(None not in r and all(v is not None for v in r.values()) for r in a), p
    return a


def v(r, names):
    return tuple(float(r[n]) for n in names.split())


def add(a, b):
    return tuple(x+y for x, y in zip(a, b))


def sub(a, b):
    return tuple(x-y for x, y in zip(a, b))


def mul(a, s):
    return tuple(x*s for x in a)


def cross(a, b):
    return a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]


def norm(a):
    return math.sqrt(sum(x*x for x in a))


def vec(r, stem):
    return tuple(float(r[f'{stem}_{a}']) for a in 'xyz')


def audit(directory):
    support = rows(directory/'support-v1.csv')
    states = {int(r['tick']): r for r in rows(directory/'state.csv')}
    dynamics = {int(r['tick']): r for r in rows(directory/'dynamics-v1.csv')}
    points = defaultdict(list)
    for r in rows(directory/'support-points-v1.csv'):
        points[int(r['tick'])].append(r)
    links = defaultdict(dict)
    for r in rows(directory/'contacts.csv'):
        links[int(r['tick'])][r['geom1']] = int(r['link1'])
        links[int(r['tick'])][r['geom2']] = int(r['link2'])
    max_force = max_moment = max_cop = max_integral = max_residual = 0.
    first = {}; loss = defaultdict(int); prior_loaded = {}
    counters = defaultdict(int)
    for r in support:
        tick = int(r['tick']); side = r['side']; geom, floor = r['geom'], r['surface']
        origin = v(r, 'ox_m oy_m oz_m'); force = (0.,)*3; moment = (0.,)*3
        active = records = 0
        for c in points[tick]:
            if {c['geom1'], c['geom2']} != {geom, floor}:
                continue
            records += 1
            if c['active'] != '1' or float(c['fn_N']) <= 1e-9:
                continue
            active += 1
            sign = 1 if c['geom2'] == geom else -1
            f = mul(v(c, 'f2x_N f2y_N f2z_N'), sign)
            t = mul(v(c, 't2x_Nm t2y_Nm t2z_Nm'), sign)
            point = v(c, 'x_m y_m z_m')
            force = add(force, f)
            moment = add(moment, add(cross(sub(point, origin), f), t))
        assert active == int(r['active']) and records == int(r['records']), (directory, tick, side)
        max_force = max(max_force, norm(sub(force, v(r, 'fx_N fy_N fz_N'))))
        max_moment = max(max_moment, norm(sub(moment, v(r, 'mx_Nm my_Nm mz_Nm'))))
        if force[1] >= 20:
            assert r['cop_reason'] == 'available'
            floor_origin = (0., float(r['com_projection_y_m']), 0.)
            transported = add(moment, cross(sub(origin, floor_origin), force))
            expected = add(floor_origin, (transported[2]/force[1], 0., -transported[0]/force[1]))
            max_cop = max(max_cop, norm(sub(expected, v(r, 'cop_x_m cop_y_m cop_z_m'))))
        else:
            assert r['cop_reason'] in ('insufficient_normal_force', 'no_floor')
            assert r['cop_x_m'] == r['cop_y_m'] == r['cop_z_m'] == 'NA'
        if active and side not in first:
            first[side] = float(r['time_s'])
        loaded = r['loaded'] == '1'
        if prior_loaded.get(side, False) and not loaded:
            loss[side] += 1
        prior_loaded[side] = loaded
        for key in ('other_active', 'self_active'):
            counters[key] = max(counters[key], int(r[key]))
        d = dynamics[tick]
        assert norm(sub(v(d, 'vx_m_s vy_m_s vz_m_s'), v(d, 'jvx_m_s jvy_m_s jvz_m_s'))) < 1e-10
    max_environment = 0.
    for tick, d in dynamics.items():
        com = v(d, 'com_x_m com_y_m com_z_m'); force = (0.,)*3; moment = (0.,)*3
        for c in points[tick]:
            l1, l2 = links[tick][c['geom1']], links[tick][c['geom2']]
            if (l1 >= 0) == (l2 >= 0) or c['active'] != '1' or float(c['fn_N']) <= 1e-9:
                continue
            sign = 1 if l2 >= 0 else -1
            f = mul(v(c, 'f2x_N f2y_N f2z_N'), sign)
            t = mul(v(c, 't2x_Nm t2y_Nm t2z_Nm'), sign)
            force = add(force, f)
            moment = add(moment, add(cross(sub(v(c, 'x_m y_m z_m'), com), f), t))
        max_environment = max(max_environment, norm(sub(force, v(d, 'environment_fx_N environment_fy_N environment_fz_N'))),
                              norm(sub(moment, v(d, 'environment_mx_Nm environment_my_Nm environment_mz_Nm'))))
        assert norm(sub(v(d, 'total_fx_N total_fy_N total_fz_N'), add(force, add(v(d, 'gravity_fx_N gravity_fy_N gravity_fz_N'), v(d, 'applied_fx_N applied_fy_N applied_fz_N'))))) < 1e-10
        assert norm(sub(v(d, 'total_mx_Nm total_my_Nm total_mz_Nm'), add(moment, v(d, 'applied_mx_Nm applied_my_Nm applied_mz_Nm')))) < 1e-10
    integral = (0.,)*3; angular = (0.,)*3
    samples = rows(directory/'dynamics-substeps-v1.csv')
    p0, l0 = vec(samples[0], 'p'), vec(samples[0], 'L')
    assert len(samples) == 11521 and float(samples[0]['dt_s']) == 0
    for i, r in enumerate(samples[1:], 1):
        h = float(r['dt_s'])
        assert abs(h-1/1920) < 1e-15
        assert abs(float(r['time_s'])-float(samples[i-1]['time_s'])-h) < 1e-12
        integral = add(integral, mul(add(vec(r, 'force_before'), vec(r, 'force')), .5*h))
        angular = add(angular, mul(add(vec(r, 'moment_before'), vec(r, 'moment')), .5*h))
        if i % 16 == 0:
            d = dynamics[int(r['tick'])]
            max_integral = max(max_integral, norm(sub(integral, vec(d, 'impulse'))), norm(sub(angular, vec(d, 'angular_impulse'))))
            rp = sub(sub(vec(r, 'p'), p0), integral)
            rl = sub(sub(vec(r, 'L'), l0), angular)
            max_residual = max(max_residual, norm(sub(rp, vec(d, 'residual_p'))), norm(sub(rl, vec(d, 'residual_L'))))
            assert norm(sub(vec(r, 'p'), v(states[int(r['tick'])], 'px_kg_m_s py_kg_m_s pz_kg_m_s'))) < 1e-10
    assert max(max_force, max_moment, max_cop, max_integral, max_residual, max_environment) < 1e-10
    return dict(case=directory.name, first_contacts=first, loaded_losses=dict(loss), max_counts=dict(counters),
                force_error_N=max_force, moment_error_Nm=max_moment, cop_error_m=max_cop,
                substep_integral_error=max_integral, residual_error=max_residual,
                environment_wrench_error=max_environment,
                max_linear_residual_Ns=max(norm(vec(r, 'residual_p')) for r in dynamics.values()),
                max_angular_residual_Nms=max(norm(vec(r, 'residual_L')) for r in dynamics.values()))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--cpu', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    a = p.parse_args(); a.output.mkdir(parents=True, exist_ok=False)
    results = [audit(a.cpu/case) for case in ('drop', 'tilt', 'pulse', 'passive', 'internal')]
    (a.output/'independent.json').write_text(json.dumps(results, indent=2), encoding='utf8')
    performance = {}
    for mode in ('on', 'off'):
        r = rows(a.cpu/f'performance-{mode}.csv'); assert len(r) == 10000
        performance[mode] = {}
        for field in ('tick_us', 'observer_us'):
            values = sorted(float(x[field]) for x in r)
            performance[mode][field] = dict(mean=sum(values)/len(values), p95=values[9500], maximum=values[-1])
    (a.output/'performance.json').write_text(json.dumps(performance, indent=2), encoding='utf8')
    (a.output/'audit.txt').write_text('PASS independent force/moment/CoP reconstruction and 57600 physical-substep quadratures; errors <=1e-10.\nPASS 20000 raw performance samples.\n', encoding='utf8')
    print(json.dumps(results, indent=2)); print('PASS independent audit')


if __name__ == '__main__':
    main()
