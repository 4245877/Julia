"""Independent static LP witnesses for stage7k COM/alpha pairs; never executes Julia control.

Each case names an exported solver pose (run directory + completed tick) and an
absolute left-load fraction alpha. The pose (root + 41 hinges) is loaded into a
scratch MuJoCo 3.3.7 model built from the exported MJCF, all velocities and
accelerations are zeroed, and an HiGHS LP searches 24 floor-frame corner forces
(same 90x170 mm corner rectangle, inner friction pyramid |fx|+|fz|<=0.6fy,
fy>=0, >=30 N per foot) that satisfy the six unactuated free-root equilibrium
equations with every hinge torque inside 95% of its original cap. Two LPs are
solved per case: the left-fraction range at that pose and the witness with the
smallest peak |tau|/cap at the requested alpha. This certifies static
equilibrium of a frozen pose only; it does not prove dynamic transitions.
"""
import os
os.environ['OPENBLAS_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'
from pathlib import Path
import argparse
import csv
import hashlib
import json
import sys
import numpy as np
from scipy.optimize import linprog


def rows(path):
    with path.open(newline='', encoding='utf-8-sig') as stream:
        return list(csv.DictReader(stream))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_case(text):
    # label=run_directory::tick::alpha_offset (offset relative to --alpha0)
    label, rest = text.split('=', 1)
    run, tick, offset = rest.split('::')
    return dict(label=label, run=Path(run), tick=int(tick), offset=float(offset))


def build(mj, run, tick):
    model = run/'model.xml'
    solver = run/'com-solver-v1.csv'
    selected = [r for r in rows(solver) if int(r['tick']) == tick]
    assert len(selected) == 1, ('Exactly one exported solver pose required', str(run), tick)
    r = selected[0]
    m = mj.MjModel.from_xml_path(str(model.resolve()))
    d = mj.MjData(m)
    assert (m.nq, m.nv, m.nu, m.neq) == (48, 47, 41, 0) and abs(float(m.body_mass.sum())-55.) < 1e-10
    joints = [mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, f'j{k}') for k in range(41)]
    qa, va = m.jnt_qposadr[joints], m.jnt_dofadr[joints]
    root = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, 'root')
    rq, rv = int(m.jnt_qposadr[root]), int(m.jnt_dofadr[root])
    d.qpos[rq:rq+3] = [float(r['root_p'+x]) for x in 'xyz']
    d.qpos[rq+3:rq+7] = [float(r['root_q'+x]) for x in 'wxyz']
    d.qpos[qa] = [float(r[f'q{k}']) for k in range(41)]
    d.qvel[:] = 0.
    mj.mj_forward(m, d)
    caps = np.array([m.actuator_ctrlrange[mj.mj_name2id(m, mj.mjtObj.mjOBJ_ACTUATOR, f'motor{k}'), 1] for k in range(41)])
    bias = d.qfrc_bias-d.qfrc_passive
    generalized = np.zeros((47, 24))
    points = []
    frames = []
    jp = np.zeros((3, 47))
    for side, link in enumerate((16, 19)):
        geom = mj.mj_name2id(m, mj.mjtObj.mjOBJ_GEOM, f'g{link}')
        rotation = d.geom_xmat[geom].reshape(3, 3)
        x = rotation[:, 0].copy(); x[1] = 0.; x /= np.linalg.norm(x)
        frame = np.column_stack((x, [0., 1., 0.], np.cross(x, [0., 1., 0.])))
        frames.append(frame)
        for c in range(4):
            point = d.geom_xpos[geom]+rotation@np.array([.045 if c & 1 else -.045, -.025, .085 if c & 2 else -.085])
            points.append(point)
            mj.mj_jac(m, d, jp, None, point, int(m.geom_bodyid[geom]))
            generalized[:, side*12+c*3:side*12+c*3+3] = jp.T@frame
    com = d.subtree_com[int(m.jnt_bodyid[root])].copy()
    return dict(model=model, solver=solver, m=m, d=d, qa=qa, va=va, rv=rv, caps=caps, bias=bias,
                generalized=generalized, points=points, frames=frames, com=com, qpos=d.qpos.copy())


def solve_case(pose, alpha):
    G, bias, caps, va, rv = pose['generalized'], pose['bias'], pose['caps'], pose['va'], pose['rv']
    left, right = np.zeros(24), np.zeros(24)
    left[np.arange(1, 12, 3)] = 1.; right[np.arange(13, 24, 3)] = 1.
    # Variables: 24 corner forces and one peak utilization u (minimized in the witness LP).
    friction = []
    for c in range(8):
        for sx in (-1, 1):
            for sz in (-1, 1):
                row = np.zeros(24); row[c*3:c*3+3] = [sx, -.6, sz]; friction.append(row)
    equality = G[rv:rv+6]
    target = bias[rv:rv+6]
    bounds = [(None, None), (0., None), (None, None)]*8
    # Range at this pose with the 95% caps (as stage7j).
    A, b = [], []
    for k, v in enumerate(va):
        A.extend((-G[v], G[v])); b.extend((.95*caps[k]-bias[v], .95*caps[k]+bias[v]))
    A.extend(friction); b.extend([0.]*len(friction))
    A.extend((-left, -right)); b.extend((-30., -30.))
    A, b = np.array(A), np.array(b)
    lo = linprog(left/100., A_ub=A, b_ub=b, A_eq=equality, b_eq=target, bounds=bounds, method='highs')
    hi = linprog(-left/100., A_ub=A, b_ub=b, A_eq=equality, b_eq=target, bounds=bounds, method='highs')
    def fraction(x):
        return float(left@x)/float((left+right)@x)
    out = dict(range_feasible=bool(lo.success and hi.success),
               alpha_min=fraction(lo.x) if lo.success else None, alpha_max=fraction(hi.x) if hi.success else None)
    # Minimum peak utilization witness at the requested alpha.
    Au, bu = [], []
    for k, v in enumerate(va):
        Au.append(np.r_[-G[v], -caps[k]]); bu.append(-bias[v])     # tau=bias-Gf <= u cap
        Au.append(np.r_[G[v], -caps[k]]); bu.append(bias[v])       # -tau <= u cap
    for row in friction:
        Au.append(np.r_[row, 0.]); bu.append(0.)
    Au.append(np.r_[-left, 0.]); bu.append(-30.)
    Au.append(np.r_[-right, 0.]); bu.append(-30.)
    Au, bu = np.array(Au), np.array(bu)
    Ae = np.vstack((np.c_[equality, np.zeros(6)], np.r_[left-alpha*(left+right), 0.]))
    be = np.r_[target, 0.]
    objective = np.zeros(25); objective[24] = 1.
    result = linprog(objective, A_ub=Au, b_ub=bu, A_eq=Ae, b_eq=be, bounds=bounds+[(0., .95)], method='highs')
    out.update(requested_alpha=alpha, witness_feasible=bool(result.success), scipy_status=int(result.status), message=result.message)
    if result.success:
        f = result.x[:24]
        tau = bias[va]-G[va]@f
        fl, fr = float(left@f), float(right@f)
        out.update(alpha=fl/(fl+fr), left_N=fl, right_N=fr, minimum_peak_utilization=float(np.max(np.abs(tau)/caps)),
                   force_floor_frame_N=f.tolist(),
                   force_world_N=[(pose['frames'][s]@f[s*12+c*3:s*12+c*3+3]).tolist() for s in range(2) for c in range(4)],
                   torque_Nm=tau.tolist(), torque_caps_Nm=caps.tolist(),
                   root_residual=float(np.max(np.abs(equality@f-target))),
                   friction_violation_N=float(max(0., max(row@f for row in friction))),
                   min_corner_normal_N=float(min(f[1::3])))
        assert out['root_residual'] < 1e-7 and out['friction_violation_N'] < 1e-7 and out['min_corner_normal_N'] >= -1e-9
        assert abs(out['alpha']-alpha) < 1e-9 and out['minimum_peak_utilization'] <= .950000001 and min(fl, fr) >= 30.-1e-7
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--alpha0', type=float)
    parser.add_argument('--alpha0-from-run', type=Path, help='Read measured alpha0 and initial COM goal from a stage7k coordinated-v1.csv run')
    parser.add_argument('--initial-com', type=float, nargs=3, metavar=('X', 'Y', 'Z'), help='Frozen initial COM goal, world m')
    parser.add_argument('--case', action='append', required=True, help='label=run::tick::alpha_offset')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.alpha0_from_run:
        calibrated = next(r for r in rows(args.alpha0_from_run/'coordinated-v1.csv') if r['baseline_valid'] == '1')
        args.alpha0 = float(calibrated['alpha0'])
        args.initial_com = [float(calibrated[f'initial_com_{k}']) for k in 'xyz']
    assert args.alpha0 is not None, 'alpha0 or --alpha0-from-run required'
    if not np.isfinite(args.alpha0) or not .05 < args.alpha0 < .95:
        raise ValueError('Finite interior measured alpha0 required')
    args.output.mkdir(parents=True, exist_ok=False)
    sys.path.insert(0, str(Path('build/diagnostics/stage7g/python-deps').resolve()))
    import mujoco as mj
    assert mj.__version__ == '3.3.7'
    cases = [parse_case(text) for text in args.case]
    results = []
    for case in cases:
        pose = build(mj, case['run'], case['tick'])
        alpha = args.alpha0+case['offset']
        out = solve_case(pose, alpha)
        offset = None
        if args.initial_com:
            delta = pose['com']-np.array(args.initial_com)
            offset = [float(delta[0]), float(delta[2])]
        out.update(label=case['label'], run=str(case['run']), tick=case['tick'], alpha_offset=case['offset'],
                   model_sha256=sha(pose['model']), solver_csv_sha256=sha(pose['solver']),
                   pose_com_world_m=pose['com'].tolist(), pose_com_offset_from_initial_xz_m=offset,
                   qpos=pose['qpos'].tolist(), corner_points_world_m=np.array(pose['points']).tolist())
        results.append(out)
        print(case['label'], 'range', out['alpha_min'], out['alpha_max'], 'alpha', alpha,
              'feasible', out['witness_feasible'], 'peak', out.get('minimum_peak_utilization'), 'com_offset_mm',
              None if offset is None else [1000*x for x in offset], flush=True)
    report = dict(method=__doc__.strip(), dynamic_control_proven=False, mujoco_version=mj.__version__, alpha0=args.alpha0,
                  initial_com=args.initial_com, cap_fraction_bound=.95,
                  all_feasible=all(r['witness_feasible'] for r in results),
                  max_minimum_peak_utilization=max((r.get('minimum_peak_utilization', float('inf')) for r in results)),
                  cases=results)
    (args.output/'certificate.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    with (args.output/'witnesses.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['label', 'run', 'tick', 'feasible', 'requested_alpha', 'alpha', 'alpha_min', 'alpha_max', 'left_N', 'right_N',
                         'minimum_peak_utilization', *[f'f{i}_{axis}_N' for i in range(8) for axis in 'xyz'], *[f'tau{k}_Nm' for k in range(41)]])
        for w in results:
            writer.writerow([w['label'], w['run'], w['tick'], w['witness_feasible'], w['requested_alpha'], w.get('alpha'), w['alpha_min'], w['alpha_max'],
                             w.get('left_N'), w.get('right_N'), w.get('minimum_peak_utilization'), *w.get('force_floor_frame_N', []), *w.get('torque_Nm', [])])
    if not report['all_feasible']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
