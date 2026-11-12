"""Independent static LP witnesses for stage7j; never executes Julia control.

Reads an exported solver pose, zeros all velocities/accelerations, and uses
MuJoCo 3.3.7 scratch data. This certifies frozen-pose equilibrium only.
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--tick', type=int, default=240)
    parser.add_argument('--alpha0', type=float, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not np.isfinite(args.alpha0) or not .05 < args.alpha0 < .95:
        raise ValueError('Finite interior measured alpha0 required')
    args.output.mkdir(parents=True, exist_ok=False)
    sys.path.insert(0, str(Path('build/diagnostics/stage7g/python-deps').resolve()))
    import mujoco as mj
    assert mj.__version__ == '3.3.7'
    model = args.run/'model.xml'
    solver = args.run/'com-solver-v1.csv'
    selected = [r for r in rows(solver) if int(r['tick']) == args.tick]
    assert len(selected) == 1, 'Exactly one exported solver pose required'
    r = selected[0]
    m = mj.MjModel.from_xml_path(str(model.resolve()))
    d = mj.MjData(m)
    assert (m.nq, m.nv, m.nu, m.neq) == (48, 47, 41, 0)
    joints = [mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, f'j{k}') for k in range(41)]
    qa = m.jnt_qposadr[joints]
    va = m.jnt_dofadr[joints]
    root = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, 'root')
    rq, rv = m.jnt_qposadr[root], m.jnt_dofadr[root]
    d.qpos[rq:rq+3] = [float(r['root_p'+x]) for x in 'xyz']
    d.qpos[rq+3:rq+7] = [float(r['root_q'+x]) for x in 'wxyz']
    d.qpos[qa] = [float(r[f'q{k}']) for k in range(41)]
    d.qvel[:] = 0.
    mj.mj_forward(m, d)
    caps = np.array([m.actuator_ctrlrange[mj.mj_name2id(m, mj.mjtObj.mjOBJ_ACTUATOR, f'motor{k}'), 1] for k in range(41)])
    bias = d.qfrc_bias-d.qfrc_passive
    generalized = np.zeros((47, 24))
    points, frames = [], []
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
    # inequalities A f <= b: 95% original actuator caps, unilateral corners,
    # same conservative friction diamond, and 30 N minimum per foot.
    A, b = [], []
    for k, v in enumerate(va):
        A.extend((-generalized[v], generalized[v]))
        b.extend((.95*caps[k]-bias[v], .95*caps[k]+bias[v]))
    left, right = np.zeros(24), np.zeros(24)
    left[np.arange(1, 12, 3)] = 1.; right[np.arange(13, 24, 3)] = 1.
    for c in range(8):
        for sx in (-1, 1):
            for sz in (-1, 1):
                row = np.zeros(24); row[c*3:c*3+3] = [sx, -.6, sz]
                A.append(row); b.append(0.)
    A.extend((-left, -right)); b.extend((-30., -30.))
    A, b = np.array(A), np.array(b)
    equality = generalized[rv:rv+6]
    target = bias[rv:rv+6]
    bounds = [(None, None), (0., None), (None, None)]*8
    witnesses = []

    def solve(label, objective, alpha=None):
        eq, rhs = equality, target
        if alpha is not None:
            eq = np.vstack((eq, left-alpha*(left+right)))
            rhs = np.r_[rhs, 0.]
        result = linprog(objective, A_ub=A, b_ub=b, A_eq=eq, b_eq=rhs, bounds=bounds, method='highs')
        out = dict(label=label, requested_alpha=alpha, scipy_status=int(result.status), message=result.message, feasible=bool(result.success))
        if result.success:
            f = result.x
            tau = bias[va]-generalized[va]@f
            fl, fr = float(left@f), float(right@f)
            out.update(alpha=fl/(fl+fr), left_N=fl, right_N=fr, force_floor_frame_N=f.tolist(),
                       force_world_N=[(frames[s]@f[s*12+c*3:s*12+c*3+3]).tolist() for s in range(2) for c in range(4)],
                       torque_Nm=tau.tolist(), torque_caps_Nm=caps.tolist(), peak_utilization=float(np.max(np.abs(tau)/caps)),
                       root_residual=float(np.max(np.abs(equality@f-target))), inequality_violation=float(max(0., np.max(A@f-b))))
            assert out['root_residual'] < 1e-7 and out['inequality_violation'] < 1e-7
            assert out['peak_utilization'] <= .950000001 and min(fl, fr) >= 30.-1e-7
        witnesses.append(out)
        return out

    minimum = solve('minimum_left', left)
    maximum = solve('maximum_left', -left)
    for label, alpha in (('baseline', args.alpha0), ('plus_005', args.alpha0+.05), ('minus_005', args.alpha0-.05)):
        solve(label, np.zeros(24), alpha)
    report = dict(method='Independent scipy HiGHS LP; frozen exported pose; zero velocity/acceleration; same rectangle and inner friction pyramid; 95% torque caps; >=30N each foot',
                  dynamic_control_proven=False, mujoco_version=mj.__version__, model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
                  solver_csv_sha256=hashlib.sha256(solver.read_bytes()).hexdigest(), tick=args.tick, alpha0=args.alpha0,
                  qpos=d.qpos.tolist(), qvel=d.qvel.tolist(), gravity=m.opt.gravity.tolist(), mass_kg=float(m.body_mass.sum()),
                  corner_points_world_m=np.array(points).tolist(), normal_range=[minimum.get('alpha'), maximum.get('alpha')],
                  all_three_targets_static_feasible=all(x['feasible'] for x in witnesses[2:]), witnesses=witnesses)
    (args.output/'certificate.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    with (args.output/'witnesses.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['label', 'feasible', 'alpha', 'left_N', 'right_N', 'peak_utilization', *[f'f{i}_{axis}_N' for i in range(8) for axis in 'xyz'], *[f'tau{k}_Nm' for k in range(41)]])
        for w in witnesses:
            writer.writerow([w['label'], w['feasible'], w.get('alpha'), w.get('left_N'), w.get('right_N'), w.get('peak_utilization'), *w.get('force_floor_frame_N', []), *w.get('torque_Nm', [])])
    print(json.dumps({k: v for k, v in report.items() if k not in ('qpos', 'qvel', 'corner_points_world_m', 'witnesses')}, indent=2))
    if not report['all_three_targets_static_feasible']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
