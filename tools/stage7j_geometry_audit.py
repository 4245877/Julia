"""Post-freeze independent actual-endpoint FK/COM/geometry audit for stage7j.

Uses only exported MJCF, state/joints CSV and scratch MuJoCo 3.3.7 data. It
does not import Julia, the controller, a QP solver or the existing auditors.
All completed ticks are checked; no live world state is changed.
"""
import os
os.environ['OPENBLAS_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'
from pathlib import Path
import argparse
import csv
import hashlib
import json
import math
import sys
import numpy as np


def rows(path):
    with path.open(newline='', encoding='utf-8-sig') as stream:
        yield from csv.DictReader(stream)


def vector(row, fields):
    return np.array([float(row[field]) for field in fields.split()])


def rotation_angle(rotation):
    return math.acos(float(np.clip((np.trace(rotation)-1.)*.5, -1., 1.)))


def audit(run, output):
    sys.path.insert(0, str(Path('build/diagnostics/stage7g/python-deps').resolve()))
    import mujoco as mj
    assert mj.__version__ == '3.3.7'
    m = mj.MjModel.from_xml_path(str((run/'model.xml').resolve()))
    d = mj.MjData(m)
    assert (m.nq, m.nv, m.nu, m.neq) == (48, 47, 41, 0)
    mass = float(m.body_mass.sum()); assert abs(mass-55.) < 1e-10
    root = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, 'root')
    body = int(m.jnt_bodyid[root]); rq, rv = int(m.jnt_qposadr[root]), int(m.jnt_dofadr[root])
    joint = [mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, f'j{k}') for k in range(41)]
    qa, va = m.jnt_qposadr[joint], m.jnt_dofadr[joint]
    limits = m.jnt_range[joint].copy()
    geoms = [mj.mj_name2id(m, mj.mjtObj.mjOBJ_GEOM, f'g{k}') for k in (16, 19)]
    floor = mj.mj_name2id(m, mj.mjtObj.mjOBJ_GEOM, 'floor')
    support = {(int(r['tick']), r['side']): r for r in rows(run/'support-v1.csv')}
    dynamics = {int(r['tick']): r for r in rows(run/'dynamics-v1.csv')}
    loads = {int(r['tick']): r for r in rows(run/'load-v1.csv')}
    joint_rows = iter(rows(run/'joints.csv'))
    jac = np.zeros((3, 47))
    initial_feet = None
    peaks = dict(com_reconstruction_m=0., com_velocity_vs_momentum_m_s=0., com_velocity_vs_dynamics_m_s=0.,
                 load_com_error_reconstruction_m=0., load_speed_reconstruction_m_s=0.,
                 sole_origin_reconstruction_m=0., gap_reconstruction_m=0., penetration_reconstruction_m=0.,
                 joint_violation_reconstruction_rad=0., joint_limit_violation_rad=0.,
                 max_support_penetration_m=0., max_actual_floor_penetration_m=0.,
                 max_com_horizontal_error_m=0., max_com_horizontal_speed_m_s=0.)
    extrema = {side: dict(min_corner_height_m=float('inf'), max_corner_height_m=-float('inf'),
                          max_normal_tilt_rad=0., max_rotation_from_initial_rad=0., max_sole_displacement_m=0.) for side in ('LeftFoot', 'RightFoot')}
    failures = []
    count = 0
    fields = ['tick', 'time_s', 'side', 'com_x_m', 'com_y_m', 'com_z_m', 'com_vx_m_s', 'com_vy_m_s', 'com_vz_m_s',
              'com_error_m', 'com_speed_m_s', 'corner0_height_m', 'corner1_height_m', 'corner2_height_m', 'corner3_height_m',
              'normal_tilt_rad', 'rotation_from_initial_rad', 'sole_displacement_m', 'minimum_box_height_m',
              'contact_penetration_m', 'joint_limit_violation_rad', 'geometry_valid']
    with (output/'actual-geometry-v1.csv').open('w', newline='') as stream:
        writer = csv.writer(stream); writer.writerow(fields)
        for state in rows(run/'state.csv'):
            tick = int(state['tick']); count += 1
            assert tick == count-1
            q = np.zeros(41); velocity = np.zeros(41); reported_limits = np.zeros(41)
            for k in range(41):
                row = next(joint_rows)
                assert int(row['tick']) == tick and int(row['coordinate']) == k
                q[k] = float(row['q_rad']); velocity[k] = float(row['v_rad_s']); reported_limits[k] = float(row['limit_rad'])
            d.qpos[rq:rq+3] = vector(state, 'root_x_m root_y_m root_z_m')
            d.qpos[rq+3:rq+7] = vector(state, 'qw qx qy qz')
            d.qvel[rv:rv+3] = vector(state, 'vx_world_m_s vy_world_m_s vz_world_m_s')
            d.qvel[rv+3:rv+6] = vector(state, 'wx_local_rad_s wy_local_rad_s wz_local_rad_s')
            d.qpos[qa] = q; d.qvel[va] = velocity
            mj.mj_forward(m, d)
            actual_com = (m.body_mass[:, None]*d.xipos).sum(axis=0)/mass
            mj.mj_jacSubtreeCom(m, d, jac, body)
            actual_velocity = jac@d.qvel
            peaks['com_reconstruction_m'] = max(peaks['com_reconstruction_m'], float(np.linalg.norm(actual_com-vector(state, 'com_x_m com_y_m com_z_m'))))
            peaks['com_velocity_vs_momentum_m_s'] = max(peaks['com_velocity_vs_momentum_m_s'], float(np.linalg.norm(actual_velocity-vector(state, 'px_kg_m_s py_kg_m_s pz_kg_m_s')/mass)))
            peaks['com_velocity_vs_dynamics_m_s'] = max(peaks['com_velocity_vs_dynamics_m_s'], float(np.linalg.norm(actual_velocity-vector(dynamics[tick], 'vx_m_s vy_m_s vz_m_s'))))
            goal = vector(loads[tick], 'initial_com_x initial_com_y initial_com_z')
            com_error = float(np.linalg.norm((actual_com-goal)[[0, 2]]))
            com_speed = float(np.linalg.norm(actual_velocity[[0, 2]]))
            peaks['load_com_error_reconstruction_m'] = max(peaks['load_com_error_reconstruction_m'], abs(com_error-float(loads[tick]['com_error_m'])))
            peaks['load_speed_reconstruction_m_s'] = max(peaks['load_speed_reconstruction_m_s'], abs(com_speed-float(loads[tick]['speed_m_s'])))
            peaks['max_com_horizontal_error_m'] = max(peaks['max_com_horizontal_error_m'], com_error)
            peaks['max_com_horizontal_speed_m_s'] = max(peaks['max_com_horizontal_speed_m_s'], com_speed)
            violation = np.maximum(0., np.maximum(limits[:, 0]-q, q-limits[:, 1]))
            max_joint_violation = float(violation.max())
            peaks['joint_limit_violation_rad'] = max(peaks['joint_limit_violation_rad'], max_joint_violation)
            peaks['joint_violation_reconstruction_rad'] = max(peaks['joint_violation_reconstruction_rad'], float(np.abs(violation-reported_limits).max()))
            floor_rotation = d.geom_xmat[floor].reshape(3, 3)
            normal = floor_rotation[:, 2]
            assert np.linalg.norm(normal-[0., 1., 0.]) < 1e-12
            floor_origin = d.geom_xpos[floor].copy()
            current_feet = []
            for side, geom in zip(('LeftFoot', 'RightFoot'), geoms):
                foot = support[(tick, side)]
                rotation = d.geom_xmat[geom].reshape(3, 3)
                size = m.geom_size[geom]
                # BodyRig.cpp defines Box = semantic Sole * T(0,.025,-.035).
                # The controller's displacement guard uses that original Sole,
                # not the bottom centre of the physical box. Box corners below
                # remain derived directly from the exported compiled geom size.
                sole = d.geom_xpos[geom]+rotation@np.array([0., -size[1], .035])
                current_feet.append((sole.copy(), rotation.copy()))
                if initial_feet is None:
                    initial_sole, initial_rotation = sole, rotation
                else:
                    initial_sole, initial_rotation = initial_feet[len(current_feet)-1]
                heights = [float(normal@(d.geom_xpos[geom]+rotation@np.array([size[0] if c & 1 else -size[0], -size[1], size[2] if c & 2 else -size[2]])-floor_origin)) for c in range(4)]
                full_box_heights = [float(normal@(d.geom_xpos[geom]+rotation@(size*np.array([x, y, z]))-floor_origin)) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
                tilt = math.acos(float(np.clip(normal@rotation[:, 1], -1., 1.)))
                rotation_delta = rotation_angle(initial_rotation@rotation.T)
                displacement = float(np.linalg.norm((sole-initial_sole)[[0, 2]]))
                penetration = max([0.]+[-float(c.dist) for c in d.contact if {int(c.geom1), int(c.geom2)} == {floor, geom}])
                e = extrema[side]
                e['min_corner_height_m'] = min(e['min_corner_height_m'], min(heights))
                e['max_corner_height_m'] = max(e['max_corner_height_m'], max(heights))
                e['max_normal_tilt_rad'] = max(e['max_normal_tilt_rad'], tilt)
                e['max_rotation_from_initial_rad'] = max(e['max_rotation_from_initial_rad'], rotation_delta)
                e['max_sole_displacement_m'] = max(e['max_sole_displacement_m'], displacement)
                peaks['sole_origin_reconstruction_m'] = max(peaks['sole_origin_reconstruction_m'], float(np.linalg.norm(sole-vector(foot, 'ox_m oy_m oz_m'))))
                peaks['gap_reconstruction_m'] = max(peaks['gap_reconstruction_m'], abs(min(full_box_heights)-float(foot['gap_m'])))
                peaks['penetration_reconstruction_m'] = max(peaks['penetration_reconstruction_m'], abs(penetration-float(foot['penetration_m'])))
                peaks['max_support_penetration_m'] = max(peaks['max_support_penetration_m'], float(foot['penetration_m']))
                peaks['max_actual_floor_penetration_m'] = max(peaks['max_actual_floor_penetration_m'], penetration)
                valid = min(heights) >= -.004 and max(heights) <= .002 and tilt <= .05 and rotation_delta <= .05 and displacement <= .010 and max_joint_violation <= .005 and penetration <= .004
                if not valid and len(failures) < 20:
                    failures.append(dict(tick=tick, side=side, heights_m=heights, normal_tilt_rad=tilt, rotation_from_initial_rad=rotation_delta, displacement_m=displacement, hinge_violation_rad=max_joint_violation, penetration_m=penetration))
                writer.writerow([tick, state['time_s'], side, *actual_com, *actual_velocity, com_error, com_speed, *heights, tilt, rotation_delta, displacement, min(full_box_heights), penetration, max_joint_violation, valid])
            if initial_feet is None:
                initial_feet = current_feet
    assert next(joint_rows, None) is None
    checks = dict(actual_geometry=not failures,
                  actual_com_reconstruction=peaks['com_reconstruction_m'] < 1e-10,
                  actual_velocity_reconstruction=max(peaks['com_velocity_vs_momentum_m_s'], peaks['com_velocity_vs_dynamics_m_s']) < 1e-10,
                  published_com_metrics=max(peaks['load_com_error_reconstruction_m'], peaks['load_speed_reconstruction_m_s']) < 1e-10,
                  actual_foot_geometry_reconstruction=max(peaks['sole_origin_reconstruction_m'], peaks['gap_reconstruction_m'], peaks['penetration_reconstruction_m']) < 1e-10,
                  original_joint_limits=peaks['joint_limit_violation_rad'] <= .005 and peaks['joint_violation_reconstruction_rad'] < 1e-10)
    return dict(schema='stage7j-independent-actual-geometry-v1', run=str(run), reader_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                method='All exported completed tick actual q/root/v states reconstructed in separate MuJoCo 3.3.7 mjData; mass-weighted inertial positions and COM Jacobian; physical box corners/floor/contact distances; original hinge limits',
                mujoco_version=mj.__version__, model_sha256=hashlib.sha256((run/'model.xml').read_bytes()).hexdigest(),
                semantic_sole_from_box_translation_m=[0., -.025, .035], semantic_sole_source='libs/physics/src/BodyRig.cpp: Box = Sole * translate(0,.025,-.035)',
                semantic_sole_source_sha256=hashlib.sha256(Path('libs/physics/src/BodyRig.cpp').read_bytes()).hexdigest(),
                files_sha256={name: hashlib.sha256((run/name).read_bytes()).hexdigest() for name in ('state.csv', 'joints.csv', 'support-v1.csv', 'dynamics-v1.csv', 'load-v1.csv')},
                completed_ticks=count-1, samples_including_initial=count, original_mass_kg=mass, peaks=peaks, feet=extrema,
                first_failures=failures, checks=checks, pass_all=all(checks.values()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args(); args.output.mkdir(parents=True, exist_ok=False)
    result = audit(args.run, args.output)
    (args.output/'certificate.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps(result, indent=2))
    if not result['pass_all']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
