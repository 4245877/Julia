"""Independent CSV audit and exact comparison against the preserved stage7e baseline."""
import argparse
import csv
import math
import re
import statistics
from pathlib import Path
from verify_stage7c import compare

p = argparse.ArgumentParser()
p.add_argument('--cpu', type=Path, required=True)
p.add_argument('--baseline', type=Path, required=True, help='baseline/cpu-release/artifacts')
p.add_argument('--current', type=Path, required=True, help='publication/cpu-release/artifacts')
p.add_argument('--output', type=Path, required=True)
p.add_argument('--applications', type=Path, help='Optional real-process parity root, audit headless point records too')
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=True)

def rows(path):
    with path.open(newline='') as f:
        return list(csv.DictReader(f))

def number(row, key):
    v = float(row[key])
    assert math.isfinite(v), (key, row)
    return v

with (a.output / 'legacy-comparison.txt').open('w', encoding='utf-8') as report:
    count = 0
    for stage in ['stage7a', 'stage7b', 'stage7c', 'stage7d']:
        for old in sorted((a.baseline / stage / 'cpu').rglob('*.csv')):
            if 'performance' in old.name:
                continue
            new = a.current / old.relative_to(a.baseline)
            result = compare(old, new)
            report.write(f'PASS {old.relative_to(a.baseline)} {result}\n')
            count += 1
    assert count >= 44
    report.write(f'PASS {count} original physical CSV files, all deterministic fields, no golden updates.\n')

with (a.output / 'telemetry-audit.txt').open('w', encoding='utf-8') as report:
    paths=sorted(a.cpu.glob('*/contact-v1.csv'))
    if a.applications:
        paths+=sorted(a.applications.glob('*-headless/contact-v1.csv'))
    for path in paths:
        directory = path.parent
        setup=(directory/'contact-setup.txt').read_text()
        h=float(re.search(r'substep_s=(\S+)',setup)[1])
        contact = rows(path)
        assert len(contact) == 1920
        old = None
        for row in contact:
            t = number(row, 'time_s')
            assert number(row, 'sample_feedback_time_s') <= t-h+1e-10
            for key in ['normal_W','tangent_W','normal_J','tangent_J','geometric_gap_m']:
                number(row,key)
            n=[number(row,'normal_'+axis) for axis in 'xyz']
            f=[number(row,'force_'+axis+'_N') for axis in 'xyz']
            tangent=[number(row,'tangent_'+axis+'_N') for axis in 'xyz']
            assert abs(sum(x*y for x,y in zip(n,f))-number(row,'normal_N'))<1e-10
            assert abs(sum(x*y for x,y in zip(n,tangent)))<1e-10
            assert abs(sum(number(row,'tool_q'+axis)**2 for axis in 'wxyz')-1)<1e-10
            if number(row,'slip_speed_m_s')>=.01:
                assert number(row, 'tangent_W') <= 1e-8, (directory, row['tick'])
            assert row['friction_ratio_valid'] == ('1' if number(row, 'normal_N') >= .6 else '0')
            if row['friction_ratio_valid'] == '0':
                assert row['tangent_normal_ratio'] == 'NA'
            if old and row['phase'] not in ['fault', 'disabled']:
                assert abs(number(row, 'reference_N')-number(old, 'reference_N')) <= .025+1e-10
            old = row
        assert number(contact[-1],'tangent_J')<=1e-9
        limits = {}
        for line in (directory/'calibration.txt').read_text().splitlines():
            match = re.match(r'drive(\d+) kp=.* limit_Nm=(\S+)', line)
            if match:
                limits[int(match[1])] = float(match[2])
        previous = {}
        for row in rows(directory/'drive-v1.csv'):
            k = int(row['coordinate'])
            for key, value in row.items():
                assert math.isfinite(float(value)), (directory, key)
            req, actual, cap = number(row, 'requested_Nm'), number(row, 'actual_Nm'), limits[k]
            assert abs(actual-max(-cap, min(cap, req))) < 1e-12
            assert abs(number(row, 'power_W')-actual*number(row, 'v_rad_s')) < 1e-10
            assert abs(number(row, 'net_J')-number(row, 'delivered_J')+number(row, 'absorbed_J')) < 1e-10
            assert row['reference_reached'] == '0'
            if k in previous:
                for key in ['delivered_J', 'absorbed_J', 'saturated_s']:
                    assert number(row,key) >= number(previous[k],key)-1e-12
            previous[k] = row
        # Each point row independently reconstructs the world force from all
        # three contact axes; no assumption about a fixed solver contact index.
        point_file = directory/'contact-points-v1.csv'
        if point_file.exists():
            for c in rows(point_file):
                for axis in 'xyz':
                    computed = sum(number(c, basis+axis)*number(c, f) for basis, f in [('n','fn_N'),('t1','ft1_N'),('t2','ft2_N')])
                    assert abs(computed-number(c,'f2'+axis+'_N')) < 1e-10
                    assert abs(number(c,'f2'+axis+'_N')+number(c,'f1'+axis+'_N')) < 1e-12
        positive=max(number(x,'tangent_W') for x in contact)
        report.write(f'PASS {directory.name} force/time/ratio/torque/power/work/reference audit; max signed tangent power_W={positive:.12g} (soft near-stiction need not be instantaneously dissipative)\n')
    report.write('PASS selected-pair normal/tangent work is separate from active/passive/external work.\n')

with (a.output / 'summary.csv').open('w', newline='', encoding='utf-8') as stream:
    writer = csv.writer(stream)
    writer.writerow(['run','phase','failure','hold_samples','hold_RMS_N','hold_peak_error_N','peak_N','peak_penetration_mm','peak_slip_mm_s','max_x_drift_mm','normal_J','tangent_J','settle_3_s','settle_6_s'])
    for path in sorted(a.cpu.glob('*/contact-v1.csv')):
        r = rows(path)
        hold = [x for x in r if x['phase']=='hold' and number(x,'phase_s')>=1.5]
        errors = [abs(number(x,'normal_N')-number(x,'goal_N')) for x in hold]
        settles=[]
        for goal in [3,6]:
            good = [x for x in r if x['phase']=='hold' and number(x,'goal_N')==goal and number(x,'good_hold_s')>=.5]
            settles.append(min(number(x,'phase_s')-number(x,'good_hold_s') for x in good) if good else 'NA')
        end=r[-1]
        writer.writerow([path.parent.name,end['phase'],end['failure'],len(hold),math.sqrt(statistics.mean(e*e for e in errors)) if errors else 'NA',max(errors,default=0),end['peak_N'],number(end,'peak_penetration_m')*1000,max(number(x,'slip_speed_m_s') for x in r)*1000,max(abs(number(x,'tool_x_m')-number(r[0],'tool_x_m')) for x in r)*1000,end['normal_J'],end['tangent_J'],*settles])
print('PASS CSV audit, summary, preserved baseline comparison')
