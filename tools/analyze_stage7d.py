"""Audit the published drive contract independently from recorded C++ output."""
import argparse
import csv
import math
from pathlib import Path
import re
import statistics


def audit(directory):
    calibration = (directory / 'calibration.txt').read_text(encoding='utf-8')
    limits = {}
    for line in calibration.splitlines():
        arm = re.match(r'drive(\d+) kp=.* limit_Nm=(\S+)', line)
        body = re.search(r' q=(\d+).* PD=[^,]+,[^,]+,(\S+)', line)
        match = arm or body
        if match:
            limits[int(match[1])] = float(match[2])
    substeps = 16 if 'nu=41' in calibration else 8
    previous = {}
    maximum = dict(power=0., request=0., actual=0., acceleration=0., speed=0.)
    rows = 0
    with (directory / 'drive-v1.csv').open(newline='') as stream:
        for text in csv.DictReader(stream):
            r = {key: float(value) for key, value in text.items()}
            assert all(math.isfinite(value) for value in r.values()), (directory, 'nonfinite')
            k, tick = int(r['coordinate']), int(r['tick'])
            assert r['schema'] == 1
            if tick:
                power_error = abs(r['power_W'] - r['actual_Nm']*r['v_rad_s'])
                request_error = abs(r['requested_Nm'] - r['p_Nm']-r['d_Nm']-r['ff_Nm'])
                actual_error = abs(r['actual_Nm']-r['limited_Nm'])
                maximum['power'] = max(maximum['power'], power_error)
                maximum['request'] = max(maximum['request'], request_error)
                maximum['actual'] = max(maximum['actual'], actual_error)
                assert power_error < 1e-10 and request_error < 1e-10 and actual_error < 1e-12, (directory, tick, k)
                assert abs(r['time_s']-r['sample_time_s']-1/(120*substeps)) < 1e-12
                assert abs(r['limited_Nm']-max(-limits[k], min(limits[k], r['requested_Nm']))) < 1e-12
                assert bool(r['saturated_last']) == (abs(r['requested_Nm']) > limits[k])
            assert r['delivered_J'] >= 0 and r['absorbed_J'] >= 0
            assert abs(r['net_J']-r['delivered_J']+r['absorbed_J']) < 1e-10
            assert abs(r['error_rad']-r['q_ref_rad']+r['q_rad']) < 1e-12
            assert abs(r['error_rad_s']-r['v_ref_rad_s']+r['v_rad_s']) < 1e-12
            maximum['speed'] = max(maximum['speed'], abs(r['v_ref_rad_s']))
            assert abs(r['v_ref_rad_s']) <= r['max_ref_speed_rad_s']+1e-10
            if k in previous:
                old = previous[k]
                assert tick == old['tick']+1
                for field in ('delivered_J', 'absorbed_J', 'saturated_s'):
                    assert r[field] >= old[field]-1e-12
                assert r['saturated_s']-old['saturated_s'] <= 1/120+1e-12
                acceleration = abs(r['v_ref_rad_s']-old['v_ref_rad_s'])*120
                maximum['acceleration'] = max(maximum['acceleration'], acceleration)
                assert acceleration <= r['max_ref_acceleration_rad_s2']+1e-9
            previous[k] = r
            rows += 1
    delivered = sum(r['delivered_J'] for r in previous.values())
    absorbed = sum(r['absorbed_J'] for r in previous.values())
    last = previous[0]
    return (f'PASS {directory.name} rows={rows} max_errors={maximum} delivered_J={delivered:.12g} '
            f'absorbed_J={absorbed:.12g} passive_J={last["total_passive_damping_J"]:.12g} '
            f'external_J={last["total_external_J"]:.12g} mechanical_delta_J={last["mechanical_energy_change_J"]:.12g}')


def stats(values):
    values = sorted(values)
    # Same upper order statistic as the C++ diagnostics (zero-based floor(.95*N)).
    return statistics.mean(values), values[len(values)*95//100], values[-1]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--root', type=Path, default=Path('build/diagnostics/stage7d'))
    p.add_argument('--measurements', type=Path)
    p.add_argument('--performance-report', default='performance-summary.txt')
    a = p.parse_args()
    measurements = a.measurements or a.root / 'cpu-publication'
    with (a.root / 'telemetry-audit.txt').open('w', encoding='utf-8') as report:
        for directory in sorted((a.root / 'parity-final').glob('*-headless')):
            message = audit(directory)
            print(message, flush=True)
            report.write(message+'\n')
    with (a.root / a.performance_report).open('w', encoding='utf-8') as report:
        report.write('Microseconds: mean / p95 / max. 10000 samples per file. No import/CSV IO/GPU in timers.\n')
        report.write(f'Measurements: {measurements}\n')
        for file in sorted(measurements.glob('*-performance.csv')):
            with file.open(newline='') as stream:
                rows = list(csv.DictReader(stream))
            report.write(file.stem+'\n')
            for field in ('tick_us', 'control_us', 'telemetry_us', 'publication_us'):
                report.write(f'  {field}: {stats([float(r[field]) for r in rows])}\n')
            report.write(f'  control_plus_telemetry_us: {stats([float(r["control_us"])+float(r["telemetry_us"]) for r in rows])}\n')
        for name in ('arm', 'body'):
            means = {}
            for version in ('baseline', 'current'):
                with (a.root / f'performance-{version}' / f'{name}.csv').open(newline='') as stream:
                    rows = list(csv.DictReader(stream))
                means[version] = statistics.mean(float(r['control_us']) for r in rows)
                report.write(f'{name} {version} tick mean/p95/max: {stats([float(r["tick_us"]) for r in rows])}\n')
            with (measurements / f'{name}-legacy-performance.csv').open(newline='') as stream:
                telemetry = statistics.mean(float(r['telemetry_us']) for r in csv.DictReader(stream))
            report.write(f'{name} new control selection + telemetry mean cost delta (component estimate): '
                         f'{means["current"]-means["baseline"]+telemetry:.6g} us\n')
        report.write('Whole-tick differences contain Windows scheduling/cache/frequency noise; a negative difference is not evidence of acceleration.\n')


if __name__ == '__main__':
    main()
