"""Independent reader of stage7k negative runs (Disable, support loss, insufficient effort).

For each run: first terminal tick (lost/disabled), no further controller evaluations,
zero applied joint torque on every later completed tick, the same epoch with 16 world
steps per tick and a changing physical pose (no reset, physics continues), raw-contact
alpha validity reconstructed independently (invalid actual exported as NaN), and the
terminal reason. A numerical/physical residual failure is never reported as a proven
infeasibility; only reason 'infeasible' would be.
"""
from pathlib import Path
import argparse
import csv
import json
import math
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))


def table(path):
    with path.open(newline='', encoding='utf-8-sig') as stream:
        return list(csv.DictReader(stream))


def check(label, folder, output):
    from stage7k_audit import audit
    stand = table(folder/'stand-v1.csv')
    terminals = [r for r in stand if int(r['tick']) > 0 and r['phase'] in ('lost', 'disabled')]
    assert terminals, (label, 'no terminal phase')
    terminal = terminals[0]; tick = int(terminal['tick'])
    later = [r for r in stand if int(r['tick']) >= tick]
    assert all(r['evaluations'] == terminal['evaluations'] and r['phase'] == terminal['phase'] for r in later), (label, 'controller evaluated after terminal')
    torques = [float(r['applied_Nm']) for r in table(folder/'joints.csv') if int(r['tick']) > tick]
    assert torques and all(t == 0 for t in torques), (label, 'active torque after terminal')
    states = {int(r['tick']): r for r in table(folder/'state.csv')}
    after = [r for n, r in sorted(states.items()) if n > tick]
    assert len(after) >= 24, (label, 'too few post-terminal ticks')
    assert all(int(r['epoch']) == int(states[tick]['epoch']) and int(r['world_steps'])-int(states[tick]['world_steps']) == (int(r['tick'])-tick)*16
               and float(r['time_s']) > float(states[tick]['time_s']) for r in after), (label, 'reset or missing substeps after terminal')
    fields = ('root_x_m', 'root_y_m', 'root_z_m', 'qw', 'qx', 'qy', 'qz')
    change = max(abs(float(r[f])-float(states[tick][f])) for r in after for f in fields)
    assert change > 1e-8, (label, 'physics stopped after terminal')
    destination = output/f'independent-{label}'; destination.mkdir()
    independent = audit(folder, destination, mode='none', include_dynamics=False)
    for key in ('raw_reconstruction', 'independent_com', 'actual_friction', 'moments', 'reference_limits', 'reference_c2'):
        assert independent['checks'][key], (label, key)
    (destination/'certificate.json').write_text(json.dumps(independent, indent=2), encoding='utf-8')
    coordinated = table(folder/'coordinated-v1.csv')
    invalid = [int(r['tick']) for r in coordinated if int(r['tick']) >= tick and r['actual_valid'] == '0']
    assert all(math.isnan(float(r['alpha_actual'])) for r in coordinated if r['actual_valid'] == '0'), (label, 'invalid actual must be NaN')
    phases = sorted({r['phase'] for r in coordinated if int(r['tick']) > tick})
    if label == 'support-loss':
        assert invalid, 'support loss must make actual alpha invalid'
    return dict(terminal_tick=tick, terminal_time_s=float(states[tick]['time_s']), stand_phase=terminal['phase'], reason=terminal['reason'],
                evaluations_frozen_at=int(terminal['evaluations']), subsequent_qp=0, subsequent_applied_torque_max_abs=max(abs(t) for t in torques),
                completed_post_terminal_ticks=len(after), physical_pose_change=change, epoch_unchanged=True, substeps_per_tick=16, no_reset=True,
                coordinated_phase_after_terminal=phases, invalid_actual_post_terminal_ticks=len(invalid),
                first_invalid_post_terminal_tick=min(invalid) if invalid else None,
                proven_infeasible=terminal['reason'] == 'infeasible', independent_peaks=independent['peaks'], lost=independent['lost'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--disable', type=Path, required=True)
    parser.add_argument('--support-loss', type=Path, required=True)
    parser.add_argument('--weak', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(); args.output.mkdir(parents=True, exist_ok=False)
    result = {label: check(label, folder, args.output) for label, folder in (('disable', args.disable), ('support-loss', args.support_loss), ('weak', args.weak))}
    assert result['disable']['stand_phase'] == 'disabled' and result['support-loss']['stand_phase'] == 'lost' and result['weak']['stand_phase'] == 'lost'
    assert not result['weak']['proven_infeasible']
    result['pass_all'] = True
    (args.output/'certificate.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps({k: (v if not isinstance(v, dict) else {x: v[x] for x in ('terminal_tick', 'stand_phase', 'reason', 'completed_post_terminal_ticks', 'invalid_actual_post_terminal_ticks', 'proven_infeasible')}) for k, v in result.items()}, indent=1))


if __name__ == '__main__':
    main()
