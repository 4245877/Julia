"""Report figures from independent stage7k reconstructions (no controller code).

  --causal <causal audit dir>: joint vs COM-only vs load-only actual COM offsets and alpha
  --pairs <audit dir>: actual COM path in the placement plane coloured by raw alpha, with goals
"""
from pathlib import Path
import argparse
import csv
import numpy as np


def load(path):
    with (path/'reconstructed-v1.csv').open(newline='') as stream:
        data = list(csv.DictReader(stream))
    def series(field, scale=1.):
        values = []
        for r in data:
            try:
                values.append(float(r[field]))
            except ValueError:
                values.append(np.nan)
        return np.array(values)*scale
    return series


def causal(folder, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    variants = [('joint COM+load', load(folder)), ('COM task only', load(folder/'variant-com-only')), ('load task only', load(folder/'variant-load-only'))]
    t = variants[0][1]('time_s')
    fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True, constrained_layout=True)
    for axis, field, scale, label in ((axes[0], 'offset_x', 1000, 'COM offset placement X (mm)'), (axes[1], 'offset_z', 1000, 'COM offset placement Z (mm)'),
                                      (axes[2], 'alpha_actual', 1, 'raw actual alpha')):
        goal = {'offset_x': 'goal_offset_x', 'offset_z': 'goal_offset_z', 'alpha_actual': 'goal_alpha'}[field]
        axis.plot(t, variants[0][1](goal, scale), 'k--', linewidth=1, label='terminal goal')
        for name, series in variants:
            axis.plot(t, series(field, scale), linewidth=1.2, label=name)
        axis.set_ylabel(label); axis.grid(alpha=.25); axis.legend(ncol=4, fontsize=8)
    axes[2].set_xlabel('Physical time (s)')
    axes[0].set_title('Same model, initial state, schedule, commands and references; only the QP tasks differ')
    fig.savefig(output/'causality.png', dpi=150); fig.savefig(output/'causality.svg'); plt.close(fig)


def pairs(folder, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    series = load(folder)
    x, z, a = series('offset_x', 1000), series('offset_z', 1000), series('alpha_actual')
    gx, gz, ga = series('goal_offset_x', 1000), series('goal_offset_z', 1000), series('goal_alpha')
    fig, axis = plt.subplots(figsize=(8, 7), constrained_layout=True)
    points = axis.scatter(x, z, c=a, s=2, cmap='viridis')
    goals = sorted({(round(p, 6), round(q, 6), round(r, 6)) for p, q, r in zip(gx, gz, ga) if np.isfinite(r)})
    for p, q, r in goals:
        axis.plot(p, q, 'r+', markersize=14)
        axis.annotate(f'alpha {r:.4f}', (p, q), textcoords='offset points', xytext=(6, 6), fontsize=8)
    axis.add_patch(plt.Circle((0, 0), 5, fill=False, linestyle=':', color='gray'))
    axis.set_aspect('equal'); axis.set_xlabel('COM offset placement X (mm)'); axis.set_ylabel('COM offset placement Z (mm)')
    axis.set_title('Actual COM path (independent) coloured by raw actual alpha; + terminal goals; dotted 5 mm limit')
    fig.colorbar(points, ax=axis, label='raw actual alpha')
    fig.savefig(output/'com-alpha-plane.png', dpi=150); fig.savefig(output/'com-alpha-plane.svg'); plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--causal', type=Path)
    parser.add_argument('--pairs', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(); args.output.mkdir(parents=True, exist_ok=True)
    if args.causal:
        causal(args.causal, args.output)
    if args.pairs:
        pairs(args.pairs, args.output)


if __name__ == '__main__':
    main()
