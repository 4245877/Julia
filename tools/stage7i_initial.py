"""Immutable bootstrap; run once before implementation changes."""
from pathlib import Path
import csv, hashlib, json, math, platform, shutil, statistics, subprocess

root = Path('build/diagnostics/stage7i')
root.mkdir(exist_ok=False)
def save(name, value):
    (root/name).write_text(json.dumps(value, indent=2), encoding='utf-8')
save('initial-state.json', dict(commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
    status=subprocess.check_output(['git','status','--porcelain=v1'],text=True),
    machine=platform.uname()._asdict(), instructions_found=[str(p) for p in Path('.').rglob('AGENTS.md')]))
old=json.loads(Path('build/diagnostics/stage7h/runtime-manifest-v3.json').read_text(encoding='utf-8-sig'))
checks=[dict(path=x['path'],match=Path(x['path']).is_file() and hashlib.sha256(Path(x['path']).read_bytes()).hexdigest()==x['sha256']) for x in old]
save('baseline-manifest-check.json',checks)
assert all(x['match'] for x in checks)
for item in old:
    p=Path(item['path'])
    if p.suffix in ('.cpp','.hpp','.ps1','.py','.txt') and 'build' not in p.parts:
        dest=root/'accepted-source'/p;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest)
for variant in ('cpu-release','vs-release','cpu-no-physics'):
    for name in ('CMakeCache.txt','build.ninja'):
        p=Path('build')/variant/name;dest=root/'accepted-build'/variant/name;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest)
for folder in ('build/cpu-release/tests','build/cpu-release/apps/julia_body_physics'):
    for p in Path(folder).glob('*'):
        if p.suffix in ('.exe','.dll') and (p.suffix=='.dll' or p.stem in ('julia_test_com_control','julia_test_bounded_qp','julia_body_physics')):
            dest=root/'accepted-binaries'/Path(folder).name/p.name;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest)
def stats(v):
    v=sorted(v);return dict(count=len(v),mean=statistics.mean(v),p95=v[len(v)*95//100],p99=v[len(v)*99//100],max=max(v),over_8333=sum(x>1e6/120 for x in v))
report={}
for variant in ('cpu-release','vs-release'):
    folder=Path('build/diagnostics/stage7h/final-tests-03')/variant/'artifacts/stage7h/cpu'
    ticks=list(csv.DictReader((folder/'performance.csv').open()))
    samples=list(csv.DictReader((folder/'solver-performance.csv').open()))
    groups={}
    for r in samples:groups.setdefault(int(r['tick']),[]).append(float(r['solver_us']))
    assert len(groups)==2400 and all(len(v)==4 for v in groups.values())
    report[variant]=dict(tick_us=stats([float(r['tick_us']) for r in ticks]),
        bounded_assembly_and_solve_one_call_us=stats([float(r['solver_us']) for r in samples]),
        bounded_assembly_and_solve_four_calls_us=stats([sum(v) for v in groups.values()]),
        evaluate_all_substeps_us=stats([float(r['controller_us']) for r in ticks]),
        source_files=[str(folder/x) for x in ('performance.csv','solver-performance.csv')])
save('stage7h-timing-recalculated.json',report)
(root/'criteria.txt').write_text('''Stage7i criteria fixed before optimization, 2026-11-11.
Baseline HEAD and complete accepted stage7h manifest checked in initial-state.json.
Preserve shared QP construction, all physics/model/settings and default Equality7g;
Bounded7h default solver remains reference; optimized explicitly selectable.
Cold start; max128 deterministic transitions, fixed arrays; no timing decisions.
Physical root/contact residual <1e-6, normalized inequality <=1e-7,
projected stationarity <=1e-5. Failure => existing Lost/zero active torque semantics.
Identical input QP comparisons: feasible code, relative objective difference <=1e-8
(abs difference / max(1,abs(reference objective))), max primal difference<=1e-5,
equality<=1e-6, inequality<=1e-7, projected stationarity<=1e-5,
dual nonnegativity>=-1e-8, absolute complementarity<=1e-5.
Independent equality multiplier reconstruction; do not compare nonunique dual values.
Same-input corpus selected BEFORE solving: hold every120th controller update;
full54s motion+pulses every120th plus all active-set transitions and pulse windows.
Across implementations physical histories: position/COM<=1e-5m, q<=1e-4rad,
velocity<=1e-4m/s or rad/s, torque<=1e-3Nm; phases, goals, support and terminal
semantics unchanged (goal timestamps allowance one120Hz tick). This is much smaller
than 0.5mm goal tolerance and 4mm penetration bound. Within one implementation:
CLI/real GLFW exact deterministic CSV30/60/144Hz (timing excluded only).
All stage7h actual goal/hold/speed/support/torque/slip/penetration/joint bounds unchanged.
Final: full desktop/CPU/OFF suites, legacy histories, analytic and independent dynamics,
retarget/cancel/return, yaw+translation, reset/replay, negatives, actual contacts.
Performance runs sequential, no build/CTest/parity concurrent, warmup>=240 ticks,
hold>=2400 measured ticks and full54s motion+pulses after warmup (>=2400 ticks).
CPU same Release /WX machine; >=3 pairs/scenario, order R/O,O/R,R/O.
For BOTH scenarios every pair and pooled p95 optimized<=.75*reference and <5000us.
Report mean/p95/p99/max, >8333.333us count, four480Hz updates and16 mj_step/tick,
iterations, model/data/controller/snapshot/process memory, per-run dispersion.
No omission of observations/checks, no precision/frequency/model/parameter changes.
Max6 implementation variants; max24 exploratory native process launches including
failed/corrected retries and analytic launches; fixed final validation separate after
freeze. Record each launch before start. No posture or physics tuning. Old outputs
immutable. No hard realtime; GPU/callback/actual OS input distinguished.
''',encoding='utf-8')
save('budget.json',dict(variant_limit=6,exploratory_native_process_limit=24,variants=[],runs=[]))
print('baseline verified',len(checks),'files; criteria fixed; raw timing recalculated')
