from pathlib import Path
import hashlib,json,subprocess
p=Path('build/diagnostics/stage7h');p.mkdir(exist_ok=False)
(p/'initial-status.txt').write_bytes(subprocess.check_output(['git','status','--porcelain=v1']))
(p/'initial-commit.txt').write_bytes(subprocess.check_output(['git','rev-parse','HEAD']))
old=json.loads(Path('build/diagnostics/stage7g/runtime-manifest.json').read_text(encoding='utf-8-sig'))
checks=[dict(path=x['path'],match=Path(x['path']).exists() and hashlib.sha256(Path(x['path']).read_bytes()).hexdigest()==x['sha256']) for x in old]
(p/'baseline-manifest-check.json').write_text(json.dumps(checks,indent=2))
print(sum(x['match'] for x in checks),len(checks))
(p/'criteria.txt').write_text('''Stage7h criteria fixed BEFORE runtime implementation/tuning, 2026-11-10.
Reuse stage7g prepared posture and static certificate. No new whole-body pose search.
Keep masses, 0.4 scale, root freedom, friction .6, feet, caps, damping, legacy control/CSV.
New explicit bounded mode: dynamic root balance, torque caps, contact acceleration equalities,
8 corner forces with inner square friction |fx|+|fz|<=.6 fy, fy>=0, each foot>=30 N.
Solver max128 active-set transitions, fixed memory; residual<=1e-6 physical units;
separate feasible/infeasible/budget/numerical/invalid; time never selects control.
Commands in initial placement horizontal axes, fixed at startStand, max radial5mm.
Reference speed<=1mm/s, acceleration<=2mm/s^2, retarget from current reference (p,v,a).
Goal error<=0.5mm AND horizontal speed<=1mm/s AND two measured loaded active supports
continuously>=2s. At least2mm measured displacement and another horizontal direction.
Primary sequence: t=2s X+3mm, t=10s X-3mm, t=20s Z+3mm, t=28s origin; duration38s.
Retarget/cancel: at2s X+3mm,3s Z+3mm,4s cancel (bounded stop),10s origin; duration18s.
Pulse3N x0.1s at Chest COM: localX at7s, localZ at25s in sequence; recover within3s,
then satisfy precise goal region2s. Also original nominal/pulse16s bounded mode.
Geometry, slips, contacts, penetration, joints, saturation use unchanged7g bounds.
Placement translation(1.2,0,-.7), yaw.73; sensitivity32substeps, contact.008/.012.
Weak caps*.5, support lift450N*.2s, outside force150N*.2s, outside commands rejected atomically.
Independent known active inequality, cap redistribution, infeasibility, iteration exhaustion,
full wrench/corner certificate, dynamics and actual torque checks, invalid/reset/replay.
Legacy/new same initial/physics/gains and same goal reference for causal comparison.
Total tuning budget: <=12 controller parameter candidates AND <=60 native exploratory
BodyRig process runs (all failures and corrected fixtures count). No reset of expense.
Independent analytic fixtures/build fixes not parameter tuning; record those attempts too.
Final validation/parity/regression runs fixed after tuning, recorded separately; no rebuild
during verification. New output paths only. No overwriting published evidence.
Native CLI/desktop30/60/144 exact deterministic fields including new schema; exclude timings.
>=2000 raw performance ticks; mean/p95/max solver/tick, iterations/state bytes;
p95 tick<8333us target; no hard realtime. GPU/callbacks/OS input reported separately.
Full final desktop/CPU/OFF suites; legacy physical CSV compare against accepted manifest.
Exit0/reference completion alone never imply command success.
''',encoding='utf-8')
(p/'experiment-history.md').write_text('# Stage7h experiment history\n\nInitial manifest verified before runtime edits. Criteria fixed in criteria.txt.\nBudget: 0/12 parameter candidates, 0/60 exploratory BodyRig processes.\nNo posture search. Build, analytic fixtures and final verification tracked separately.\n',encoding='utf-8')
