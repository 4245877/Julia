"""Recompute sequential A/B raw samples, using order statistics (floor(N*p))."""
from pathlib import Path
import argparse,csv,json,statistics

def rows(p):return list(csv.DictReader(p.open(newline='')))
def stats(v):
    v=sorted(v);return dict(count=len(v),mean=statistics.mean(v),p95=v[len(v)*95//100],p99=v[len(v)*99//100],max=max(v),over_8333_333=sum(x>1e6/120 for x in v))
def analyze(path):
    ticks=rows(path/'performance.csv');calls=rows(path/'calls.csv');meta=json.loads((path/'metadata.json').read_text())
    assert len(ticks)>=2400 and len(calls)==4*len(ticks)
    assert all(int(r['evaluations'])==4 and int(r['physical_steps'])==16 for r in ticks)
    groups={}
    for r in calls:groups.setdefault(int(r['tick']),[]).append(r)
    assert len(groups)==len(ticks) and all([int(r['call']) for r in v]==[0,1,2,3] for v in groups.values())
    result=dict(path=str(path),metadata=meta,tick_us=stats([float(r['tick_us']) for r in ticks]),controller_us=stats([float(r['controller_us']) for r in ticks]),per_call={})
    for key in ('assembly_and_solve_us','preparation_us','input_us','linear_us','active_us','verification_us','physical_us'):
        result['per_call'][key]=stats([float(r[key]) for r in calls])
        result['four_'+key]=stats([sum(float(r[key]) for r in v) for v in groups.values()])
    by_tick={int(r['tick']):r for r in ticks}
    result['remaining_tick_us']=stats([float(by_tick[t]['tick_us'])-sum(float(r['assembly_and_solve_us'])+float(r['physical_us']) for r in v) for t,v in groups.items()])
    result['common_controller_us']=stats([float(by_tick[t]['controller_us'])-sum(float(r['assembly_and_solve_us'])+float(r['physical_us']) for r in v) for t,v in groups.items()])
    result['physics_and_observation_us']=stats([float(r['tick_us'])-float(r['controller_us']) for r in ticks])
    result['iteration_max']=max(int(r['iterations']) for r in calls)
    return result
def main():
    p=argparse.ArgumentParser();p.add_argument('--root',required=True,type=Path);p.add_argument('--output',required=True,type=Path);p.add_argument('--require-target',action='store_true');a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    allruns={str(x.parent.relative_to(a.root)):analyze(x.parent) for x in sorted(a.root.rglob('metadata.json')) if (x.parent/'calls.csv').exists()}
    pairs=[];pooled={}
    for scenario in ('hold','motion'):
        pool={}
        for i in range(1,4):
            found={}
            for kind in ('reference','optimized'):
                matches=[r for key,r in allruns.items() if f'{scenario}-pair{i}-{kind}' in key]
                if matches:
                    assert len(matches)==1;found[kind]=matches[0]
                    pool.setdefault(kind,[]).extend(float(r['tick_us']) for r in rows(Path(matches[0]['path'])/'performance.csv'))
            if len(found)==2:
                r,o=found['reference']['tick_us']['p95'],found['optimized']['tick_us']['p95']
                pairs.append(dict(scenario=scenario,pair=i,reference_p95=r,optimized_p95=o,ratio=o/r,pass_target=o/r<=.75 and o<5000))
        if len(pool)==2:
            pooled[scenario]={kind:stats(v) for kind,v in pool.items()};r,o=pooled[scenario]['reference']['p95'],pooled[scenario]['optimized']['p95']
            pooled[scenario].update(ratio=o/r,pass_target=o/r<=.75 and o<5000)
    result=dict(runs=allruns,pairs=pairs,pooled=pooled,target_met=len(pairs)==6 and all(x['pass_target'] for x in pairs) and all(x['pass_target'] for x in pooled.values()))
    (a.output/'summary.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(dict(pairs=pairs,pooled=pooled,target_met=result['target_met']),indent=2))
    if a.require_target:assert result['target_met']
if __name__=='__main__':main()
