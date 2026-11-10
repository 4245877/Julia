"""Read-only old physical CSV comparison against accepted7g; no archive reruns."""
from pathlib import Path
import argparse,json
from verify_stage7c import compare
p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path);p.add_argument('--batch',default='final-tests-02');a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
result=[];skipped=[]
for variant in ('cpu-release','vs-release'):
    old=Path('build/diagnostics/stage7g')/('final-'+variant)/'artifacts'
    new=Path('build/diagnostics/stage7h')/a.batch/variant/'artifacts'
    for f in sorted(old.rglob('*.csv')):
        rel=f.relative_to(old)
        if not rel.parts[0].startswith('stage7'):continue
        if 'performance' in f.name:skipped.append(str(rel));continue
        count,fields=compare(f,new/rel);result.append(dict(variant=variant,path=rel.as_posix(),rows=count,fields=fields))
(a.output/'comparison.json').write_text(json.dumps(result,indent=2))
(a.output/'comparison.txt').write_text(f'PASS {len(result)} accepted7g physical CSV files: unchanged schemas and exact deterministic fields. Timings excluded. Skipped{len(skipped)} performance workload CSV.\n')
print((a.output/'comparison.txt').read_text())
