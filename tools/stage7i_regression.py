"""Compare accepted stage7h histories and frozen reference/optimized suites."""
from pathlib import Path
import argparse,json
from verify_stage7c import compare
p=argparse.ArgumentParser();p.add_argument('--tests',required=True,type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
records=[];excluded=[]
for variant in ('cpu-release','vs-release'):
    old=Path('build/diagnostics/stage7h/final-tests-03')/variant/'artifacts'
    new=a.tests/variant/'artifacts'
    for f in sorted(old.rglob('*.csv')):
        rel=f.relative_to(old)
        if not rel.parts[0].startswith('stage7'):continue
        if 'performance' in f.name:excluded.append(str(rel));continue
        count,fields=compare(f,new/rel);records.append(dict(variant=variant,path=str(rel),rows=count,fields=fields))
    ref=new/'stage7h/cpu';opt=new/'stage7i/optimized-suite'
    for f in sorted(ref.rglob('*.csv')):
        if 'performance' in f.name:continue
        rel=f.relative_to(ref);count,fields=compare(f,opt/rel);records.append(dict(variant=variant,path='optimized/'+str(rel),rows=count,fields=fields))
(a.output/'comparison.json').write_text(json.dumps(records,indent=2));(a.output/'excluded-performance.json').write_text(json.dumps(excluded,indent=2))
print('PASS',len(records),'exact deterministic CSV histories vs stage7h and reference; timing fields only excluded')
