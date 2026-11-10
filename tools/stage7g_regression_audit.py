"""Read-only exact old physics CSV comparison with accepted stage7f full suites."""
from pathlib import Path
import argparse,csv,json,hashlib
from verify_stage7c import compare

p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--desktop',action='store_true');a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
variants=[('final-cpu','final-cpu-release')]
if a.desktop:variants.append(('final-desktop','final-vs-release'))
result=[];skipped=[]
for old,new in variants:
    reference=Path('build/diagnostics/stage7f')/old/'artifacts'
    actual=Path('build/diagnostics/stage7g')/new/'artifacts'
    for f in sorted(reference.rglob('*.csv')):
        rel=f.relative_to(reference)
        if not rel.parts[0].startswith('stage7'):continue
        if 'performance' in f.name:
            skipped.append(str(rel));continue
        count,fields=compare(f,actual/rel)
        result.append(dict(variant=new,path=rel.as_posix(),rows=count,deterministic_fields=fields))
(a.output/'comparison.json').write_text(json.dumps(result,indent=2))
(a.output/'comparison.txt').write_text(f'PASS {len(result)} accepted old physics CSV files: exact deterministic fields and unchanged schemas; timings excluded.\nSkipped performance workload CSV: {len(skipped)} (runtime/memory values are not deterministic physical contracts).\n')
print((a.output/'comparison.txt').read_text())
