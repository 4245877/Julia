"""Run existing frozen desktop/CPU/OFF suites in isolated new output directories."""
from pathlib import Path
import argparse,subprocess,os,sys,re,json,time
p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path);p.add_argument('--parallel',action='store_true');a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
cache=Path('build/vs-release/CMakeCache.txt').read_text();ctest=re.search(r'CMAKE_CTEST_COMMAND:INTERNAL=(.*)',cache).group(1).strip()
env=os.environ.copy();env['PATH']=str(Path(ctest).parent)+os.pathsep+env['PATH']
work=[];results={};start=time.perf_counter()
for variant in ('vs-release','cpu-release','cpu-no-physics'):
    log=(a.output/(variant+'-process.log')).open('w');command=[sys.executable,'tools/stage7d_ctest.py','--build','build/'+variant,'--output',str(a.output/variant)]
    process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,env=env);work.append((variant,process,log))
    if not a.parallel:
        results[variant]=process.wait();log.close()
if a.parallel:
    for variant,process,log in work:results[variant]=process.wait();log.close()
(a.output/'process-results.json').write_text(json.dumps(dict(returncodes=results,wall_seconds=time.perf_counter()-start,parallel=a.parallel),indent=2))
print(results,flush=True);assert not any(results.values())
