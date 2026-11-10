"""Run an existing CTest suite with diagnostic destinations isolated from history."""
import argparse
from pathlib import Path
import subprocess

p = argparse.ArgumentParser()
p.add_argument('--build', required=True, type=Path)
p.add_argument('--output', required=True, type=Path)
p.add_argument('--regex')
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=True)
source = (a.build / 'tests/CTestTestfile.cmake').read_text(encoding='utf-8')
root = Path.cwd().as_posix()
destination = a.output.resolve().as_posix()
source = source.replace(root + '/build/diagnostics/', destination + '/artifacts/')
source = source.replace(a.build.resolve().as_posix() + '/verification', destination + '/verification')
(a.output / 'CTestTestfile.cmake').write_text(source, encoding='utf-8')
command = ['ctest', '--test-dir', str(a.output), '--output-on-failure']
if a.regex:
    command += ['-R', a.regex]
with (a.output / 'ctest.log').open('w', encoding='utf-8') as log:
    result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
print((a.output / 'ctest.log').read_text(encoding='utf-8'), flush=True)
raise SystemExit(result.returncode)
