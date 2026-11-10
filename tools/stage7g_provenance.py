"""Check accepted stage7f evidence before any stage7g physics edits."""
from pathlib import Path
import hashlib, json, subprocess
p=Path('build/diagnostics/stage7g')
entries=json.loads(Path('build/diagnostics/stage7f/final-source-binary-manifest.json').read_text())
results=[]
for e in entries:
    f=Path(e['path'])
    actual=hashlib.sha256(f.read_bytes()).hexdigest() if f.is_file() else 'MISSING'
    results.append(dict(path=e['path'],expected=e['sha256'],actual=actual,match=actual==e['sha256']))
(p/'baseline-manifest-check.json').write_text(json.dumps(results,indent=2))
(p/'initial-status.txt').write_bytes(subprocess.check_output(['git','status','--short']))
(p/'initial-commit.txt').write_bytes(subprocess.check_output(['git','rev-parse','HEAD']))
print(f"accepted stage7f manifest: {sum(e['match'] for e in results)}/{len(results)}")
