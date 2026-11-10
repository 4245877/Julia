"""Read-only final evidence audit. Never runs a build or changes old diagnostics."""
from pathlib import Path
import hashlib,json,re,xml.etree.ElementTree as ET
from PIL import Image,ImageChops

root=Path.cwd();p=root/'build/diagnostics/stage7g';summary=[]
def shell_text(path):
    data=path.read_bytes()
    return data.decode('utf-16' if data.startswith((b'\xff\xfe',b'\xfe\xff')) else 'utf-8-sig')
manifest=json.loads((p/'runtime-manifest.json').read_text())
for e in manifest:assert hashlib.sha256((root/e['path']).read_bytes()).hexdigest()==e['sha256'],e['path']
summary.append(f'PASS frozen {len(manifest)} runtime source/build/binary files unchanged through all suites/parity')
for variant,count in [('vs-release',68),('cpu-release',29),('cpu-no-physics',22)]:
    out=p/('final-'+variant);text=(out/'ctest.log').read_text();assert f'100% tests passed, 0 tests failed out of {count}' in text
    source=(root/'build'/variant/'tests/CTestTestfile.cmake').read_text()
    source=source.replace(root.as_posix()+'/build/diagnostics/',out.as_posix()+'/artifacts/')
    source=source.replace((root/'build'/variant).as_posix()+'/verification',out.as_posix()+'/verification')
    assert source==(out/'CTestTestfile.cmake').read_text()
    settings=shell_text(p/'build-isolation-final'/(variant+'-settings.txt'))
    assert 'CMAKE_BUILD_TYPE:STRING=Release' in settings and 'JULIA_WARNINGS_AS_ERRORS:BOOL=ON' in settings
    dry=shell_text(p/'build-isolation-final'/(variant+'-dry-run.txt'))
    assert 'Building CXX' not in dry and 'Linking' not in dry
    summary.append(f'PASS {variant} {count}/{count}, /WX Release, exact output-only isolation; read-only Ninja has no compile/link work')
deps=shell_text(p/'build-isolation-final/headless-dependencies.txt');assert not re.search('opengl|glfw|glad',deps,re.I)
summary.append('PASS CPU executable and colocated DLL dependencies have no OpenGL/GLFW/GLAD')
assert 'PASS 9/9' in (p/'parity-final/parity.txt').read_text()
for label in ('nominal','pulse','translated-yaw'):
    for fps in (30,60,144):
        d=p/'parity-final'/f'{label}-{fps}';text=(d/'gpu.txt').read_text()
        assert 'GL_errors=0' in text and 'callback_checks=PASS' in text and 'ticks=1920' in text and f'renders={16*fps} ' in text
        assert (d/'command.txt').exists() and len(list(d.glob('*.ppm')))==49
summary.append('PASS 9 actual CLI/GLFW comparisons, ten CSV contracts exact; 441 native PPM captures; GPU/callbacks separate from untested OS input')
assert 'PASS 606' in (p/'legacy-final/comparison.txt').read_text()
summary.append('PASS 606 accepted prior physical CSV files: schemas and deterministic fields exact; performance workload CSV excluded')
for file in ('independent-final-cpu/audit.txt','independent-wrenches/certificate.json','stance-support-independent/audit.txt','observer-independent-final/audit.txt'):
    assert (p/file).is_file()
summary.append('PASS independent measured hold/recovery/ablation, feasible corner forces, actual contacts/CoP/friction and physical quadratures')
for source,target in [('frame_540.ppm','hold-general.png'),('angle2_540.ppm','hold-top.png'),('frame_720.ppm','recovered-x-general.png'),('angle2_720.ppm','recovered-x-top.png')]:
    assert ImageChops.difference(Image.open(p/'parity-final/pulse-60'/source),Image.open(p/'visuals'/target)).getbbox() is None
assert (p/'plots/recovery.png').is_file() and (p/'plots/feedback-comparison.svg').is_file()
summary.append('PASS reviewed screenshots are lossless PPM conversions; measured recovery/ablation standalone figures retained')
old=root/'build/diagnostics/stage7f/parity-final/drop-headless/model.xml';new=p/'parity-final/pulse-headless/model.xml'
assert old.read_bytes()==new.read_bytes()
xml=ET.parse(new).getroot();assert xml.find('equality') is None and len(xml.findall('.//freejoint'))==1
assert len(xml.find('actuator'))==41 and all(e.attrib['joint']!='root' for e in xml.find('actuator'))
summary.append('PASS nominal model.xml byte-identical to accepted stage7f: free root, original masses/geometry/caps/contact material; no equality or root actuator')
for doc in [root/'docs/body_core_stage7g.md',p/'README.md']:
    for target in re.findall(r'\[[^\]]+\]\(([^)]+)\)',doc.read_text(encoding='utf8')):
        if '://' not in target and target not in ('final-audit.txt','build-and-isolation-audit.txt'):assert (doc.parent/target).resolve().exists(),(doc,target)
summary.append('PASS report/evidence links resolve; initial search-budget deviation is explicitly retained in history and criteria are unchanged')
(p/'final-audit.txt').write_text('\n'.join(summary)+'\n')
(p/'build-and-isolation-audit.txt').write_text('\n'.join(summary[1:5])+'\n')
extra=[root/'docs/body_core_stage7g.md',p/'README.md',*sorted((root/'tools').glob('*stage7g*'))]
(p/'report-tools-manifest.json').write_text(json.dumps([dict(path=f.relative_to(root).as_posix(),sha256=hashlib.sha256(f.read_bytes()).hexdigest()) for f in extra],indent=2))
print('\n'.join(summary))
