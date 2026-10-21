"""Independent checks of the earlier repair claims against actual Blender data."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import bpy
import numpy as np
from mathutils import Vector
from bpy_extras.object_utils import world_to_camera_view
from PIL import Image,ImageDraw
from reaudit import ROOT,reset,render


def main():
    p=argparse.ArgumentParser();p.add_argument('--blend',default='assets/models/Raphtalia_prepared.blend')
    p.add_argument('--output',default='docs/raphtalia/reaudit/structure.json');p.add_argument('--render',action='store_true');args=p.parse_args()
    bpy.ops.wm.open_mainfile(filepath=str(ROOT/args.blend));arm=next(o for o in bpy.data.objects if o.type=='ARMATURE');reset(arm)
    prep=json.loads((ROOT/'docs/raphtalia/preparation.json').read_text(encoding='utf-8'))
    checks=[]
    for change in prep['changes']:
        b=arm.data.bones[change['bone']];delta=(b.head_local-Vector(change['new_head'])).length
        checks.append({'bone':b.name,'head_delta_source_units':delta,'matches':delta<.001})
    cloth=[{'bone':x['bone'],'actual_parent':arm.data.bones[x['bone']].parent.name,
            'matches':arm.data.bones[x['bone']].parent.name==x['parent']} for x in prep['reparent']]
    tails=[{'bone':x['bone'],'matches':(arm.data.bones[x['bone']].tail_local-Vector(x['new_tail'])).length<.001} for x in prep['tail_repairs']]
    meshes=[o for o in bpy.data.objects if o.type=='MESH'];report={'bones':len(arm.data.bones),'original_refit_heads':checks,'cloth_links':cloth,
        'tail_repairs':tails,'roots':[b.name for b in arm.data.bones if not b.parent],
        'degenerate_bones':[b.name for b in arm.data.bones if b.length<1e-4],
        'invalid_rest_matrices':[b.name for b in arm.data.bones if not np.isfinite(np.asarray(b.matrix_local)).all() or abs(b.matrix_local.determinant()-1)>1e-4],
        'lbs_modifiers':{o.name:[{'preserve_volume':m.use_deform_preserve_volume,'armature':m.object.name} for m in o.modifiers if m.type=='ARMATURE'] for o in meshes},
        'bones_with_constraints':{p.name:[c.type for c in p.constraints] for p in arm.pose.bones if p.constraints},
        'new_brows':{s:sum(any(o.vertex_groups[g.group].name=='brow.'+s and g.weight>.99 for g in v.groups) for v in o.data.vertices)
                     for s in ('L','R') for o in meshes if o.name=='eyelashes_tex_eyelashes_0'},
        'cuffs':{},'protected_files':[]}
    cuff=bpy.data.objects['cuff.001_tex_cuff_0']
    report['cuffs']={'vertices':len(cuff.data.vertices),'nonrigid':sum(sum(g.weight>1e-8 for g in v.groups)!=1 for v in cuff.data.vertices)}
    for path in ('assets/models/Raphtalia.fbx','assets/models/backups/Raphtalia.original.fbx','assets/models/Raphtalia1.fbx'):
        data=(ROOT/path).read_bytes();report['protected_files'].append({'path':path,'sha256':hashlib.sha256(data).hexdigest()})
    assert all(x['matches'] for x in checks+cloth+tails)
    assert not report['degenerate_bones'] and not report['invalid_rest_matrices']
    assert len(report['roots'])==1 and report['cuffs']['nonrigid']==0
    out=ROOT/args.output;out.parent.mkdir(parents=True,exist_ok=True)
    if args.render:
        major=[b for b in arm.data.bones if b.name.startswith(('arm_stretch','forearm_stretch','hand.','shoulder.','thigh_stretch','leg_stretch','foot.','toes_','spine_','neck.','head.'))]
        for view,direction in [('front',(0,-1,0)),('side',(1,0,0))]:
            file=out.parent/('joints_'+view+'.png');render(file,(0,0,2.1),4.6,direction)
            image=Image.open(file).convert('RGB');draw=ImageDraw.Draw(image);scene=bpy.context.scene
            for b in major:
                points=[world_to_camera_view(scene,scene.camera,arm.matrix_world@pos) for pos in (b.head_local,b.tail_local)]
                xy=[(int(v.x*image.width),int((1-v.y)*image.height)) for v in points]
                color=(80,240,110) if '.l_' in b.name else (255,140,80) if '.r_' in b.name else (80,200,255)
                draw.line(xy,fill=color,width=2);x,y=xy[0];draw.ellipse((x-3,y-3,x+3,y+3),fill=color)
            image.save(file)
    with (ROOT/'docs/raphtalia/skeleton_prepared.csv').open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.writer(f);w.writerow(['bone','parent','head_x','head_y','head_z','tail_x','tail_y','tail_z'])
        for b in arm.data.bones:w.writerow([b.name,b.parent.name if b.parent else '',*b.head_local,*b.tail_local])
    report['result']='PASS';out.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print('STRUCTURE PASS',len(checks),'refit heads,',len(cloth),'cloth links,',len(tails),'tail repairs;',len(arm.data.bones),'bones')


if __name__=='__main__':main()
