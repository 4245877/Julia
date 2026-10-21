"""Render consistent diagnostic states; this never saves the opened scene."""
from pathlib import Path
import argparse
import hashlib
import json
import bpy
from mathutils import Quaternion,Vector
from PIL import Image,ImageDraw,ImageFont
from reaudit import reset,turn
from blink_render import render

ROOT=Path(__file__).resolve().parents[2]

def sheet(rows,path,tile=(600,360)):
    w,h=tile;canvas=Image.new('RGB',(max(map(len,rows))*w,len(rows)*(h+30)),(238,240,243));d=ImageDraw.Draw(canvas)
    try:font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',18)
    except OSError:font=ImageFont.load_default()
    for y,row in enumerate(rows):
        for x,(image,label) in enumerate(row):
            im=Image.open(image).convert('RGB');im=im.resize(tile,Image.Resampling.LANCZOS)
            canvas.paste(im,(x*w,y*(h+30)+30));d.text((x*w+10,y*(h+30)+5),label,fill=(20,25,35),font=font)
    canvas.save(path,optimize=True)

def main():
    p=argparse.ArgumentParser();p.add_argument('--blend',default='build/raphtalia_tools/blink/candidate.blend')
    p.add_argument('--output',default='docs/raphtalia/blink');args=p.parse_args()
    source=ROOT/args.blend;out=ROOT/args.output;out.mkdir(parents=True,exist_ok=True)
    bpy.ops.wm.open_mainfile(filepath=str(source));arm=next(o for o in bpy.data.objects if o.type=='ARMATURE')
    rows=[];files=[]
    def state(name,left,right,yaw=0,pitch=0,side=False):
        reset(arm)
        for s in ('L','R'):
            p=arm.pose.bones['eye.'+s];q=p.bone.matrix_local.to_quaternion()
            p.rotation_quaternion=q.inverted()@Quaternion(Vector((0,0,1)),__import__('math').radians(yaw))@Quaternion(Vector((1,0,0)),__import__('math').radians(pitch))@q
        arm['Blink_L']=float(left);arm['Blink_R']=float(right);arm.update_tag();bpy.context.view_layer.update()
        path=out/(name+'.png');render(path,side=side);files.append({'file':path.name,'L':left,'R':right,'yaw':yaw,'pitch':pitch,'oblique':side})
        return path
    for group in ('both','left','right'):
        row=[]
        for level in (0,.25,.5,.75,1.):
            l=level if group!='right' else 0;r=level if group!='left' else 0
            path=state(f'{group}_{int(level*100):03}',l,r)
            row.append((path,f'{group}: Blink_L={l:g}  Blink_R={r:g}'))
        rows.append(row)
    sheet(rows,out/'states.png',tile=(480,288))
    gaze_rows=[]
    for name,yaw,pitch in [('straight',0,0),('up',0,8),('down',0,-8),('left',12,0),('right',-12,0),('diagonal',10,6)]:
        gaze_rows.append([(state(f'gaze_{name}_{int(b*100):03}',b,b,yaw,pitch),f'{name} ({yaw:g}, {pitch:g} deg) | Blink={b:g}') for b in (0,.5,1.)])
    sheet(gaze_rows,out/'gaze.png')
    for b in (.5,1.):state(f'oblique_{int(b*100):03}',b,b,side=True)
    head=[]
    action=bpy.data.actions['TEST_blink_head']
    for frame in (1,19,20,50,80,121):
        reset(arm);arm['Blink_L']=arm['Blink_R']=0.;arm.animation_data.action=action;bpy.context.scene.frame_set(frame);arm.update_tag();bpy.context.view_layer.update()
        path=out/f'head_{frame:03}.png';render(path);head.append((path,f'Head + gaze + existing brows: frame {frame}'));files.append({'file':path.name,'action':action.name,'frame':frame})
    sheet([head[:3],head[3:]],out/'head.png')
    # A compact playback preview uses the same diagnostic clay shader.
    frames=[];action=bpy.data.actions['TEST_blink_natural']
    for frame in range(1,32):
        reset(arm);arm['Blink_L']=arm['Blink_R']=0.;arm.animation_data.action=action;bpy.context.scene.frame_set(frame);arm.update_tag();bpy.context.view_layer.update()
        path=out/f'_natural_{frame:03}.png';render(path)
        frames.append(Image.open(path).convert('RGB').resize((720,432),Image.Resampling.LANCZOS))
    frames[0].save(out/'natural.gif',save_all=True,append_images=frames[1:],duration=[33]*31,loop=0)
    # Temporary per-frame files are known outputs in this directory only.
    for frame in range(1,32):(out/f'_natural_{frame:03}.png').unlink()
    result={'blend_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
        'render':'Blender Workbench studio, clay, fixed camera/lighting; no material or texture changes are saved',
        'states':files,'sha256':{name:hashlib.sha256((out/name).read_bytes()).hexdigest()
                               for name in sorted({r['file'] for r in files}|{'states.png','gaze.png','head.png'})},
        'limits':'Clay images assess geometry. Actual material rendering is checked separately by the OpenGL auditor.'}
    (out/'visual.json').write_text(json.dumps(result,indent=2),encoding='utf-8')

if __name__=='__main__':main()
