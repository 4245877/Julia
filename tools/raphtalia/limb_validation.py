"""Read-only replay of the static knee/boot authoring patch and held-out poses.

Input must be the retained pre-deep-pass model. No asset is saved or exported.
"""
import argparse
import hashlib
import json
from pathlib import Path

import bpy
import numpy as np

from reaudit import ROOT, reset, find, coordinates
from deep_pose_audit import definitions, apply_pose
from deep_body import apply_limb_clothes


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', default='build/raphtalia_tools/deep/base/Raphtalia_prepared.blend')
    parser.add_argument('--output', default='docs/raphtalia/deep_deformation/limb_validation.json')
    args = parser.parse_args()
    source, output = (ROOT/args.input).resolve(), (ROOT/args.output).resolve()
    source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
    bpy.ops.wm.open_mainfile(filepath=str(source))
    arm = next(o for o in bpy.data.objects if o.type == 'ARMATURE')
    meshes = [bpy.data.objects[n] for n in ('stocking_tex_sock_0','shoes_tex_shoes_0')]
    reset(arm)
    cache = {}
    for obj in meshes:
        rest = coordinates(obj)
        edges = np.array([edge.vertices[:] for edge in obj.data.edges])
        length = np.linalg.norm(rest[edges[:,0]]-rest[edges[:,1]],axis=1)
        cache[obj.name] = (rest,edges,length)

    def metrics():
        result = {}
        for obj in meshes:
            rest, edges, length = cache[obj.name]
            posed = coordinates(obj)
            current = np.linalg.norm(posed[edges[:,0]]-posed[edges[:,1]],axis=1)
            valid = length>1e-6
            ratio = current[valid]/length[valid]
            result[obj.name] = {'max_ratio':float(ratio.max()), 'p99':float(np.percentile(ratio,99)),
                'edges_over_2':int((ratio>2).sum()), 'edges_under_half':int((ratio<.5).sum()),
                'nonfinite':int((~np.isfinite(posed).all(1)).sum())}
        return result

    cases = definitions(arm)
    standard_names = list(cases)
    # These angles and coupled foot motions were not used by the fitter.
    for angle in range(5,136,5):
        if angle in (60,90,120,135):
            continue
        cases['heldout_knee_'+str(angle)] = [(find(arm,'leg_stretch',s),angle,(1,0,0)) for s in ('l','r')]
    for angle in (-35,-25,-15,-5,5,15,25,35):
        cases['heldout_ankle_'+str(angle)] = [(find(arm,'foot',s),angle,(1,0,0)) for s in ('l','r')]
    for angle in (-30,-15,15,30):
        cases['heldout_foot_roll_'+str(angle)] = [(find(arm,'foot',s),angle,tuple(arm.data.bones[find(arm,'foot',s)].y_axis)) for s in ('l','r')]
    for side in ('l','r'):
        for angle in (35,75,105,130):
            cases['heldout_unilateral_'+side+'_'+str(angle)] = [(find(arm,'leg_stretch',side),angle,(1,0,0)),(find(arm,'foot',side),20,(1,0,0))]
        for prefix in ('leg_stretch','leg_twist','thigh_twist'):
            bone = find(arm,prefix,side)
            for angle in (-60,-30,30,60):
                cases['heldout_axial_'+prefix+'_'+side+'_'+str(angle)] = [(bone,angle,tuple(arm.data.bones[bone].y_axis))]
    report = {'source':str(source.relative_to(ROOT)), 'source_sha256':source_hash,
        'tool_sha256':hashlib.sha256((ROOT/'tools/raphtalia/deep_body.py').read_bytes()).hexdigest(),
        'blender':bpy.app.version_string,'standard_poses':standard_names,
        'heldout_poses':[n for n in cases if n not in standard_names],
        'limits':'Edge and collapse diagnostics plus matched visual review; no proof of collision-free cloth. No runtime solver.',
        'rejected_candidates':[
            {'method':'full-radius stocking scalar diffusion','reason':'ordinary knee motion worsened',
             'knee_60_max_before_after':[1.749,2.122],'knee_90_max_before_after':[2.077,2.521],
             'disposition':'replaced by bounded motion-constrained fit'},
            {'method':'233-vertex boot scalar diffusion','reason':'unnecessary spread and mild ordinary positive-ankle regressions',
             'ankle_15_max_before_after':[1.135,1.213],
             'disposition':'replaced by 144-vertex connected-fold coherence'}],
        'before':{},'after':{}}
    for stage in ('before','after'):
        if stage == 'after':
            reset(arm)
            patches = apply_limb_clothes(arm,meshes)
            report['patches'] = [{k:v for k,v in patch.items() if k!='changes'} | {'changed_vertices':len(patch['changes'])} for patch in patches]
            report['rest_max_delta_mm'] = {obj.name:float(np.linalg.norm(coordinates(obj)-cache[obj.name][0],axis=1).max()*1000) for obj in meshes}
        for name, changes in cases.items():
            reset(arm)
            apply_pose(arm,changes)
            report[stage][name] = metrics()
        for action in bpy.data.actions:
            frames = np.linspace(action.frame_range[0],action.frame_range[1],17 if action.name=='TEST_practical_ranges' else 5)
            for frame in frames:
                reset(arm)
                arm.animation_data.action = action
                bpy.context.scene.frame_set(int(frame),subframe=float(frame%1))
                bpy.context.view_layer.update()
                report[stage]['legacy_'+action.name+'_'+str(round(float(frame),3))] = metrics()
    report['source_unchanged'] = source_hash == hashlib.sha256(source.read_bytes()).hexdigest()
    report['samples_per_stage'] = len(report['before'])
    report['standard_over2_regressions'] = []
    report['standard_max_increases_over_0_05'] = []
    for name in standard_names:
        for mesh in report['before'][name]:
            before, after = report['before'][name][mesh], report['after'][name][mesh]
            if after['edges_over_2']>before['edges_over_2']:
                report['standard_over2_regressions'].append([name,mesh,before,after])
            if after['max_ratio']>before['max_ratio']+.05:
                report['standard_max_increases_over_0_05'].append([name,mesh,before,after])
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({'samples':report['samples_per_stage'],'patches':[(p['mesh'],p['changed_vertices']) for p in report['patches']],
        'rest_delta_mm':report['rest_max_delta_mm'], 'standard_over2_regressions':len(report['standard_over2_regressions'])}))


if __name__ == '__main__':
    main()
