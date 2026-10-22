"""Read-only audit of runtime Raphtalia eye axes and baked blink parameterization.

Run from the workspace with the installed MSVC/Assimp and local bpy 4.2:
    python tools/raphtalia/runtime_eye_audit.py

The small C++ probe reuses the existing standalone Assimp auditor's JSON and
hierarchy utilities; it does not load OpenGL or write FBX/BLEND. Generated probe
sources/binaries stay under build/eye_audit. The script independently compares
all exported closing keys against the saved Blender Blink_L curve and verifies
all seven deliverable-manifest hashes before and after inspection.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]

# Compile-time reuse is limited to the existing read-only auditor. Its original
# main is renamed so the generated executable runs only this focused probe.
PROBE = r'''
#define main original_assimp_audit_main
#include "@AUDIT_SOURCE@"
#undef main
Json quat(const aiQuaternion& q){Json j=Json::array();j.push(q.w);j.push(q.x);j.push(q.y);j.push(q.z);return j;}
const aiNodeAnim* channel(const aiAnimation* a,const std::string& name){for(unsigned i=0;i<a->mNumChannels;++i)if(a->mChannels[i]->mNodeName.C_Str()==name)return a->mChannels[i];return nullptr;}
int main(int argc,char** argv){if(argc!=3)return 2;Assimp::Importer importer;auto s=importer.ReadFile(argv[1],aiProcess_ValidateDataStructure|aiProcess_PopulateArmatureData);if(!s){std::cerr<<importer.GetErrorString();return 1;}SceneData d(s);Json out;out["nodes"]=Json::array();out["clips"]=Json::array();unsigned lids=0;std::set<std::string> parents;
for(unsigned n=0;n<d.nodes.size();++n){const auto& nd=d.nodes[n];std::string name=nd.ptr->mName.C_Str();if(name.starts_with("lid.")){++lids;parents.insert(d.nodes[nd.parent].ptr->mName.C_Str());}if(name=="eye.L"||name=="eye.R"||name=="head.x_018"||name=="Raphtalia_Rig"||name=="RootNode"||name=="lid.upper.L.16"){Json j;aiVector3D sc,p;aiQuaternion q;nd.ptr->mTransformation.Decompose(sc,q,p);j["name"]=name;j["index"]=n;j["parent"]=nd.parent<0?"none":d.nodes[nd.parent].ptr->mName.C_Str();j["local"]=matrix(nd.ptr->mTransformation);j["global"]=matrix(nd.global);j["local_q"]=quat(q);j["translation"]=vec(p);j["scale"]=vec(sc);out["nodes"].push(j);}}
out["lid_count"]=lids;out["lid_parents"]=Json::array();for(auto p:parents)out["lid_parents"].push(p);
for(unsigned a=0;a<s->mNumAnimations;++a){auto clip=s->mAnimations[a];std::string name=clip->mName.C_Str();Json j;j["name"]=name;j["duration_ticks"]=clip->mDuration;j["tps"]=clip->mTicksPerSecond;j["channels"]=clip->mNumChannels;float lidMax=0,eyeMax=0;for(unsigned c=0;c<clip->mNumChannels;++c){auto ch=clip->mChannels[c];std::string cn=ch->mNodeName.C_Str();if(cn.starts_with("lid.")){for(unsigned k=1;k<ch->mNumPositionKeys;++k)lidMax=std::max(lidMax,(ch->mPositionKeys[k].mValue-ch->mPositionKeys[0].mValue).Length());}if(cn=="eye.L"||cn=="eye.R"){for(unsigned k=1;k<ch->mNumRotationKeys;++k){auto p=ch->mRotationKeys[0].mValue;auto q=ch->mRotationKeys[k].mValue;float dot=std::abs(p.w*q.w+p.x*q.x+p.y*q.y+p.z*q.z);eyeMax=std::max(eyeMax,2*std::acos(std::min(1.f,dot))*180.f/3.14159265f);}}}j["max_lid_translation_change"]=lidMax;j["max_eye_rotation_deg"]=eyeMax;
if(name.ends_with("TEST_blink_natural")){auto c=channel(clip,"lid.upper.L.16");j["upper_center_keys"]=Json::array();for(unsigned k=0;k<c->mNumPositionKeys;++k){const auto& key=c->mPositionKeys[k];if(key.mTime>=7.999&&key.mTime<=11.001){Json z;z["ticks"]=key.mTime;z["position"]=vec(key.mValue);j["upper_center_keys"].push(z);}}}
if(name.ends_with("TEST_blink_gaze")){j["gaze_keys"]=Json::array();for(double t:{0.,23.,41.,59.,77.,95.,114.}){Json z;z["ticks"]=t;for(auto side:{"eye.L","eye.R"}){auto c=channel(clip,side);z[side]=quat(sampleQuaternion(c->mRotationKeys,c->mNumRotationKeys,t,aiQuaternion{}));}j["gaze_keys"].push(z);}}
out["clips"].push(j);}
out["eye_weights"]=Json::array();for(unsigned m=0;m<s->mNumMeshes;++m){auto mesh=s->mMeshes[m];Json j;j["mesh"]=mesh->mName.C_Str();unsigned count=0;double sum=0;for(unsigned b=0;b<mesh->mNumBones;++b){std::string n=mesh->mBones[b]->mName.C_Str();if(n=="eye.L"||n=="eye.R"){for(unsigned w=0;w<mesh->mBones[b]->mNumWeights;++w){auto x=mesh->mBones[b]->mWeights[w];if(x.mWeight>0){++count;sum+=x.mWeight;}}}}if(count){j["positive_assignments"]=count;j["weight_sum"]=sum;out["eye_weights"].push(j);}}
std::ofstream f(argv[2]);out.write(f);std::cout<<"Written FBX rig evidence: "<<lids<<" lids, "<<s->mNumAnimations<<" clips\n";}
'''


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def check_manifest():
    records = json.loads((ROOT / "docs/raphtalia/deliverable_manifest.json").read_text(encoding="utf-8-sig"))
    result = []
    for expected in records:
        path = ROOT / expected["path"]
        sha = digest(path)
        record = {"path": expected["path"], "bytes": path.stat().st_size, "sha256": sha,
                  "matches_manifest": sha.upper() == expected["sha256"].upper()
                      and path.stat().st_size == expected["bytes"]}
        if not record["matches_manifest"]:
            raise ValueError(f"Deliverable differs from retained manifest: {path}")
        result.append(record)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="docs/raphtalia/eyes/rig.json")
    parser.add_argument("--build", default="build/eye_audit")
    parser.add_argument("--skip-build", action="store_true")
    args = parser.parse_args()
    before = check_manifest()
    build = (ROOT / args.build).resolve()
    build.mkdir(parents=True, exist_ok=True)
    installed = ROOT / "build/vs-release/vcpkg_installed/x64-windows"
    compiler_setup = Path("C:/Program Files/Microsoft Visual Studio/18/Professional/VC/Auxiliary/Build/vcvars64.bat")
    executable = build / "runtime_eye_audit.exe"
    if not args.skip_build:
        libraries = list((installed / "lib").glob("assimp-*-mt.lib"))
        if len(libraries) != 1:
            raise ValueError("Expected one installed Release Assimp library")
        if not compiler_setup.is_file():
            raise ValueError(f"MSVC environment missing: {compiler_setup}")
        source = build / "runtime_eye_audit.cpp"
        source.write_text(PROBE.replace("@AUDIT_SOURCE@", (ROOT / "tools/raphtalia/assimp_audit.cpp").as_posix()), encoding="utf-8")
        batch = build / "compile_runtime_eye_audit.cmd"
        batch.write_text(f'''@echo off
call "{compiler_setup}" >nul
if errorlevel 1 exit /b %errorlevel%
cl /nologo /std:c++20 /EHsc /O2 /utf-8 /MD /D NOMINMAX /I "{installed / 'include'}" "{source}" /Fo"{build / 'runtime_eye_audit.obj'}" /Fe"{executable}" /link /LIBPATH:"{installed / 'lib'}" "{libraries[0]}"
exit /b %errorlevel%
''', encoding="ascii")
        subprocess.run([os.environ.get("ComSpec", "cmd.exe"), "/d", "/c", str(batch)], check=True, cwd=ROOT)
    environment = dict(os.environ)
    environment["PATH"] = str(installed / "bin") + os.pathsep + environment.get("PATH", "")
    fbx = ROOT / "assets/models/Raphtalia_prepared.fbx"
    raw = build / "fbx_rig.json"
    subprocess.run([str(executable), str(fbx), str(raw)], check=True, cwd=ROOT, env=environment)
    result = json.loads(raw.read_text(encoding="utf-8"))
    sys.path.insert(0, str(ROOT / "build/raphtalia_tools/bpy"))
    import bpy
    bpy.ops.wm.open_mainfile(filepath=str(ROOT / "assets/models/Raphtalia_prepared.blend"))
    arm = next(o for o in bpy.data.objects if o.type == "ARMATURE")
    assert len(arm.data.bones) == 429 and len(bpy.data.actions) == 15
    assert result["lid_count"] == 188 and result["lid_parents"] == ["head.x_018"]
    lids = [b for b in arm.data.bones if b.name.startswith("lid.")]
    result["lid_count_by_side"] = {side: sum(f".{side}." in b.name for b in lids) for side in ("L", "R")}
    drivers = [c for c in arm.animation_data.drivers if c.data_path.startswith('pose.bones["lid.')]
    assert len(drivers) == 188 * 3
    result["blender_source"] = {"bpy_version": bpy.app.version_string, "deforming_bones": len(arm.data.bones),
        "actions": len(bpy.data.actions), "eyelid_location_drivers": len(drivers),
        "eyelid_rotation_scale_locked": all(all(arm.pose.bones[b.name].lock_rotation)
            and all(arm.pose.bones[b.name].lock_scale) for b in lids),
        "driver_form": "delta * clamp(Blink_side,0,1) + depth_arc * sin(pi * clamp(Blink_side,0,1)); each side independent"}
    action = bpy.data.actions["TEST_blink_natural"]
    curve = next(c for c in action.fcurves if c.data_path == '["Blink_L"]')
    clip = next(c for c in result["clips"] if c["name"].endswith("TEST_blink_natural"))
    keys = clip["upper_center_keys"]
    open_y, closed_y = keys[0]["position"][1], keys[-1]["position"][1]
    errors = []
    for key in keys:
        closure = (key["position"][1] - open_y) / (closed_y - open_y)
        authored = float(curve.evaluate(key["ticks"] + 1))
        key["closure_from_local_y"] = closure
        key["authored_closure"] = authored
        errors.append(abs(closure - authored))
    assert all(keys[i]["closure_from_local_y"] < keys[i+1]["closure_from_local_y"] for i in range(len(keys)-1))
    assert max(errors) < 1e-5
    result["blink_parameterization"] = {"source_clip_suffix": "TEST_blink_natural",
        "probe_bone": "lid.upper.L.16", "linear_coordinate": "local translation Y",
        "source_ticks": [8, 11], "source_seconds": [8/30, 11/30], "authored_frames": [9, 12],
        "open_y": open_y, "closed_y": closed_y, "key_count": len(keys),
        "max_error_against_blender_curve": max(errors), "strictly_monotone": True,
        "center_half_closed_depth_arc_source_units": keys[6]["position"][2] - .5 * (keys[0]["position"][2] + keys[-1]["position"][2]),
        "mapping": "b = (sampled_local_Y - open_Y) / (closed_Y - open_Y); invert piecewise linear exported keys, then sample all 188 lid channels at that source time",
        "depth_arc": "Local Z carries the nonlinear depth arc; a direct linear open/closed blend drops it."}
    result["runtime_axes_contract"] = {
        "space": "Assimp scene/model space before the desktop fit/model transform",
        "head_rest_axes": "+X horizontal, +Y up, +Z forward (within imported float tolerance)",
        "eye_local_axes": "+X horizontal, +Z up, -Y forward; same on both eyes",
        "eye_bind_rotation_wxyz": next(n for n in result["nodes"] if n["name"] == "eye.L")["local_q"],
        "gaze_rotation_in_head_local": "R_y(yaw) * R_x(-pitch_up) * eye_bind_R",
        "authored_pitch_sign": "Positive authored X pitch lowers gaze. A positive-up API must negate it.",
        "previously_visual_checked_limits_degrees": {"yaw": 12, "pitch": 8, "diagonal": [10, 6]},
        "composition": "Base animation local matrices; eye-only gaze; additive lid translation relative to source-open; parent-first globals; existing palette. Suppress layers for authored competing eye/blink channels."}
    after = check_manifest()
    assert before == after
    result["deliverable_manifest"] = after
    result["all_seven_deliverables_unchanged"] = True
    result["audit_scope"] = "Fresh read-only Assimp import and saved Blender controls; this rig audit does not claim visual validation of the new desktop behavior."
    output = (ROOT / args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"PASS: 429 bones, 15 clips, 188 eyelid bones, {len(keys)} closing keys, seven preserved assets: {output}")


if __name__ == "__main__":
    main()
