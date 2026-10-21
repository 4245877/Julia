// Read-only CPU audit: no OpenGL context and no re-export of the source asset.
#include <assimp/Importer.hpp>
#include <assimp/config.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/material.h>
#include <assimp/version.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

namespace fs = std::filesystem;
struct Json {
    using Array = std::vector<Json>;
    using Object = std::map<std::string, Json>;
    std::variant<std::nullptr_t, bool, double, std::string, Array, Object> v = nullptr;
    Json() = default;
    Json(bool x) : v(x) {}
    Json(double x) : v(std::isfinite(x) ? decltype(v){x} : decltype(v){nullptr}) {}
    Json(float x) : Json(static_cast<double>(x)) {}
    Json(unsigned x) : Json(static_cast<double>(x)) {}
    Json(int x) : Json(static_cast<double>(x)) {}
    Json(std::size_t x) : Json(static_cast<double>(x)) {}
    Json(const std::string& x) : v(x) {}
    Json(const char* x) : v(std::string{x}) {}
    static Json array() { Json x; x.v = Array{}; return x; }
    static Json object() { Json x; x.v = Object{}; return x; }
    Json& operator[](const std::string& k) { if (!std::holds_alternative<Object>(v)) v = Object{}; return std::get<Object>(v)[k]; }
    void push(Json x) { if (!std::holds_alternative<Array>(v)) v = Array{}; std::get<Array>(v).push_back(std::move(x)); }
    static void string(std::ostream& out, const std::string& s) {
        out << '"';
        for (const unsigned char c : s) {
            switch (c) {
            case '"': out << "\\\""; break; case '\\': out << "\\\\"; break;
            case '\n': out << "\\n"; break; case '\r': out << "\\r"; break; case '\t': out << "\\t"; break;
            default: if (c < 32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c) << std::dec << std::setfill(' '); else out << c;
            }
        }
        out << '"';
    }
    void write(std::ostream& out, int depth = 0) const {
        const auto indent = [&](int d) { out << std::string(static_cast<std::size_t>(d) * 2, ' '); };
        if (std::holds_alternative<std::nullptr_t>(v)) out << "null";
        else if (const auto p = std::get_if<bool>(&v)) out << (*p ? "true" : "false");
        else if (const auto p = std::get_if<double>(&v)) out << std::setprecision(12) << *p;
        else if (const auto p = std::get_if<std::string>(&v)) string(out, *p);
        else if (const auto p = std::get_if<Array>(&v)) {
            out << '['; for (std::size_t i = 0; i < p->size(); ++i) { if (i) out << ','; out << '\n'; indent(depth+1); (*p)[i].write(out,depth+1); }
            if (!p->empty()) { out << '\n'; indent(depth); } out << ']';
        } else {
            const auto& values = std::get<Object>(v); out << '{'; std::size_t i=0;
            for (const auto& [k,x] : values) { if(i++) out << ','; out << '\n'; indent(depth+1); string(out,k); out << ": "; x.write(out,depth+1); }
            if (!values.empty()) { out << '\n'; indent(depth); } out << '}';
        }
    }
};
std::string utf8(const fs::path& p) { const auto s=p.u8string(); return {s.begin(),s.end()}; }
bool finite(const aiVector3D& p) { return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z); }
bool finite(const aiMatrix4x4& m) { for(int r=0;r<4;++r) for(int c=0;c<4;++c) if(!std::isfinite(m[r][c])) return false; return true; }
Json vec(const aiVector3D& v) { Json j=Json::array(); j.push(v.x); j.push(v.y); j.push(v.z); return j; }
Json matrix(const aiMatrix4x4& m) { Json j=Json::array(); for(int r=0;r<4;++r) for(int c=0;c<4;++c) j.push(m[r][c]); return j; }
struct Bounds {
    bool valid=false; aiVector3D low{},high{};
    void add(const aiVector3D& p) { if(!finite(p)) return; if(!valid){low=high=p;valid=true;} else for(unsigned i=0;i<3;++i){low[i]=std::min(low[i],p[i]);high[i]=std::max(high[i],p[i]);} }
    Json json() const {Json j; j["valid"]=valid; if(valid){j["min"]=vec(low);j["max"]=vec(high);j["size"]=vec(high-low);}return j;}
};
struct Node { const aiNode* ptr=nullptr; int parent=-1; aiMatrix4x4 global; };
struct Instance { unsigned mesh=0,node=0; };
struct SceneData {
    const aiScene* scene;
    std::vector<Node> nodes;
    std::vector<Instance> instances;
    std::unordered_map<const aiNode*,unsigned> pointer;
    std::map<std::string,std::vector<unsigned>> names;
    explicit SceneData(const aiScene* s):scene(s){walk(s->mRootNode,-1);}
    void walk(const aiNode* n,int parent){
        const unsigned i=static_cast<unsigned>(nodes.size()); aiMatrix4x4 global=parent<0?n->mTransformation:nodes[static_cast<unsigned>(parent)].global*n->mTransformation;
        nodes.push_back({n,parent,global});pointer[n]=i;names[n->mName.C_Str()].push_back(i);
        for(unsigned k=0;k<n->mNumMeshes;++k) instances.push_back({n->mMeshes[k],i});
        for(unsigned k=0;k<n->mNumChildren;++k)walk(n->mChildren[k],static_cast<int>(i));
    }
    int boneNode(const aiBone* b)const{
#ifndef ASSIMP_BUILD_NO_ARMATUREPOPULATE_PROCESS
        if(b->mNode){const auto it=pointer.find(b->mNode);if(it!=pointer.end())return static_cast<int>(it->second);}
#endif
        const auto it=names.find(b->mName.C_Str());return it!=names.end()&&it->second.size()==1?static_cast<int>(it->second[0]):-1;
    }
    int uniqueNode(const std::string& s)const{const auto it=names.find(s);return it!=names.end()&&it->second.size()==1?static_cast<int>(it->second[0]):-1;}
};
aiVector3D sampleVector(const aiVectorKey* keys,unsigned n,double time,const aiVector3D& fallback){
    if(!n)return fallback;if(n==1||time<=keys[0].mTime)return keys[0].mValue;
    for(unsigned i=1;i<n;++i)if(time<keys[i].mTime){const double d=keys[i].mTime-keys[i-1].mTime;const float f=d>0?static_cast<float>((time-keys[i-1].mTime)/d):0;return keys[i-1].mValue*(1-f)+keys[i].mValue*f;}
    return keys[n-1].mValue;
}
aiQuaternion sampleQuaternion(const aiQuatKey* keys,unsigned n,double time,const aiQuaternion& fallback){
    if(!n)return fallback;if(n==1||time<=keys[0].mTime)return keys[0].mValue;
    for(unsigned i=1;i<n;++i)if(time<keys[i].mTime){const double d=keys[i].mTime-keys[i-1].mTime;aiQuaternion q;aiQuaternion::Interpolate(q,keys[i-1].mValue,keys[i].mValue,d>0?static_cast<float>((time-keys[i-1].mTime)/d):0);q.Normalize();return q;}
    return keys[n-1].mValue;
}
std::vector<aiMatrix4x4> pose(const SceneData& d,const aiAnimation* clip,double time){
    std::vector<const aiNodeAnim*> channels(d.nodes.size(),nullptr);
    if(clip)for(unsigned c=0;c<clip->mNumChannels;++c){const int n=d.uniqueNode(clip->mChannels[c]->mNodeName.C_Str());if(n>=0)channels[static_cast<unsigned>(n)]=clip->mChannels[c];}
    std::vector<aiMatrix4x4> out;out.reserve(d.nodes.size());
    for(unsigned n=0;n<d.nodes.size();++n){aiMatrix4x4 local=d.nodes[n].ptr->mTransformation;
        if(const auto* c=channels[n]){aiVector3D scale,position;aiQuaternion rotation;local.Decompose(scale,rotation,position);local=aiMatrix4x4(sampleVector(c->mScalingKeys,c->mNumScalingKeys,time,scale),sampleQuaternion(c->mRotationKeys,c->mNumRotationKeys,time,rotation),sampleVector(c->mPositionKeys,c->mNumPositionKeys,time,position));}
        out.push_back(d.nodes[n].parent<0?local:out[static_cast<unsigned>(d.nodes[n].parent)]*local);
    }return out;
}
struct Deformed {std::vector<aiVector3D> position;std::vector<double> weights;unsigned invalid=0;};
Deformed deform(const SceneData& d,const aiMesh* mesh,const aiMatrix4x4& meshGlobal,const std::vector<aiMatrix4x4>& globals){
    Deformed result;result.position.resize(mesh->mNumVertices);result.weights.resize(mesh->mNumVertices,0);
    for(unsigned b=0;b<mesh->mNumBones;++b){const auto* bone=mesh->mBones[b];const int node=d.boneNode(bone);if(node<0){++result.invalid;continue;}const auto transform=globals[static_cast<unsigned>(node)]*bone->mOffsetMatrix;
        for(unsigned w=0;w<bone->mNumWeights;++w){const auto& weight=bone->mWeights[w];if(weight.mVertexId>=mesh->mNumVertices||!std::isfinite(weight.mWeight)||weight.mWeight<0){++result.invalid;continue;}const auto p=transform*mesh->mVertices[weight.mVertexId];result.position[weight.mVertexId]+=p*weight.mWeight;result.weights[weight.mVertexId]+=weight.mWeight;}
    }
    for(unsigned v=0;v<mesh->mNumVertices;++v){if(result.weights[v]>0)result.position[v]/=static_cast<float>(result.weights[v]);else result.position[v]=meshGlobal*mesh->mVertices[v];}
    return result;
}
Json audit(const fs::path& path,bool project){
    Assimp::Importer importer;
    unsigned flags=aiProcess_ValidateDataStructure|aiProcess_PopulateArmatureData;
    if(project){flags|=aiProcess_Triangulate|aiProcess_GenSmoothNormals|aiProcess_JoinIdenticalVertices|aiProcess_LimitBoneWeights|aiProcess_FlipUVs|aiProcess_GenUVCoords|aiProcess_TransformUVCoords|aiProcess_FindInvalidData|aiProcess_FindDegenerates|aiProcess_SortByPType|aiProcess_ImproveCacheLocality;
        importer.SetPropertyInteger(AI_CONFIG_PP_LBW_MAX_WEIGHTS,4);importer.SetPropertyFloat(AI_CONFIG_PP_GSN_MAX_SMOOTHING_ANGLE,75.f);importer.SetPropertyBool(AI_CONFIG_PP_FD_CHECKAREA,false);importer.SetPropertyBool(AI_CONFIG_PP_FID_IGNORE_TEXTURECOORDS,true);importer.SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE,aiPrimitiveType_POINT|aiPrimitiveType_LINE);
    }
    Json out;out["mode"]=project?"project_postprocess_four_weights":"source_validation_only";out["flags"]=flags;
    const auto* scene=importer.ReadFile(utf8(path),flags);
    out["loaded"]=scene!=nullptr;out["assimp_message"]=importer.GetErrorString();if(!scene)return out;
    out["scene_flags"]=scene->mFlags;out["validation_warning"]=(scene->mFlags&AI_SCENE_FLAGS_VALIDATION_WARNING)!=0;out["incomplete"]=(scene->mFlags&AI_SCENE_FLAGS_INCOMPLETE)!=0;
    SceneData data(scene);out["node_count"]=data.nodes.size();out["mesh_count"]=scene->mNumMeshes;out["mesh_instance_count"]=data.instances.size();out["material_count"]=scene->mNumMaterials;out["embedded_texture_count"]=scene->mNumTextures;out["animation_count"]=scene->mNumAnimations;
    Json nodes=Json::array(),duplicates=Json::array();unsigned nonfiniteNode=0;
    for(unsigned n=0;n<data.nodes.size();++n){Json j;j["index"]=n;j["parent"]=data.nodes[n].parent;j["name"]=data.nodes[n].ptr->mName.C_Str();j["local_matrix_row_major"]=matrix(data.nodes[n].ptr->mTransformation);j["global_matrix_row_major"]=matrix(data.nodes[n].global);nodes.push(j);if(!finite(data.nodes[n].global))++nonfiniteNode;}
    for(const auto& [name,indices]:data.names)if(indices.size()>1){Json j;j["name"]=name;j["count"]=indices.size();duplicates.push(j);}
    out["nodes"]=nodes;out["duplicate_node_names"]=duplicates;out["nonfinite_global_bind_matrices"]=nonfiniteNode;
    Json meshes=Json::array();std::size_t totalVertices=0,totalFaces=0,totalTriangles=0,totalWeights=0,totalUnweighted=0,totalNonunit=0,totalInvalidWeights=0,totalBoneEntries=0,totalMorphs=0;unsigned maxInfluences=0,unresolvedBones=0;std::set<std::string> boneNames;std::set<int> weightedNodes;
    for(unsigned m=0;m<scene->mNumMeshes;++m){const auto* mesh=scene->mMeshes[m];Json j;j["index"]=m;j["name"]=mesh->mName.C_Str();j["vertices"]=mesh->mNumVertices;j["faces"]=mesh->mNumFaces;j["material_index"]=mesh->mMaterialIndex;j["bone_entries"]=mesh->mNumBones;j["morph_target_count"]=mesh->mNumAnimMeshes;totalMorphs+=mesh->mNumAnimMeshes;
        std::vector<double>sums(mesh->mNumVertices);std::vector<unsigned>counts(mesh->mNumVertices);Json bones=Json::array(),morphNames=Json::array();std::size_t invalid=0,weights=0,triangles=0;
        for(unsigned f=0;f<mesh->mNumFaces;++f)if(mesh->mFaces[f].mNumIndices==3)++triangles;
        for(unsigned b=0;b<mesh->mNumBones;++b){const auto* bone=mesh->mBones[b];const int n=data.boneNode(bone);Json bj;bj["name"]=bone->mName.C_Str();bj["node_index"]=n;bj["weight_entries"]=bone->mNumWeights;bj["offset_matrix_finite"]=finite(bone->mOffsetMatrix);bones.push(bj);boneNames.insert(bone->mName.C_Str());if(n<0)++unresolvedBones;else weightedNodes.insert(n);
            for(unsigned w=0;w<bone->mNumWeights;++w){const auto& x=bone->mWeights[w];++weights;if(x.mVertexId>=sums.size()||!std::isfinite(x.mWeight)||x.mWeight<0){++invalid;continue;}if(x.mWeight>0){sums[x.mVertexId]+=x.mWeight;++counts[x.mVertexId];}}
        }
        unsigned maximum=0;std::size_t unweighted=0,nonunit=0;double minSum=std::numeric_limits<double>::infinity(),maxSum=0;
        for(unsigned v=0;v<mesh->mNumVertices;++v){maximum=std::max(maximum,counts[v]);if(counts[v]==0)++unweighted;else{if(std::abs(sums[v]-1)>1e-5)++nonunit;minSum=std::min(minSum,sums[v]);maxSum=std::max(maxSum,sums[v]);}}
        for(unsigned a=0;a<mesh->mNumAnimMeshes;++a)morphNames.push(mesh->mAnimMeshes[a]->mName.C_Str());
        j["triangles"]=triangles;j["max_positive_influences"]=maximum;j["unweighted_vertices"]=unweighted;j["nonunit_weight_vertices_epsilon_1e_5"]=nonunit;j["positive_weight_sum_min"]=minSum;j["positive_weight_sum_max"]=maxSum;j["invalid_weight_entries"]=invalid;j["weight_entries"]=weights;j["bones"]=bones;j["morph_target_names"]=morphNames;meshes.push(j);
        totalVertices+=mesh->mNumVertices;totalFaces+=mesh->mNumFaces;totalTriangles+=triangles;totalWeights+=weights;totalUnweighted+=unweighted;totalNonunit+=nonunit;totalInvalidWeights+=invalid;totalBoneEntries+=mesh->mNumBones;maxInfluences=std::max(maxInfluences,maximum);
    }
    out["meshes"]=meshes;out["vertices"]=totalVertices;out["faces"]=totalFaces;out["triangles"]=totalTriangles;out["weight_entries"]=totalWeights;out["unweighted_vertices"]=totalUnweighted;out["nonunit_weight_vertices_epsilon_1e_5"]=totalNonunit;out["invalid_weight_entries"]=totalInvalidWeights;out["max_positive_influences"]=maxInfluences;out["mesh_bone_entries"]=totalBoneEntries;out["unique_weighted_bone_names"]=boneNames.size();out["unique_weighted_nodes"]=weightedNodes.size();out["unresolved_bone_entries"]=unresolvedBones;out["morph_target_count"]=totalMorphs;Json names=Json::array();for(const auto& s:boneNames)names.push(s);out["bone_names"]=names;
    Bounds sceneBounds;Json binds=Json::array();auto globals=pose(data,nullptr,0);double globalMaxResidual=0;std::size_t globalNonfinite=0;
    for(const auto& instance:data.instances){if(instance.mesh>=scene->mNumMeshes)continue;const auto* mesh=scene->mMeshes[instance.mesh];const auto& meshGlobal=data.nodes[instance.node].global;const auto moved=deform(data,mesh,meshGlobal,globals);double maximum=0,sumSquares=0;std::size_t nonfinite=0;
        for(unsigned v=0;v<mesh->mNumVertices;++v){const auto expected=meshGlobal*mesh->mVertices[v];sceneBounds.add(expected);if(!finite(moved.position[v])){++nonfinite;continue;}const double residual=(moved.position[v]-expected).Length();maximum=std::max(maximum,residual);sumSquares+=residual*residual;}
        Json j;j["mesh"]=instance.mesh;j["node"]=instance.node;j["max_bind_position_residual_scene_units"]=maximum;j["rms_bind_position_residual_scene_units"]=mesh->mNumVertices?std::sqrt(sumSquares/mesh->mNumVertices):0.;j["nonfinite_deformed_vertices"]=nonfinite;j["invalid_deform_entries"]=moved.invalid;binds.push(j);globalMaxResidual=std::max(globalMaxResidual,maximum);globalNonfinite+=nonfinite;
    }
    out["bounds_imported_scene_units"]=sceneBounds.json();out["bind_pose_instances"]=binds;out["max_bind_position_residual_scene_units"]=globalMaxResidual;out["nonfinite_bind_deformed_vertices"]=globalNonfinite;
    Json materials=Json::array();for(unsigned m=0;m<scene->mNumMaterials;++m){const auto* material=scene->mMaterials[m];Json j;aiString name;material->Get(AI_MATKEY_NAME,name);j["name"]=name.C_Str();Json textures=Json::array();
        for(int type=1;type<=static_cast<int>(AI_TEXTURE_TYPE_MAX);++type)for(unsigned t=0;t<material->GetTextureCount(static_cast<aiTextureType>(type));++t){aiString texture;unsigned uv=0;material->GetTexture(static_cast<aiTextureType>(type),t,&texture,nullptr,&uv);Json tj;tj["type"]=type;tj["path"]=texture.C_Str();tj["uv_channel"]=uv;tj["embedded"]=scene->GetEmbeddedTexture(texture.C_Str())!=nullptr;fs::path ref=fs::u8path(texture.C_Str());std::error_code error;const fs::path direct=ref.is_absolute()?ref:path.parent_path()/ref;const fs::path sibling=path.parent_path().parent_path()/"textures"/ref.filename();tj["direct_exists"]=fs::is_regular_file(direct,error);error.clear();tj["sibling_textures_basename_exists"]=fs::is_regular_file(sibling,error);textures.push(tj);}
        j["textures"]=textures;materials.push(j);
    }out["materials"]=materials;
    Json animations=Json::array();for(unsigned a=0;a<scene->mNumAnimations;++a){const auto* clip=scene->mAnimations[a];Json j;j["name"]=clip->mName.C_Str();j["duration_ticks"]=clip->mDuration;j["ticks_per_second"]=clip->mTicksPerSecond;j["duration_seconds_if_rate_nonzero"]=clip->mTicksPerSecond>0?Json(clip->mDuration/clip->mTicksPerSecond):Json();j["node_channels"]=clip->mNumChannels;j["mesh_channels"]=clip->mNumMeshChannels;j["morph_channels"]=clip->mNumMorphMeshChannels;Json channels=Json::array();unsigned unresolvedChannels=0;std::size_t nonfiniteKeys=0;
        for(unsigned c=0;c<clip->mNumChannels;++c){const auto* channel=clip->mChannels[c];Json cj;cj["name"]=channel->mNodeName.C_Str();const int node=data.uniqueNode(channel->mNodeName.C_Str());cj["node_index"]=node;if(node<0)++unresolvedChannels;cj["position_keys"]=channel->mNumPositionKeys;cj["rotation_keys"]=channel->mNumRotationKeys;cj["scale_keys"]=channel->mNumScalingKeys;
            for(unsigned k=0;k<channel->mNumPositionKeys;++k)if(!std::isfinite(channel->mPositionKeys[k].mTime)||!finite(channel->mPositionKeys[k].mValue))++nonfiniteKeys;
            for(unsigned k=0;k<channel->mNumScalingKeys;++k)if(!std::isfinite(channel->mScalingKeys[k].mTime)||!finite(channel->mScalingKeys[k].mValue))++nonfiniteKeys;
            for(unsigned k=0;k<channel->mNumRotationKeys;++k){const auto& key=channel->mRotationKeys[k];const auto& q=key.mValue;if(!std::isfinite(key.mTime)||!std::isfinite(q.w)||!std::isfinite(q.x)||!std::isfinite(q.y)||!std::isfinite(q.z))++nonfiniteKeys;}channels.push(cj);
        }
        j["channels"]=channels;j["unresolved_or_ambiguous_channels"]=unresolvedChannels;j["nonfinite_key_entries"]=nonfiniteKeys;Json samples=Json::array();double clipMaxDisplacement=0;std::size_t clipNonfinite=0;
        for(unsigned s=0;s<9;++s){const double time=clip->mDuration*s/8.;const auto sampled=pose(data,clip,time);Json sj;sj["ticks"]=time;std::size_t nonfiniteMatrices=0,nonfiniteVertices=0;double displacement=0;Bounds sampleBounds;for(const auto& g:sampled)if(!finite(g))++nonfiniteMatrices;
            for(const auto& instance:data.instances){if(instance.mesh>=scene->mNumMeshes)continue;const auto* mesh=scene->mMeshes[instance.mesh];const auto& meshGlobal=data.nodes[instance.node].global;const auto deformed=deform(data,mesh,meshGlobal,sampled);for(unsigned v=0;v<mesh->mNumVertices;++v){const auto& p=deformed.position[v];if(!finite(p)){++nonfiniteVertices;continue;}sampleBounds.add(p);displacement=std::max(displacement,static_cast<double>((p-meshGlobal*mesh->mVertices[v]).Length()));}}
            sj["nonfinite_global_matrices"]=nonfiniteMatrices;sj["nonfinite_deformed_vertices"]=nonfiniteVertices;sj["max_displacement_from_bind_scene_units"]=displacement;sj["bounds"]=sampleBounds.json();samples.push(sj);clipMaxDisplacement=std::max(clipMaxDisplacement,displacement);clipNonfinite+=nonfiniteMatrices+nonfiniteVertices;
        }
        j["samples"]=samples;j["max_sampled_displacement_from_bind_scene_units"]=clipMaxDisplacement;j["sampled_nonfinite_total"]=clipNonfinite;animations.push(j);
    }out["animations"]=animations;
    out["sampling_scope"]="9 equally spaced times per clip; linear position/scale and quaternion slerp, endpoint clamp. Mesh/morph channels counted but not evaluated. Finite numerical deformation is not visual/anatomical validation.";
    return out;
}
int main(int argc,char** argv){
    if(argc!=3){std::cerr<<"Usage: assimp_audit input.fbx output.json\n";return 2;}
    try{const fs::path input=fs::absolute(fs::u8path(argv[1])),output=fs::absolute(fs::u8path(argv[2]));Json j;j["input"]=utf8(input);j["assimp_version"]=std::to_string(aiGetVersionMajor())+"."+std::to_string(aiGetVersionMinor())+"."+std::to_string(aiGetVersionPatch());j["matrix_layout"]="row-major in JSON; Assimp CPU matrices multiply column vectors";j["source_validation"]=audit(input,false);j["project_import"]=audit(input,true);fs::create_directories(output.parent_path());std::ofstream stream(output,std::ios::binary);if(!stream)throw std::runtime_error("Cannot write output");j.write(stream);stream<<'\n';std::cout<<"Assimp audit written: "<<utf8(output)<<'\n';const bool sourceLoaded=std::get<bool>(std::get<Json::Object>(j["source_validation"].v).at("loaded").v),projectLoaded=std::get<bool>(std::get<Json::Object>(j["project_import"].v).at("loaded").v);return sourceLoaded&&projectLoaded?0:1;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
