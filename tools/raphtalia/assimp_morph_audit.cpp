// Read-only inspection of actual FBX morph arrays and animation keys.
// Complements assimp_audit.cpp; deliberately does not claim renderer support.
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/config.h>
#include <assimp/version.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
std::string utf8(const fs::path& path) { const auto s=path.u8string(); return {s.begin(),s.end()}; }
std::string quote(const std::string& value) {
    std::ostringstream out; out << '"';
    for (unsigned char c:value) {
        if(c=='"'||c=='\\') out << '\\' << c;
        else if(c<32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c) << std::dec;
        else out << c;
    }
    return out.str()+'"';
}
std::string number(double value) { if(!std::isfinite(value))return "null";std::ostringstream out;out<<std::setprecision(12)<<value;return out.str(); }
bool finite(const aiVector3D& value) { return std::isfinite(value.x)&&std::isfinite(value.y)&&std::isfinite(value.z); }
void nodeMeshCandidates(const aiNode* node,const std::string& prefix,std::vector<unsigned>& meshes) {
    if(node->mName.C_Str()==prefix)for(unsigned m=0;m<node->mNumMeshes;++m)meshes.push_back(node->mMeshes[m]);
    for(unsigned c=0;c<node->mNumChildren;++c)nodeMeshCandidates(node->mChildren[c],prefix,meshes);
}
void mode(std::ostream& out,const fs::path& input,bool project) {
    Assimp::Importer importer;
    unsigned flags=aiProcess_ValidateDataStructure|aiProcess_PopulateArmatureData;
    if(project) {
        flags|=aiProcess_Triangulate|aiProcess_GenSmoothNormals|aiProcess_JoinIdenticalVertices|aiProcess_LimitBoneWeights|aiProcess_FlipUVs|aiProcess_GenUVCoords|aiProcess_TransformUVCoords|aiProcess_FindInvalidData|aiProcess_FindDegenerates|aiProcess_SortByPType|aiProcess_ImproveCacheLocality;
        importer.SetPropertyInteger(AI_CONFIG_PP_LBW_MAX_WEIGHTS,4);
        importer.SetPropertyFloat(AI_CONFIG_PP_GSN_MAX_SMOOTHING_ANGLE,75.f);
        importer.SetPropertyBool(AI_CONFIG_PP_FD_CHECKAREA,false);
        importer.SetPropertyBool(AI_CONFIG_PP_FID_IGNORE_TEXTURECOORDS,true);
        importer.SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE,aiPrimitiveType_POINT|aiPrimitiveType_LINE);
    }
    const auto* scene=importer.ReadFile(utf8(input),flags);
    out<<"{\"loaded\":"<<(scene?"true":"false")<<",\"flags\":"<<flags<<",\"message\":"<<quote(importer.GetErrorString());
    if(!scene) {out<<'}';return;}
    unsigned targets=0,channels=0,keys=0,invalid=0;
    out<<",\"meshes\":[";
    for(unsigned m=0;m<scene->mNumMeshes;++m) {
        const auto* mesh=scene->mMeshes[m]; if(m)out<<',';
        out<<"{\"index\":"<<m<<",\"name\":"<<quote(mesh->mName.C_Str())<<",\"vertices\":"<<mesh->mNumVertices<<",\"morph_method\":"<<mesh->mMethod<<",\"targets\":[";
        for(unsigned a=0;a<mesh->mNumAnimMeshes;++a) {
            if(a)out<<',';const auto* target=mesh->mAnimMeshes[a];++targets;
            const bool validCount=target->mNumVertices==mesh->mNumVertices;
            unsigned changed=0,nonfinite=0;double maxDelta=0,sumSquares=0,maxNormalDelta=0;
            if(!validCount)++invalid;
            if(target->mVertices&&mesh->mVertices&&validCount)for(unsigned v=0;v<mesh->mNumVertices;++v) {
                if(!finite(target->mVertices[v])){++nonfinite;continue;}
                const double delta=(target->mVertices[v]-mesh->mVertices[v]).Length();
                if(delta>1e-9)++changed;maxDelta=std::max(maxDelta,delta);sumSquares+=delta*delta;
            }
            if(target->mNormals&&mesh->mNormals&&validCount)for(unsigned v=0;v<mesh->mNumVertices;++v) {
                if(!finite(target->mNormals[v])){++nonfinite;continue;}
                maxNormalDelta=std::max(maxNormalDelta,static_cast<double>((target->mNormals[v]-mesh->mNormals[v]).Length()));
            }
            invalid+=nonfinite;
            out<<"{\"index\":"<<a<<",\"name\":"<<quote(target->mName.C_Str())<<",\"vertices\":"<<target->mNumVertices<<",\"vertex_count_matches\":"<<(validCount?"true":"false")<<",\"positions_present\":"<<(target->mVertices?"true":"false")<<",\"normals_present\":"<<(target->mNormals?"true":"false")<<",\"default_weight\":"<<number(target->mWeight)<<",\"changed_position_vertices_epsilon_1e_9\":"<<changed<<",\"max_position_delta_mesh_units\":"<<number(maxDelta)<<",\"rms_position_delta_mesh_units\":"<<number(mesh->mNumVertices?std::sqrt(sumSquares/mesh->mNumVertices):0)<<",\"max_normal_delta\":"<<number(maxNormalDelta)<<",\"nonfinite_components\":"<<nonfinite<<'}';
        }
        out<<"]}";
    }
    out<<"],\"animations\":[";
    for(unsigned a=0;a<scene->mNumAnimations;++a) {
        if(a)out<<',';const auto* clip=scene->mAnimations[a];
        out<<"{\"name\":"<<quote(clip->mName.C_Str())<<",\"duration_ticks\":"<<number(clip->mDuration)<<",\"ticks_per_second\":"<<number(clip->mTicksPerSecond)<<",\"node_channel_count\":"<<clip->mNumChannels<<",\"legacy_mesh_channel_count\":"<<clip->mNumMeshChannels<<",\"morph_channels\":[";
        for(unsigned c=0;c<clip->mNumMorphMeshChannels;++c) {
            if(c)out<<',';const auto* channel=clip->mMorphMeshChannels[c];++channels;
            std::vector<unsigned> matches;
            for(unsigned m=0;m<scene->mNumMeshes;++m)if(scene->mMeshes[m]->mName==channel->mName)matches.push_back(m);
            const std::string name=channel->mName.C_Str();const auto separator=name.find('*');
            std::vector<unsigned> nodeCandidates;nodeMeshCandidates(scene->mRootNode,name.substr(0,separator),nodeCandidates);
            out<<"{\"name\":"<<quote(channel->mName.C_Str())<<",\"exact_mesh_name_matches\":[";
            for(unsigned i=0;i<matches.size();++i){if(i)out<<',';out<<matches[i];}
            out<<"],\"node_prefix_mesh_candidates_not_resolved\":[";
            for(unsigned i=0;i<nodeCandidates.size();++i){if(i)out<<',';out<<nodeCandidates[i];}
            out<<"],\"keys\":[";double previous=-std::numeric_limits<double>::infinity();
            for(unsigned k=0;k<channel->mNumKeys;++k) {
                if(k)out<<',';const auto& key=channel->mKeys[k];++keys;
                if(!std::isfinite(key.mTime)||key.mTime<previous)++invalid;previous=key.mTime;
                out<<"{\"time\":"<<number(key.mTime)<<",\"values\":[";
                for(unsigned w=0;w<key.mNumValuesAndWeights;++w) {
                    if(w)out<<',';if(!std::isfinite(key.mWeights[w]))++invalid;
                    bool validIndex=true;for(unsigned m:matches)if(key.mValues[w]>=scene->mMeshes[m]->mNumAnimMeshes)validIndex=false;
                    if(!validIndex)++invalid;
                    out<<"{\"target\":"<<key.mValues[w]<<",\"weight\":"<<number(key.mWeights[w])<<",\"index_in_range_for_exact_matches\":"<<(matches.empty()?"null":validIndex?"true":"false")<<'}';
                }
                out<<"]}";
            }
            out<<"]}";
        }
        out<<"]}";
    }
    out<<"],\"morph_target_count\":"<<targets<<",\"morph_channel_count\":"<<channels<<",\"morph_key_count\":"<<keys<<",\"invalid_entries\":"<<invalid<<'}';
}
int main(int argc,char** argv) {
    if(argc!=3){std::cerr<<"Usage: assimp_morph_audit input.fbx output.json\n";return 2;}
    try {
        const auto input=fs::absolute(fs::u8path(argv[1])),output=fs::absolute(fs::u8path(argv[2]));
        if(input==output)throw std::runtime_error("Output must differ from input");
        fs::create_directories(output.parent_path());std::ofstream out(output,std::ios::binary);if(!out)throw std::runtime_error("Cannot open output");
        out<<"{\"input\":"<<quote(utf8(input))<<",\"assimp_version\":"<<quote(std::to_string(aiGetVersionMajor())+"."+std::to_string(aiGetVersionMinor())+"."+std::to_string(aiGetVersionPatch()))<<",\"scope\":\"Actual aiAnimMesh positions/normals and all aiMeshMorphAnim keys in two import modes. Exact mesh-name matches are reported, not assumed. No Blender comparison, CPU morph evaluation, skinning composition or OpenGL morph support is claimed.\",\"source_validation\":";
        mode(out,input,false);out<<",\"project_import\":";mode(out,input,true);out<<"}\n";
        std::cout<<"Morph evidence written: "<<utf8(output)<<'\n';return 0;
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
