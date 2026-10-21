// Standalone GPU proof of exported skeletal clips. Does not alter application runtime.
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "julia/render_opengl/GLModelLoader.hpp"
#include "julia/render_opengl/OpenGLRenderer.hpp"
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs=std::filesystem;
using namespace julia::render_opengl;
constexpr int Width=768,Height=768;
using Pixels=std::vector<unsigned char>;
std::string utf8(const fs::path& p){const auto s=p.u8string();return {s.begin(),s.end()};}
glm::mat4 toGlm(const aiMatrix4x4& a){glm::mat4 r;for(int row=0;row<4;++row)for(int col=0;col<4;++col)r[col][row]=a[row][col];return r;}
bool finite(const glm::mat4& m){for(int c=0;c<4;++c)for(int r=0;r<4;++r)if(!std::isfinite(m[c][r]))return false;return true;}
void checkGl(const std::string& stage){const GLenum e=glGetError();if(e!=GL_NO_ERROR)throw std::runtime_error(stage+": OpenGL error "+std::to_string(e));}
aiVector3D sample(const aiVectorKey* k,unsigned count,double t,const aiVector3D& rest){
    if(!count)return rest;if(count==1||t<=k[0].mTime)return k[0].mValue;
    for(unsigned i=1;i<count;++i)if(t<k[i].mTime){const double delta=k[i].mTime-k[i-1].mTime;const float f=delta>0?static_cast<float>((t-k[i-1].mTime)/delta):0;return k[i-1].mValue*(1-f)+k[i].mValue*f;}return k[count-1].mValue;
}
aiQuaternion sample(const aiQuatKey* k,unsigned count,double t,const aiQuaternion& rest){
    if(!count)return rest;if(count==1||t<=k[0].mTime)return k[0].mValue;
    for(unsigned i=1;i<count;++i)if(t<k[i].mTime){const double delta=k[i].mTime-k[i-1].mTime;aiQuaternion q;aiQuaternion::Interpolate(q,k[i-1].mValue,k[i].mValue,delta>0?static_cast<float>((t-k[i-1].mTime)/delta):0);q.Normalize();return q;}return k[count-1].mValue;
}
std::vector<glm::mat4> localPose(const GLModel& model,const aiAnimation* clip,double time,unsigned& mapped,unsigned& unresolved){
    std::vector<glm::mat4> result;for(const auto& n:model.nodes())result.push_back(n.localTransform);
    std::map<std::string,std::vector<unsigned>> names;for(unsigned n=0;n<model.nodes().size();++n)names[model.nodes()[n].name].push_back(n);
    mapped=unresolved=0;
    for(unsigned c=0;c<clip->mNumChannels;++c){const auto* channel=clip->mChannels[c];const auto found=names.find(channel->mNodeName.C_Str());if(found==names.end()||found->second.size()!=1){++unresolved;continue;}const unsigned n=found->second[0];
        aiMatrix4x4 rest;for(int row=0;row<4;++row)for(int col=0;col<4;++col)rest[row][col]=result[n][col][row];aiVector3D scale,position;aiQuaternion rotation;rest.Decompose(scale,rotation,position);
        result[n]=toGlm(aiMatrix4x4(sample(channel->mScalingKeys,channel->mNumScalingKeys,time,scale),sample(channel->mRotationKeys,channel->mNumRotationKeys,time,rotation),sample(channel->mPositionKeys,channel->mNumPositionKeys,time,position)));if(!finite(result[n]))throw std::runtime_error("Nonfinite local pose for "+model.nodes()[n].name);++mapped;
    }return result;
}
Pixels readFrame(){Pixels p(static_cast<std::size_t>(Width)*Height*3);glReadBuffer(GL_BACK);glPixelStorei(GL_PACK_ALIGNMENT,1);glReadPixels(0,0,Width,Height,GL_RGB,GL_UNSIGNED_BYTE,p.data());checkGl("readback");return p;}
void save(const fs::path& path,const Pixels& p){std::ofstream f(path,std::ios::binary);if(!f)throw std::runtime_error("Cannot write image");f<<"P6\n"<<Width<<' '<<Height<<"\n255\n";for(int y=Height-1;y>=0;--y)f.write(reinterpret_cast<const char*>(p.data()+static_cast<std::size_t>(y)*Width*3),Width*3);}
std::size_t changed(const Pixels& a,const Pixels& b){std::size_t n=0;for(std::size_t i=0;i<a.size();++i)if(a[i]!=b[i])++n;return n;}
std::string slug(std::string s){for(char& c:s)if(!((c>='A'&&c<='Z')||(c>='a'&&c<='z')||(c>='0'&&c<='9')||c=='_'))c='_';if(s.empty())s="unnamed";return s;}
float matrixDelta(const std::vector<glm::mat4>& a,const std::vector<glm::mat4>& b){float max=0;for(std::size_t i=0;i<a.size();++i)for(int c=0;c<4;++c)for(int r=0;r<4;++r)max=std::max(max,std::abs(a[i][c][r]-b[i][c][r]));return max;}
glm::mat4 blendTRS(const glm::mat4& a,const glm::mat4& b,float t){
    aiMatrix4x4 am,bm;for(int r=0;r<4;++r)for(int c=0;c<4;++c){am[r][c]=a[c][r];bm[r][c]=b[c][r];}aiVector3D as,ap,bs,bp;aiQuaternion aq,bq,q;am.Decompose(as,aq,ap);bm.Decompose(bs,bq,bp);aiQuaternion::Interpolate(q,aq,bq,t);q.Normalize();return toGlm(aiMatrix4x4(as*(1-t)+bs*t,q,ap*(1-t)+bp*t));
}
int main(int argc,char** argv){
    if(argc!=4){std::cerr<<"Usage: gpu_pose_audit input.fbx assets_root output_directory\n";return 2;}
    GLFWwindow* window=nullptr;
    try{
        const fs::path modelPath=fs::absolute(fs::u8path(argv[1])),assets=fs::absolute(fs::u8path(argv[2])),out=fs::absolute(fs::u8path(argv[3]));fs::create_directories(out);
        glfwSetErrorCallback([](int e,const char* s){std::cerr<<"GLFW "<<e<<": "<<(s?s:"")<<'\n';});if(!glfwInit())throw std::runtime_error("Cannot initialize GLFW");glfwWindowHint(GLFW_VISIBLE,GLFW_FALSE);glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3);glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,3);glfwWindowHint(GLFW_OPENGL_PROFILE,GLFW_OPENGL_CORE_PROFILE);glfwWindowHint(GLFW_SRGB_CAPABLE,GLFW_TRUE);glfwWindowHint(GLFW_SAMPLES,4);
        window=glfwCreateWindow(Width,Height,"Raphtalia skeletal audit",nullptr,nullptr);if(!window)throw std::runtime_error("Cannot create hidden GL context");glfwMakeContextCurrent(window);glfwSwapInterval(0);if(!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)))throw std::runtime_error("Cannot initialize GLAD");
        {
            std::ofstream report(out/"gpu_audit.txt"),csv(out/"gpu_samples.csv"),endpoints(out/"gpu_endpoints.csv");if(!report||!csv||!endpoints)throw std::runtime_error("Cannot write reports");GLint maxTexels=0,samples=0;glGetIntegerv(GL_MAX_TEXTURE_BUFFER_SIZE,&maxTexels);glGetIntegerv(GL_SAMPLES,&samples);
            report<<std::setprecision(10)<<"input="<<utf8(modelPath)<<"\nGL_VENDOR="<<glGetString(GL_VENDOR)<<"\nGL_RENDERER="<<glGetString(GL_RENDERER)<<"\nGL_VERSION="<<glGetString(GL_VERSION)<<"\nGLSL_VERSION="<<glGetString(GL_SHADING_LANGUAGE_VERSION)<<"\nGL_MAX_TEXTURE_BUFFER_SIZE="<<maxTexels<<"\nmax_palette_matrices="<<maxTexels/4<<"\nresolution="<<Width<<'x'<<Height<<"\nMSAA="<<samples<<'\n';
            OpenGLRenderer renderer;renderer.initialize(Width,Height,assets);auto model=GLModelLoader::loadFromFile(modelPath);const auto fit=model.fitToHeightTransform(2.2f);const auto bind=model.skinningMatrices();if(bind.empty())throw std::runtime_error("No imported skinning palette");
            Assimp::Importer importer;const auto* scene=importer.ReadFile(utf8(modelPath),aiProcess_ValidateDataStructure);if(!scene)throw std::runtime_error(importer.GetErrorString());
            report<<"meshes="<<model.meshCount()<<"\nnodes="<<model.nodes().size()<<"\npalette_entries="<<model.boneCount()<<"\nanimation_clips="<<scene->mNumAnimations<<"\npose_sampler=linear_TRS_quaternion_slerp_endpoint_clamp\n";
            const glm::vec3 center=glm::vec3{fit*glm::vec4{model.bounds().center(),1}};glm::vec3 faceTarget=center+glm::vec3{0,.7f,0};for(const auto& node:model.nodes())if(node.name.rfind("head.x",0)==0){faceTarget=glm::vec3{fit*node.globalBindTransform*glm::vec4{0,0,0,1}}+glm::vec3{0,.12f,0};break;}
            std::array<CameraConfig,4> cameras;for(unsigned v=0;v<4;++v){cameras[v].target=v>=2?faceTarget:center;cameras[v].position=cameras[v].target+glm::vec3{0,v>=2?.015f:.1f,(v%2==0?1.f:-1.f)*(v>=2?.82f:3.6f)};}
            const std::array<std::string,4> viewNames{"front","back","face_front","face_back"};std::array<Pixels,4> reference;
            const auto render=[&](unsigned view,const std::vector<glm::mat4>& palette){renderer.setCamera(cameras[view]);renderer.beginFrame();renderer.renderModel(model,fit,palette);checkGl("render");return readFrame();};
            for(unsigned v=0;v<4;++v){reference[v]=render(v,bind);save(out/("gpu_neutral_"+viewNames[v]+".ppm"),reference[v]);}
            csv<<"clip_index,clip_name,fraction,time_ticks,mapped_channels,unresolved_channels,max_palette_delta,view,changed_rgb_bytes_vs_bind,image\n";endpoints<<"clip_index,clip_name,view,start_vs_bind_changed_rgb_bytes,end_vs_start_changed_rgb_bytes,start_vs_bind_max_palette_delta,end_vs_start_max_palette_delta\n";unsigned rendered=4,changedClipCount=0;
            for(unsigned a=0;a<scene->mNumAnimations;++a){const auto* clip=scene->mAnimations[a];const std::string name=slug(clip->mName.C_Str());bool clipChanged=false;
                const bool practical=std::string(clip->mName.C_Str()).ends_with("TEST_practical_ranges");
                const std::vector<double> fractions=practical?std::vector<double>{.125,.375,.625,.875}:std::vector<double>{.25,.5,.75};
                for(const double fraction:fractions){unsigned mapped=0,unresolved=0;const auto local=localPose(model,clip,clip->mDuration*fraction,mapped,unresolved);if(unresolved)throw std::runtime_error("Unresolved animation channels in "+name);const auto palette=model.skinningMatrices(local);for(const auto& matrix:palette)if(!finite(matrix))throw std::runtime_error("Nonfinite palette");const auto delta=matrixDelta(palette,bind);
                    // Full-body front/back at all samples; face closeups at midpoint.
                    for(unsigned v=0;v<((practical?fraction==.875:fraction==.5)?4u:2u);++v){const auto pixels=render(v,palette);const auto difference=changed(reference[v],pixels);clipChanged|=difference>0;const std::string file="gpu_"+std::to_string(a)+"_"+name+"_"+std::to_string(static_cast<int>(fraction*100))+"_"+viewNames[v]+".ppm";save(out/file,pixels);csv<<a<<','<<name<<','<<fraction<<','<<clip->mDuration*fraction<<','<<mapped<<','<<unresolved<<','<<delta<<','<<viewNames[v]<<','<<difference<<','<<file<<'\n';++rendered;}
                }
                unsigned mapped=0,unresolved=0;const auto firstPalette=model.skinningMatrices(localPose(model,clip,0,mapped,unresolved));const auto lastPalette=model.skinningMatrices(localPose(model,clip,clip->mDuration,mapped,unresolved));std::size_t endpointChanged=0;
                for(unsigned v=0;v<4;++v){const auto first=render(v,firstPalette);const auto last=render(v,lastPalette);const auto change=changed(first,last);endpointChanged+=change;endpoints<<a<<','<<name<<','<<viewNames[v]<<','<<changed(reference[v],first)<<','<<change<<','<<matrixDelta(firstPalette,bind)<<','<<matrixDelta(lastPalette,firstPalette)<<'\n';}
                if(clipChanged)++changedClipCount;report<<"clip="<<name<<" ticks="<<clip->mDuration<<" tps="<<clip->mTicksPerSecond<<" changed_pixels="<<(clipChanged?"YES":"NO")<<" endpoint_start_end_rgb_differences="<<endpointChanged<<'\n';
            }
            // Evaluation-only example of local-space masks and TRS crossfade.
            // No animation player or blend logic is added to the application.
            const auto findClip=[&](const std::string& suffix)->const aiAnimation*{for(unsigned i=0;i<scene->mNumAnimations;++i)if(std::string(scene->mAnimations[i]->mName.C_Str()).ends_with(suffix))return scene->mAnimations[i];return nullptr;};
            const auto* elbows=findClip("TEST_elbows_90");const auto* knees=findClip("TEST_knees_90");const auto* head=findClip("TEST_head_torso");const auto* fingers=findClip("TEST_fingers_35");const auto* secondary=findClip("TEST_hair_tail_cloth");
            if(elbows&&knees&&head&&fingers&&secondary){
                unsigned mapped=0,unresolved=0;const auto elbowPose=localPose(model,elbows,elbows->mDuration*.5,mapped,unresolved);const auto kneePose=localPose(model,knees,knees->mDuration*.5,mapped,unresolved);const auto headPose=localPose(model,head,head->mDuration*.5,mapped,unresolved);const auto fingerPose=localPose(model,fingers,fingers->mDuration*.5,mapped,unresolved);const auto secondaryPose=localPose(model,secondary,secondary->mDuration*.5,mapped,unresolved);
                std::vector<glm::mat4> combined;for(const auto& node:model.nodes())combined.push_back(node.localTransform);unsigned maskNodes=0;
                for(unsigned n=0;n<model.nodes().size();++n){const auto& name=model.nodes()[n].name;
                    if(name=="forearm_stretch.l_0159"||name=="forearm_stretch.r_0138"){combined[n]=elbowPose[n];++maskNodes;}
                    else if(name=="spine_02.x_015"||name=="head.x_018"){combined[n]=headPose[n];++maskNodes;}
                    else if(name.starts_with("c_index")||name.starts_with("c_middle")||name.starts_with("c_ring")||name.starts_with("c_pinky")||name.starts_with("c_thumb")){combined[n]=fingerPose[n];++maskNodes;}
                    else if(name=="hair.d.000_093"||name=="hair.h.000_0108"||name=="c_tail_00.x_0178"||name=="cloth.c.000_0182"){combined[n]=secondaryPose[n];++maskNodes;}
                }
                std::ofstream mix(out/"gpu_mix.csv");mix<<"pose,fraction,view,max_palette_delta_vs_bind,changed_rgb_bytes_vs_bind,image\n";
                const auto saveMix=[&](const std::string& label,double fraction,const std::vector<glm::mat4>& locals,unsigned views){for(const auto& local:locals)if(!finite(local))throw std::runtime_error("Nonfinite blended local pose");const auto palette=model.skinningMatrices(locals);for(const auto& matrix:palette)if(!finite(matrix))throw std::runtime_error("Nonfinite blended palette");for(unsigned v=0;v<views;++v){const auto pixels=render(v,palette);const std::string file="gpu_mix_"+label+"_"+viewNames[v]+".ppm";save(out/file,pixels);mix<<label<<','<<fraction<<','<<viewNames[v]<<','<<matrixDelta(palette,bind)<<','<<changed(reference[v],pixels)<<','<<file<<'\n';++rendered;}};
                saveMix("combined_masks",.5,combined,4);
                for(const float fraction:{0.f,.25f,.5f,.75f,1.f}){std::vector<glm::mat4> blended;for(unsigned n=0;n<model.nodes().size();++n)blended.push_back(blendTRS(elbowPose[n],kneePose[n],fraction));saveMix("crossfade_"+std::to_string(static_cast<int>(fraction*100)),fraction,blended,2);}
                report<<"combined_pose=elbows_head_torso_fingers_hair_tail_cloth\ncombined_masked_nodes="<<maskNodes<<"\ncrossfade=elbow_peak_to_knee_peak_local_TRS_lerp_quaternion_slerp\ncrossfade_fractions=0,0.25,0.5,0.75,1\nblend_tests=finite_and_rendered_evaluation_only\n";
            }
            for(unsigned v=0;v<4;++v)if(changed(reference[v],render(v,bind)))throw std::runtime_error("Bind reset nondeterministic");checkGl("final");report<<"rendered_images="<<rendered<<"\nclips_with_pixel_changes="<<changedClipCount<<"\ngl_errors=0\nbind_reset=EXACT\nresult=PASS\nlimitations=Pixel changes prove submitted animation affects rendering; they do not establish natural joints, collision freedom or morph support.\n";
            std::cout<<"GPU audit PASS: "<<scene->mNumAnimations<<" clips, "<<rendered<<" snapshots, zero GL errors\n";
        }
        glfwDestroyWindow(window);glfwTerminate();return 0;
    }catch(const std::exception& e){std::cerr<<"GPU audit failed: "<<e.what()<<'\n';if(window)glfwDestroyWindow(window);glfwTerminate();return 1;}
}
