#include "julia/embodiment/BodyProfile.hpp"
#include "julia/embodiment/Animation.hpp"
#include "julia/embodiment/EyePoseLayer.hpp"
#include "julia/model_import/ModelImporter.hpp"
#include "julia_test.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <cmath>

using namespace julia;
using namespace julia::embodiment;
namespace
{
    bool close(const glm::mat4& a,const glm::mat4& b,float eps=3e-5f)
    {
        for (int c=0;c<4;++c) for (int r=0;r<4;++r)
            if (!std::isfinite(a[c][r]) || std::abs(a[c][r]-b[c][r])>eps) return false;
        return true;
    }
    template<class F> bool rejects(F&& f) { try { f(); } catch (const std::exception&) { return true; } return false; }
    bool has(const std::vector<BindingIssue>& issues,BindingError code)
    { return std::any_of(issues.begin(),issues.end(),[&](const auto& i){return i.code==code;}); }
    std::uint32_t node(const model::ModelAsset& a,const std::string& name)
    {
        for (std::uint32_t i=0;i<a.nodes().size();++i) if (a.nodes()[i].name==name) return i;
        throw std::runtime_error("Test landmark missing: " + name);
    }
    glm::vec3 position(const BodyRigBinding& b,JointId id,const AnimationPlayer& p)
    { return glm::vec3{b.jointFrame(id,p.globalTransforms())[3]}; }
    void audit(const model::ModelAsset& a,std::ostream& out)
    {
        out << std::setprecision(10) << "fingerprint=" << bodyRigFingerprint(a) << "\nnodes=" << a.nodes().size()
            << "\nbindings=" << a.bones().size() << "\nclips=" << a.animations().size() << '\n';
        std::size_t reflected=0,nonuniform=0,sheared=0;
        for (std::size_t i=0;i<a.nodes().size();++i)
        {
            const auto& n=a.nodes()[i]; const glm::mat3 l{n.localTransform};
            glm::mat3 basis=l;
            const glm::vec3 scale{glm::length(l[0]),glm::length(l[1]),glm::length(l[2])};
            reflected+=glm::determinant(l)<0;
            nonuniform+=std::max({scale.x,scale.y,scale.z})-std::min({scale.x,scale.y,scale.z})>1e-5f;
            for (int c=0;c<3;++c) basis[c]/=scale[c];
            const bool shear=std::abs(glm::dot(basis[0],basis[1]))>1e-5f ||
                std::abs(glm::dot(basis[0],basis[2]))>1e-5f || std::abs(glm::dot(basis[1],basis[2]))>1e-5f;
            sheared+=shear;
            out << i << ' ' << n.parent << ' ' << n.name << " det=" << glm::determinant(l)
                << " scale=" << scale.x << ',' << scale.y << ',' << scale.z << " shear=" << shear
                << " global=" << n.globalBindTransform[3].x << ',' << n.globalBindTransform[3].y << ',' << n.globalBindTransform[3].z << '\n';
        }
        out << "reflected_local=" << reflected << " nonuniform_local=" << nonuniform << " sheared_local=" << sheared << '\n';
        for (const auto* name:{"foot.l_04","toes_01.l_05","foot.r_010","toes_01.r_011"})
        {
            const auto n=node(a,name);
            glm::vec3 lo{1e9f},hi{-1e9f},bottomSum{0}; std::size_t count=0,bottomCount=0;
            for (const auto& m:a.meshes()) for (const auto& v:m.vertices)
            {
                float weight=0;
                for (int k=0;k<4;++k) if (v.boneIds[k]>=0 && a.bones()[static_cast<std::size_t>(v.boneIds[k])].nodeIndex==n) weight+=v.boneWeights[k];
                if (weight>.5f) { lo=glm::min(lo,v.position); hi=glm::max(hi,v.position); ++count; }
            }
            for (const auto& m:a.meshes()) for (const auto& v:m.vertices)
            {
                float weight=0;
                for (int k=0;k<4;++k) if (v.boneIds[k]>=0 && a.bones()[static_cast<std::size_t>(v.boneIds[k])].nodeIndex==n) weight+=v.boneWeights[k];
                if (weight>.5f && v.position.y<lo.y+.015f) { bottomSum+=v.position; ++bottomCount; }
            }
            out << name << " vertices=" << count << " bounds=" << lo.x << ',' << lo.y << ',' << lo.z << '/' << hi.x << ',' << hi.y << ',' << hi.z;
            if (bottomCount) { const auto b=bottomSum/static_cast<float>(bottomCount); out << " bottom_centroid=" << b.x << ',' << b.y << ',' << b.z; }
            out << '\n';
        }
    }
    void affineFixture()
    {
        model::ModelAsset asset;
        auto root=glm::scale(glm::mat4{1},glm::vec3{-2,3,4}); root[1][0]=.7f;
        asset.addNode({"arbitrary root",animation::NoNode,root});
        const auto local=glm::translate(glm::mat4{1},glm::vec3{1,2,3});
        asset.addNode({"arbitrary hinge",0,local});
        asset.addNode({"tip",1,glm::translate(glm::mat4{1},glm::vec3{0,2,0})});
        asset.addBone({"palette entry",glm::inverse(asset.nodes()[2].globalBindTransform),2});
        BodyProfile p; p.name="affine fixture"; p.rigFingerprint=bodyRigFingerprint(asset);
        const auto pivot=glm::vec3{asset.nodes()[1].globalBindTransform[3]};
        p.joints.push_back({JointId::LeftElbow,SegmentId::LeftForearm,"hinge",{"arbitrary hinge","arbitrary root"},
            glm::translate(glm::mat4{1},pivot),{{"q","positive Z",{0,0,1},-1,1}},{"tip","arbitrary hinge"}});
        BodyRigBinding b(p,asset); DirectBodyControl control(b); control.setEnabled(true);
        animation::Pose pose(asset.skeleton()); const auto bind=pose.locals;
        control.apply(pose.locals); JULIA_CHECK(pose.locals==bind);
        control.setCoordinate(JointId::LeftElbow,0,.5f);
        control.apply(pose.locals); animation::computeGlobalTransforms(asset.skeleton(),pose.locals,pose.globals);
        const auto deformation=pose.globals[1]*glm::inverse(asset.nodes()[1].globalBindTransform);
        const auto expected=glm::translate(glm::mat4{1},pivot)*glm::rotate(glm::mat4{1},.5f,glm::vec3{0,0,1})*glm::translate(glm::mat4{1},-pivot);
        JULIA_CHECK(close(deformation,expected));
        JULIA_CHECK(std::abs(glm::determinant(glm::mat3{deformation})-1)<1e-5f);
        JULIA_CHECK(pose.locals[0]==root);
        JULIA_CHECK(std::abs(glm::distance(glm::vec3{pose.globals[2][3]},glm::vec3{pose.globals[1][3]})-glm::distance(glm::vec3{asset.nodes()[2].globalBindTransform[3]},pivot))<1e-4f);
        auto ambiguous=asset; ambiguous.addNode({"arbitrary hinge",0});
        JULIA_CHECK(has(validateBodyProfile(p,ambiguous),BindingError::AmbiguousNode));
    }
}
int main(int argc,char** argv)
{
    try
    {
        const auto output=argc>1 ? std::filesystem::path{argv[1]}:std::filesystem::temp_directory_path()/"Julia/stage3";
        std::filesystem::create_directories(output);
        const auto asset=model_import::ModelImporter::loadFromFile(std::filesystem::path{JULIA_TEST_ASSET_DIR}/"models/Raphtalia_prepared.fbx");
        std::ofstream rig(output/"cpu-rig.txt"); audit(*asset,rig);
        std::cout << "Calibrated CPU rig fingerprint=" << bodyRigFingerprint(*asset) << '\n';
        if (argc>2 && std::string_view{argv[2]}=="--audit") return 0;
        auto profile=raphtaliaBodyProfile();
        const auto issues=validateBodyProfile(profile,*asset);
        for (const auto& i:issues) std::cerr << i.message << '\n';
        JULIA_CHECK(issues.empty()); if (!issues.empty()) return JULIA_TEST_RESULT();
        BodyRigBinding binding(profile,*asset); DirectBodyControl control(binding),secondControl(binding);
        AnimationPlayer player(*asset),second(*asset); player.setBodyControl(&control); second.setBodyControl(&secondControl);
        const auto bindLocals=std::vector<glm::mat4>{player.localTransforms().begin(),player.localTransforms().end()};
        const auto bindPalette=std::vector<glm::mat4>{player.palette().begin(),player.palette().end()};
        control.setEnabled(true); player.update(0);
        JULIA_CHECK(std::equal(bindLocals.begin(),bindLocals.end(),player.localTransforms().begin()));
        JULIA_CHECK(std::equal(bindPalette.begin(),bindPalette.end(),player.palette().begin()));
        JULIA_CHECK(profile.joints.size()==20 && profile.effectors.size()==8 && profile.followers.size()==8);
        std::ofstream points(output/"checkpoints.csv"); points << "name,x_m,y_m,z_m,det\n" << std::setprecision(9);
        for (const auto& e:profile.effectors)
        {
            const auto f=binding.effectorFrame(e.id,player.globalTransforms());
            JULIA_CHECK(close(f,e.frameInBodyBind));
            JULIA_CHECK(std::abs(glm::determinant(glm::mat3{f})-1)<1e-4f);
            points << e.name << ',' << f[3].x << ',' << f[3].y << ',' << f[3].z << ',' << glm::determinant(glm::mat3{f}) << '\n';
        }
        // Independent resource landmarks, measured before adding the profile.
        JULIA_CHECK(glm::distance(glm::vec3{binding.effectorFrame(EffectorId::LeftWrist,player.globalTransforms())[3]},glm::vec3{.1993272f,.8417444f,.1019266f})<2e-5f);
        JULIA_CHECK(glm::distance(glm::vec3{binding.effectorFrame(EffectorId::RightWrist,player.globalTransforms())[3]},glm::vec3{-.2788304f,.8676412f,-.0648794f})<2e-5f);
        JULIA_CHECK(glm::distance(glm::vec3{binding.effectorFrame(EffectorId::LeftPalm,player.globalTransforms())[2]},glm::vec3{.8044152f,.5377885f,.2523881f})<2e-5f);
        JULIA_CHECK(glm::distance(glm::vec3{binding.effectorFrame(EffectorId::RightPalm,player.globalTransforms())[2]},glm::vec3{-.7352896f,.4246825f,.5281988f})<2e-5f);
        JULIA_CHECK(glm::distance(glm::vec3{binding.effectorFrame(EffectorId::LeftSole,player.globalTransforms())[2]},glm::vec3{.3924938f,0,.9197546f})<2e-5f);
        JULIA_CHECK(glm::distance(glm::vec3{binding.effectorFrame(EffectorId::RightSole,player.globalTransforms())[2]},glm::vec3{-.1621108f,0,.9867726f})<2e-5f);
        for (const auto side:{0,1})
        {
            const auto elbow=side==0 ? JointId::LeftElbow:JointId::RightElbow;
            const auto wrist=side==0 ? JointId::LeftWrist:JointId::RightWrist;
            const auto base=position(binding,wrist,player);
            control.setCoordinate(elbow,0,glm::radians(20.0f)); player.update(0);
            JULIA_CHECK(position(binding,wrist,player).z>base.z+.06f);
            JULIA_CHECK(std::abs(glm::distance(position(binding,elbow,player),position(binding,wrist,player))-
                glm::distance(glm::vec3{profile.joints[binding.jointIndex(elbow)].frameInBodyBind[3]},base))<2e-5f);
            control.reset(); player.update(0);
            const auto knee=side==0 ? JointId::LeftKnee:JointId::RightKnee;
            const auto ankle=side==0 ? JointId::LeftAnkle:JointId::RightAnkle;
            const auto foot=position(binding,ankle,player);
            control.setCoordinate(knee,0,glm::radians(20.0f)); player.update(0);
            JULIA_CHECK(position(binding,ankle,player).z<foot.z-.06f);
            control.reset(); player.update(0);
        }
        // Check named positive directions independently of the configured axes.
        std::ofstream signs(output/"semantic-signs.csv"); signs << "joint,coordinate,positive_displacement_m\n";
        for (const auto& j:profile.joints) for (std::size_t k=0;k<j.coordinates.size();++k)
        {
            control.reset(); control.setCoordinate(j.id,k,glm::radians(5.0f)); player.update(0);
            const auto n=binding.jointNode(j.id),end=node(*asset,j.segmentEnd.name);
            const auto pivot=glm::vec3{j.frameInBodyBind[3]};
            auto marker=glm::vec3{profile.bodyFromResource*asset->nodes()[end].globalBindTransform[3]};
            glm::vec3 direction{0,0,1};
            const bool right=j.id==JointId::RightClavicle || j.id==JointId::RightShoulder || j.id==JointId::RightElbow ||
                j.id==JointId::RightWrist || j.id==JointId::RightHip || j.id==JointId::RightKnee || j.id==JointId::RightAnkle;
            const float side=right ? -1.0f:1.0f;
            if (j.id<=JointId::Head)
            {
                marker=pivot+(k==2 ? glm::vec3{0,0,.1f}:glm::vec3{0,.1f,0});
                direction=k==0 ? glm::vec3{0,0,1}:glm::vec3{1,0,0};
            }
            else if (j.id==JointId::LeftClavicle || j.id==JointId::RightClavicle) direction={0,1,0};
            else if (j.id==JointId::LeftKnee || j.id==JointId::RightKnee) direction={0,0,-1};
            else if (j.id==JointId::LeftWrist || j.id==JointId::RightWrist)
            {
                if (k==0) direction=glm::vec3{j.frameInBodyBind[2]};
                else
                {
                    const auto index=node(*asset,right ? "c_index1.r_0152":"c_index1.l_0173");
                    const auto pinky=node(*asset,right ? "c_pinky1.r_0143":"c_pinky1.l_0164");
                    direction=glm::normalize(glm::vec3{asset->nodes()[index].globalBindTransform[3]-asset->nodes()[pinky].globalBindTransform[3]});
                }
            }
            else if (j.id==JointId::LeftAnkle || j.id==JointId::RightAnkle)
            {
                direction={0,1,0};
                if (k==1) marker=pivot-side*glm::vec3{j.frameInBodyBind[0]}*.1f;
            }
            else if (k==1) direction={side,0,0}; // Shoulder/hip abduction.
            else if (k==2)
            {
                const auto shaft=glm::normalize(marker-pivot);
                marker=pivot+glm::normalize(glm::cross(shaft,glm::vec3{0,0,1}))*.1f;
                direction=glm::normalize(glm::cross(shaft,marker-pivot));
            }
            const auto d=profile.bodyFromResource*player.globalTransforms()[n]*glm::inverse(asset->nodes()[n].globalBindTransform)*glm::inverse(profile.bodyFromResource);
            const auto delta=glm::vec3{d*glm::vec4{marker,1}}-marker;
            const auto signedDisplacement=glm::dot(delta,direction);
            JULIA_CHECK(signedDisplacement>1e-4f);
            signs << j.name << ',' << j.coordinates[k].name << ',' << signedDisplacement << '\n';
        }
        std::size_t samples=0,dofs=0;
        std::ofstream sweep(output/"coordinate-sweep.csv"); sweep << "joint,coordinate,degrees,max_length_error_m\n";
        for (const auto& j:profile.joints) for (std::size_t k=0;k<j.coordinates.size();++k)
        {
            ++dofs; const auto& c=j.coordinates[k];
            for (const auto angle:{c.lower,c.lower+1e-5f,.01f,c.upper-1e-5f,c.upper})
            {
                control.reset(); control.setCoordinate(j.id,k,angle); player.update(0); ++samples;
                const auto& g=player.globalTransforms();
                float maxError=0;
                for (const auto& segment:profile.joints)
                {
                    const auto a=binding.jointNode(segment.id),b=node(*asset,segment.segmentEnd.name);
                    const auto v=glm::vec3{profile.bodyFromResource*(g[b][3]-g[a][3])};
                    const auto v0=glm::vec3{profile.bodyFromResource*(asset->nodes()[b].globalBindTransform[3]-asset->nodes()[a].globalBindTransform[3])};
                    maxError=std::max(maxError,std::abs(glm::length(v)-glm::length(v0)));
                }
                JULIA_CHECK(maxError<3e-5f);
                const auto n=binding.jointNode(j.id);
                const auto d=profile.bodyFromResource*g[n]*glm::inverse(asset->nodes()[n].globalBindTransform)*glm::inverse(profile.bodyFromResource);
                JULIA_CHECK(std::abs(glm::determinant(glm::mat3{d})-1)<3e-4f);
                // Independent Rodrigues formula at the calibrated joint frame.
                const glm::vec3 r{.037f,.059f,.083f},axis=c.axis;
                const auto expected=r*std::cos(angle)+glm::cross(axis,r)*std::sin(angle)+axis*glm::dot(axis,r)*(1-std::cos(angle));
                const auto observed=glm::inverse(j.frameInBodyBind)*d*j.frameInBodyBind*glm::vec4{r,1};
                JULIA_CHECK(glm::distance(glm::vec3{observed},expected)<3e-5f);
                for (const auto& follower:profile.followers)
                {
                    if (follower.joint!=j.id) continue;
                    const auto f=node(*asset,follower.node.name);
                    const auto fd=profile.bodyFromResource*g[f]*glm::inverse(asset->nodes()[f].globalBindTransform)*glm::inverse(profile.bodyFromResource);
                    const float fractionAngle=angle*follower.fraction[static_cast<int>(k)];
                    const auto wanted=r*std::cos(fractionAngle)+glm::cross(axis,r)*std::sin(fractionAngle)+axis*glm::dot(axis,r)*(1-std::cos(fractionAngle));
                    const auto actual=glm::inverse(j.frameInBodyBind)*fd*j.frameInBodyBind*glm::vec4{r,1};
                    JULIA_CHECK(glm::distance(glm::vec3{actual},wanted)<3e-5f);
                }
                sweep << j.name << ',' << c.name << ',' << glm::degrees(angle) << ',' << maxError << '\n';
                const auto world=glm::translate(glm::mat4{1},glm::vec3{3,2,-1})*glm::rotate(glm::mat4{1},.7f,glm::vec3{0,1,0});
                const auto inWorld=binding.worldFromResource(world)*g[n]*glm::vec4{0,0,0,1};
                JULIA_CHECK(glm::distance(glm::vec3{inWorld},glm::vec3{world*binding.jointFrame(j.id,g)[3]})<3e-5f);
            }
            const auto previous=control.coordinate(j.id,k);
            JULIA_CHECK(control.setCoordinate(j.id,k,c.upper+.01f).status==CommandStatus::Rejected);
            JULIA_CHECK(control.coordinate(j.id,k)==previous);
            JULIA_CHECK(control.setCoordinate(j.id,k,c.upper+.01f,LimitPolicy::Clamp).status==CommandStatus::Clamped);
            JULIA_CHECK(control.coordinate(j.id,k)==c.upper);
            JULIA_CHECK(control.setCoordinate(j.id,k,c.lower-.01f,LimitPolicy::Clamp).applied==c.lower);
        }
        // Simultaneous coordinates and serial joints must still act as rigid chains.
        std::uint32_t random=0x52415048;
        for (int sample=0;sample<128;++sample)
        {
            for (const auto& j:profile.joints) for (std::size_t k=0;k<j.coordinates.size();++k)
            {
                random=random*1664525u+1013904223u;
                const float t=static_cast<float>(random>>8)/16777216.0f;
                const auto& c=j.coordinates[k]; control.setCoordinate(j.id,k,c.lower+(c.upper-c.lower)*t);
            }
            player.update(0);
            for (const auto& j:profile.joints)
            {
                const auto a=binding.jointNode(j.id),end=node(*asset,j.segmentEnd.name);
                const float l=glm::distance(glm::vec3{player.globalTransforms()[a][3]},glm::vec3{player.globalTransforms()[end][3]});
                const float l0=glm::distance(glm::vec3{asset->nodes()[a].globalBindTransform[3]},glm::vec3{asset->nodes()[end].globalBindTransform[3]});
                JULIA_CHECK(std::abs(l-l0)*.4f<3e-5f);
                const auto f=binding.jointFrame(j.id,player.globalTransforms());
                JULIA_CHECK(std::abs(glm::determinant(glm::mat3{f})-1)<3e-4f);
            }
        }
        control.reset(); control.setCoordinate(JointId::LeftElbow,0,.4f); player.update(0);
        JULIA_CHECK(std::equal(bindLocals.begin(),bindLocals.end(),second.localTransforms().begin()));
        JULIA_CHECK(secondControl.coordinate(JointId::LeftElbow,0)==0);
        JULIA_CHECK(rejects([&]{control.setCoordinate(JointId::LeftElbow,0,std::numeric_limits<float>::quiet_NaN());}));
        JULIA_CHECK(rejects([&]{control.setCoordinate(JointId::LeftElbow,1,0);}));
        JULIA_CHECK(rejects([&]{control.coordinate(static_cast<JointId>(999),0);}));
        JULIA_CHECK(rejects([&]{control.apply({});}));
        JULIA_CHECK(rejects([&]{binding.effectorFrame(EffectorId::LeftSole,{});}));
        JULIA_CHECK(rejects([&]{binding.worldFromResource(glm::scale(glm::mat4{1},glm::vec3{2}));}));
        auto bad=profile; bad.joints[0].node.name="missing";
        JULIA_CHECK(has(validateBodyProfile(bad,*asset),BindingError::MissingNode));
        bad=profile; bad.joints[0].node.parentName="wrong";
        JULIA_CHECK(has(validateBodyProfile(bad,*asset),BindingError::WrongParent));
        bad=profile; bad.joints[0].coordinates[0].axis={0,0,0};
        JULIA_CHECK(has(validateBodyProfile(bad,*asset),BindingError::InvalidProfile));
        bad=profile; bad.rigFingerprint^=1;
        JULIA_CHECK(has(validateBodyProfile(bad,*asset),BindingError::IncompatibleRig));
        JULIA_CHECK(rejects([&]{BodyRigBinding broken(bad,*asset);}));
        auto other=*asset; other.addNode({"new helper"});
        BodyProfile otherProfile=profile; otherProfile.rigFingerprint=bodyRigFingerprint(other);
        BodyRigBinding otherBinding(otherProfile,other); DirectBodyControl otherControl(otherBinding);
        JULIA_CHECK(rejects([&]{player.setBodyControl(&otherControl);}));
        // All authored clips restore exactly after disabling diagnostic control.
        for (std::size_t i=0;i<asset->animations().size();++i)
        {
            control.setEnabled(false); player.selectClip(i); second.selectClip(i);
            player.seek(asset->animations()[i].durationSeconds()*.37); second.seek(player.timeSeconds());
            JULIA_CHECK(std::equal(player.palette().begin(),player.palette().end(),second.palette().begin()));
            control.setEnabled(true); player.update(0);
            JULIA_CHECK(player.localTransforms()[binding.jointNode(JointId::LeftElbow)]!=second.localTransforms()[binding.jointNode(JointId::LeftElbow)]);
        }
        EyePoseLayer eyes(*asset),secondEyes(*asset);
        eyes.behavior().setAutoBlink(false); eyes.behavior().setAutoGaze(false);
        secondEyes.behavior().setAutoBlink(false); secondEyes.behavior().setAutoGaze(false);
        eyes.behavior().setGazeDegrees({5,3}); secondEyes.behavior().setGazeDegrees({5,3});
        control.reset(); control.setEnabled(true); secondControl.setEnabled(true);
        player.setEyeLayer(&eyes); second.setEyeLayer(&secondEyes); second.stop(); player.update(0); second.update(0);
        JULIA_CHECK(std::equal(player.palette().begin(),player.palette().end(),second.palette().begin()));
        player.play(); // Authored eye clip is hidden by direct control; procedural eyes still advance.
        eyes.behavior().triggerBlink(); secondEyes.behavior().triggerBlink(); player.update(.1); second.update(.1);
        JULIA_CHECK(std::equal(player.palette().begin(),player.palette().end(),second.palette().begin()));
        affineFixture();
        std::cout << "CPU body: " << profile.joints.size() << " joints, " << dofs << " DoF, " << samples
            << " boundary/axis poses; exact zero, signs, lengths, spaces, errors, instances, clips, eyes and affine fixture checked\n";
        return JULIA_TEST_RESULT();
    }
    catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
