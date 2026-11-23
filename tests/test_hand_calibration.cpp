// Bare-hand calibration of the prepared Raphtalia rig: anatomical axes and signs, the
// relaxed Idle pose, empty-hand opening and its return, thumb opposition, isolated joint
// bends and the skin between fingers and palm, measured on the visible desktop skin.
#include "julia/embodiment/BodyInstance.hpp"
#include "julia/model_import/ModelImporter.hpp"
#include "hand_pose_audit.hpp"
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>

namespace
{
    using namespace julia;
    using namespace julia::embodiment;
    constexpr std::array<const char*,5> fingers{"index","middle","ring","pinky","thumb"};

    void require(bool condition,const std::string& reason)
    { if (!condition) throw std::runtime_error(reason); }

    void command(BodyInstance& b,BodyOperation operation,BodyCommandStatus expected=BodyCommandStatus::Applied)
    {
        const auto id=b.enqueue(std::move(operation)).id;
        b.step();
        require(!b.results().empty() && b.results().front().id==id && b.results().front().status==expected,
            "Unexpected command outcome: "+(b.results().empty() ? std::string{}:b.results().front().reason));
    }

    struct Hand { std::array<std::array<std::uint32_t,3>,5> node{}; };
    Hand chain(const HandLayer& layer,unsigned side)
    {
        Hand h;
        for (const auto& j:layer.joints()) if (j.side==side) h.node[j.finger][j.phalanx]=j.node;
        return h;
    }

    void refresh(const model::ModelAsset& asset,animation::Pose& pose)
    {
        animation::computeGlobalTransforms(asset.skeleton(),pose.locals,pose.globals);
        animation::buildSkinningPalette(asset.skeleton(),pose.globals,pose.palette);
    }

    // Anatomical angles measured independently from posed node positions and bone axes.
    std::array<std::array<float,4>,4> measuredFingers(const BodyRigBinding& b,const HandLayer& layer,
        std::span<const glm::mat4> globals,unsigned side)
    {
        const auto h=chain(layer,side);
        const auto frame=b.jointFrame(side==0 ? JointId::LeftWrist:JointId::RightWrist,globals);
        const auto distal=glm::normalize(glm::vec3{frame[1]}), palmar=-glm::normalize(glm::vec3{frame[2]});
        const auto radial=(side==0 ? 1.f:-1.f)*glm::normalize(glm::vec3{frame[0]});
        const auto& B=b.profile().bodyFromResource;
        const auto p=[&](std::uint32_t n) { return glm::vec3{B*globals[n][3]}; };
        std::array<std::array<float,4>,4> result{};
        for (unsigned f=0;f<4;++f) {
            const auto v1=glm::normalize(p(h.node[f][1])-p(h.node[f][0])), v2=glm::normalize(p(h.node[f][2])-p(h.node[f][1]));
            const auto v3=glm::normalize(glm::vec3{B*globals[h.node[f][2]][1]});
            const auto axis=glm::normalize(glm::cross(v1,palmar));
            const auto angle=[&](glm::vec3 u,glm::vec3 v) {
                u=glm::normalize(u-glm::dot(u,axis)*axis); v=glm::normalize(v-glm::dot(v,axis)*axis);
                return std::atan2(glm::dot(glm::cross(u,v),axis),glm::dot(u,v)); };
            result[f]={std::asin(std::clamp(glm::dot(v1,palmar),-1.f,1.f)),std::atan2(glm::dot(v1,radial),glm::dot(v1,distal)),angle(v1,v2),angle(v2,v3)};
        }
        return result;
    }

    void anatomy(const model::ModelAsset& asset,const BodyRigBinding& b,const HandLayer& layer,std::ostream& report)
    {
        animation::Pose pose(asset.skeleton());
        const auto& B=b.profile().bodyFromResource;
        for (unsigned side=0;side<2;++side) {
            const auto& a=layer.anatomy(side);
            const auto h=chain(layer,side);
            // Palmar is -Z of the profile wrist frame: the modelled curl of every finger points there.
            // Only clear curls define a plane (the left hand, PIP 51-85 degrees, within 1.5 degrees).
            unsigned curls=0;
            for (unsigned f=0;f<4;++f) {
                require(std::abs(glm::dot(a.flexion[f],a.palmar))<1e-4f && std::abs(glm::dot(a.flexion[f],a.distal))<.25f,
                    std::string{"Flexion axis leaves the palm plane: "}+fingers[f]);
                const auto pip=glm::vec3{B*asset.nodes()[h.node[f][1]].globalBindTransform[3]};
                const auto dip=glm::vec3{B*asset.nodes()[h.node[f][2]].globalBindTransform[3]};
                const auto mcp=glm::vec3{B*asset.nodes()[h.node[f][0]].globalBindTransform[3]};
                const auto curl=glm::cross(pip-mcp,dip-pip);
                if (glm::length(curl)>.6f*glm::length(pip-mcp)*glm::length(dip-pip)) {
                    require(glm::degrees(std::acos(std::clamp(glm::dot(glm::normalize(curl),a.flexion[f]),-1.f,1.f)))<3,
                        std::string{"Modelled curl plane disagrees with the flexion axis: "}+fingers[f]);
                    ++curls;
                }
            }
            require(side==1 || curls==4,"Left-hand bind curls no longer define the flexion planes");
            // Every joint moves towards its calibrated side.
            const auto moved=[&](const char* motion,unsigned finger,unsigned phalanx,const std::function<void(HandPose&)>& change,glm::vec3 towards,float minimum=.5f) {
                const auto open=openHandPose();
                auto bent=open; change(bent);
                glm::vec3 tips[2];
                for (int k=0;k<2;++k) {
                    animation::setBindPose(asset.skeleton(),pose.locals);
                    layer.applyPose(pose.locals,side,k ? bent:open);
                    animation::computeGlobalTransforms(asset.skeleton(),pose.locals,pose.globals);
                    const auto end=B*pose.globals[h.node[finger][2]];
                    tips[k]=glm::vec3{end[3]}+glm::normalize(glm::vec3{end[1]})*.012f;
                }
                const auto delta=tips[1]-tips[0];
                const float along=glm::length(delta)>0 ? glm::dot(glm::normalize(delta),towards):0;
                report << (side ? "right_":"left_") << fingers[finger] << phalanx+1 << '_' << motion << "_tip_mm=" << glm::length(delta)*1000 << " along_expected=" << along << '\n';
                require(glm::length(delta)>.003f && along>minimum,
                    std::string{"Wrong direction: "}+fingers[finger]+" joint "+std::to_string(phalanx+1)+" "+motion+(side ? " right":" left")+" cos="+std::to_string(along));
                return delta;
            };
            const float r=glm::radians(30.f);
            for (unsigned f=0;f<4;++f) {
                moved("flexion",f,0,[&](HandPose& p) { p.fingers[f].mcpFlexion+=r; },a.palmar);
                moved("flexion",f,1,[&](HandPose& p) { p.fingers[f].pipFlexion+=r; },a.palmar);
                moved("flexion",f,2,[&](HandPose& p) { p.fingers[f].dipFlexion+=r; },a.palmar);
                moved("spread",f,0,[&](HandPose& p) { p.fingers[f].mcpSpread+=glm::radians(10.f); },a.radial);
            }
            moved("palmar_abduction",4,0,[&](HandPose& p) { p.thumb.palmarAbduction+=r; },a.palmar);
            // Already abducted: the tip travels tangentially in the palm plane, away from the index finger.
            const auto abducted=moved("radial_abduction",4,0,[&](HandPose& p) { p.thumb.radialAbduction+=r; },a.radial,.35f);
            require(std::abs(glm::dot(glm::normalize(abducted),a.palmar))<.25f,"Thumb radial abduction leaves the palm plane");
            const auto pad=glm::normalize(-.70f*a.radial+.71f*a.palmar);
            moved("flexion",4,1,[&](HandPose& p) { p.thumb.mcpFlexion+=r; },glm::normalize(pad+a.palmar));
            moved("flexion",4,2,[&](HandPose& p) { p.thumb.ipFlexion+=r; },pad);
            // Pronation turns the nail (radial-dorsal in the model) towards the palmar side.
            {
                animation::setBindPose(asset.skeleton(),pose.locals);
                auto p=openHandPose(); p.thumb.pronation+=glm::radians(40.f);
                layer.applyPose(pose.locals,side,p);
                animation::computeGlobalTransforms(asset.skeleton(),pose.locals,pose.globals);
                const auto nail=glm::normalize(glm::mat3{B*pose.globals[h.node[4][2]]}*glm::inverse(glm::mat3{B*asset.nodes()[h.node[4][2]].globalBindTransform})*
                    glm::normalize(.85f*a.radial-.49f*a.palmar));
                animation::setBindPose(asset.skeleton(),pose.locals);
                layer.applyPose(pose.locals,side,openHandPose());
                animation::computeGlobalTransforms(asset.skeleton(),pose.locals,pose.globals);
                const auto before=glm::normalize(glm::mat3{B*pose.globals[h.node[4][2]]}*glm::inverse(glm::mat3{B*asset.nodes()[h.node[4][2]].globalBindTransform})*
                    glm::normalize(.85f*a.radial-.49f*a.palmar));
                require(glm::dot(nail,a.palmar)>glm::dot(before,a.palmar)+.3f,"Thumb pronation does not turn the nail palmar");
            }
            report << (side ? "right":"left") << "_bind_deg";
            for (unsigned f=0;f<4;++f) report << ' ' << fingers[f] << '(' << glm::degrees(a.bind.fingers[f].mcpFlexion) << ','
                << glm::degrees(a.bind.fingers[f].mcpSpread) << ',' << glm::degrees(a.bind.fingers[f].pipFlexion) << ',' << glm::degrees(a.bind.fingers[f].dipFlexion) << ')';
            report << " thumb_mcp_ip(" << glm::degrees(a.bind.thumb.mcpFlexion) << ',' << glm::degrees(a.bind.thumb.ipFlexion) << ")\n";
        }
        // Mirror symmetry of the modelled thumbs and exact reproduction of the targets on both hands.
        require(std::abs(layer.anatomy(0).bind.thumb.mcpFlexion-layer.anatomy(1).bind.thumb.mcpFlexion)<glm::radians(1.f) &&
            std::abs(layer.anatomy(0).bind.thumb.ipFlexion-layer.anatomy(1).bind.thumb.ipFlexion)<glm::radians(1.f),"Thumbs are not mirror symmetric");
        for (const auto& [name,target]:{std::pair{"relaxed",relaxedHandPose()},{"open",openHandPose()},{"opposition",oppositionHandPose()}}) {
            animation::setBindPose(asset.skeleton(),pose.locals);
            for (unsigned side=0;side<2;++side) layer.applyPose(pose.locals,side,target);
            animation::computeGlobalTransforms(asset.skeleton(),pose.locals,pose.globals);
            for (unsigned side=0;side<2;++side) {
                const auto m=measuredFingers(b,layer,pose.globals,side);
                for (unsigned f=0;f<4;++f) {
                    const auto& t=target.fingers[f];
                    const std::array<float,4> want{t.mcpFlexion,t.mcpSpread,t.pipFlexion,t.dipFlexion};
                    for (int k=0;k<4;++k) require(std::abs(m[f][static_cast<std::size_t>(k)]-want[static_cast<std::size_t>(k)])<glm::radians(.1f),
                        std::string{name}+" pose not reproduced: "+fingers[f]);
                }
            }
        }
        report << "anatomy_axes_signs_mirror_reproduction=PASS\n";
    }

    void requireClean(hand_pose_audit::Skin& skin,std::span<const glm::mat4> palette,const std::string& what,std::ostream* report=nullptr,
        float minArea=0,bool clearance=false)
    {
        skin.pose(palette);
        auto m=skin.measure();
        if (clearance) skin.clearance(m);
        for (unsigned side=0;side<2;++side) {
            require(m[side].interRegion()==0 && m[side].interRegionStrict()==0 && m[side].fingerFolds==0,
                what+": hand skin intersects ("+(side ? "right":"left")+")");
            require(m[side].minAreaRatio>=minArea,what+": local skin compression ("+(side ? "right":"left")+")");
            if (report) {
                *report << what << (side ? "_right":"_left") << " inter_region=" << m[side].interRegion() << " folds=" << m[side].fingerFolds
                    << " min_area_ratio=" << m[side].minAreaRatio;
                if (clearance) {
                    *report << " clearance_mm(i-m,m-r,r-p,t-i)=";
                    for (float c:m[side].neighbourClearance) *report << ' ' << c*1000;
                }
                *report << '\n';
            }
        }
    }

    void staticPoses(const model::ModelAsset& asset,const HandLayer& layer,hand_pose_audit::Skin& skin,std::ostream& report)
    {
        animation::Pose pose(asset.skeleton());
        // Local compression limits keep a margin below the measured 0.38 / 0.40 / 0.11.
        for (const auto& [name,target,area]:{std::tuple{"relaxed",relaxedHandPose(),.3f},{"open",openHandPose(),.3f},{"opposition",oppositionHandPose(),.08f}}) {
            animation::setBindPose(asset.skeleton(),pose.locals);
            for (unsigned side=0;side<2;++side) layer.applyPose(pose.locals,side,target);
            refresh(asset,pose);
            requireClean(skin,pose.palette,name,&report,area,true);
        }
        // Opposition: along the anatomical path from the open hand the thumb nail turns palmar,
        // the pad aims at the index tip and turns towards the index pad, with clean skin at every step.
        {
            std::array<float,2> firstNail{},lastNail{},lastFacing{},lastAim{},firstFacing{};
            for (int step=0;step<=10;++step) {
                const float t=static_cast<float>(step)/10;
                const auto lerp=[t](float a,float b) { return a+(b-a)*t; };
                const auto a=openHandPose(), b=oppositionHandPose();
                HandPose p;
                for (unsigned f=0;f<4;++f) p.fingers[f]={lerp(a.fingers[f].mcpFlexion,b.fingers[f].mcpFlexion),lerp(a.fingers[f].mcpSpread,b.fingers[f].mcpSpread),
                    lerp(a.fingers[f].pipFlexion,b.fingers[f].pipFlexion),lerp(a.fingers[f].dipFlexion,b.fingers[f].dipFlexion)};
                p.thumb={lerp(a.thumb.palmarAbduction,b.thumb.palmarAbduction),lerp(a.thumb.radialAbduction,b.thumb.radialAbduction),
                    lerp(a.thumb.pronation,b.thumb.pronation),lerp(a.thumb.mcpFlexion,b.thumb.mcpFlexion),lerp(a.thumb.ipFlexion,b.thumb.ipFlexion)};
                animation::setBindPose(asset.skeleton(),pose.locals);
                for (unsigned side=0;side<2;++side) layer.applyPose(pose.locals,side,p);
                refresh(asset,pose);
                requireClean(skin,pose.palette,"opposition_path_"+std::to_string(step),nullptr,.08f);
                for (unsigned side=0;side<2;++side) {
                    const auto h=chain(layer,side);
                    const auto& an=layer.anatomy(side);
                    const auto& bind=asset.nodes();
                    // Resource space: 0.025 and 0.0325 units are 10 and 13 mm.
                    const auto carried=[&](std::uint32_t n,glm::vec3 v) {
                        return glm::normalize(glm::mat3{pose.globals[n]}*glm::inverse(glm::mat3{bind[n].globalBindTransform})*v); };
                    const auto padT=carried(h.node[4][2],glm::normalize(-.70f*an.radial+.71f*an.palmar));
                    // Finger pad at bind: perpendicular to the distal phalanx in its flexion plane (the left one is curled).
                    const auto padF=carried(h.node[0][2],glm::normalize(glm::cross(an.flexion[0],glm::normalize(glm::vec3{bind[h.node[0][2]].globalBindTransform[1]}))));
                    const auto tipT=glm::vec3{pose.globals[h.node[4][2]][3]}+glm::normalize(glm::vec3{pose.globals[h.node[4][2]][1]})*.025f;
                    const auto tipF=glm::vec3{pose.globals[h.node[0][2]][3]}+glm::normalize(glm::vec3{pose.globals[h.node[0][2]][1]})*.0325f;
                    const float facing=glm::dot(padT,-padF), aim=glm::dot(padT,glm::normalize(tipF-tipT));
                    // Nail angle in the posed hand plane (radial 0, palmar +90 degrees).
                    const auto frame=glm::mat3{pose.globals[h.node[4][2]]}*glm::inverse(glm::mat3{bind[h.node[4][2]].globalBindTransform});
                    const auto nail=glm::normalize(frame*glm::normalize(.85f*an.radial-.49f*an.palmar));
                    const auto handFrame=glm::mat3{pose.globals[asset.nodes()[h.node[0][0]].parent]}*glm::inverse(glm::mat3{bind[asset.nodes()[h.node[0][0]].parent].globalBindTransform});
                    const auto radial=glm::normalize(handFrame*an.radial), palmar=glm::normalize(handFrame*an.palmar);
                    const float nailDegrees=glm::degrees(std::atan2(glm::dot(nail,palmar),glm::dot(nail,radial)));
                    if (step==0) { firstFacing[side]=facing; firstNail[side]=nailDegrees; }
                    if (step==10) { lastFacing[side]=facing; lastAim[side]=aim; lastNail[side]=nailDegrees; }
                    if (step==0 || step==10) report << "opposition_" << (step ? "end_":"start_") << (side ? "right":"left") << " nail_deg=" << nailDegrees
                        << " pad_facing_cos=" << facing << " pad_aim_cos=" << aim << '\n';
                }
            }
            for (unsigned side=0;side<2;++side)
                require(lastNail[side]>firstNail[side]+50 && lastAim[side]>.8f && lastFacing[side]>.45f && lastFacing[side]>firstFacing[side]+.8f,
                    "Thumb pad does not turn towards the fingers");
        }
        // Isolated bends within the verified working range, one chain at a time from the open hand.
        struct Sweep { const char* name; float to; std::function<void(HandPose&,unsigned,float)> set; };
        const std::array<Sweep,8> sweeps{{
            {"mcp",40,[](HandPose& p,unsigned f,float a) { p.fingers[f].mcpFlexion=a; }},
            {"pip",80,[](HandPose& p,unsigned f,float a) { p.fingers[f].pipFlexion=a; }},
            {"dip",60,[](HandPose& p,unsigned f,float a) { p.fingers[f].dipFlexion=a; }},
            {"thumb_palmar",45,[](HandPose& p,unsigned,float a) { p.thumb.palmarAbduction=a; }},
            {"thumb_radial",40,[](HandPose& p,unsigned,float a) { p.thumb.radialAbduction=a; }},
            {"thumb_pronation",60,[](HandPose& p,unsigned,float a) { p.thumb.pronation=a; }},
            {"thumb_mcp",45,[](HandPose& p,unsigned,float a) { p.thumb.mcpFlexion=a; }},
            {"thumb_ip",50,[](HandPose& p,unsigned,float a) { p.thumb.ipFlexion=a; }}}};
        unsigned samples=0;
        for (const auto& s:sweeps) for (unsigned f=0;f<(std::string_view{s.name}.starts_with("thumb") ? 1u:4u);++f)
            for (int step=0;step<=5;++step) {
                const float a=glm::radians(s.to*static_cast<float>(step)/5);
                animation::setBindPose(asset.skeleton(),pose.locals);
                for (unsigned side=0;side<2;++side) { auto p=openHandPose(); s.set(p,f,a); layer.applyPose(pose.locals,side,p); }
                refresh(asset,pose);
                requireClean(skin,pose.palette,std::string{"bend_"}+s.name+"_"+fingers[f]+"_"+std::to_string(static_cast<int>(glm::degrees(a))));
                ++samples;
            }
        report << "isolated_bend_samples=" << samples << " range_deg mcp<=40 pip<=80 dip<=60 thumb palmar<=45 radial<=40 pronation<=60 mcp<=45 ip<=50 intersections=0\n";
    }

    float angularRate(const glm::mat4& a,const glm::mat4& b)
    {
        const auto qa=normalizedReachOrientation(glm::quat_cast(glm::mat3{a})), qb=normalizedReachOrientation(glm::quat_cast(glm::mat3{b}));
        return static_cast<float>(glm::length(orientationResidual(qb,qa))*120);
    }

    void idle(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& profile,const HandLayer& layer,
        hand_pose_audit::Skin& skin,std::ostream& report)
    {
        // Outside Idle the direct mode keeps the exact bind pose required for skinning diagnostics.
        BodyInstance direct{1,asset,&profile,false};
        command(direct,SetDirectMode{true});
        require(direct.snapshot().pose.palette==asset->skinningMatrices(),"Direct mode no longer shows the exact bind pose");
        command(direct,SetIdleHands{true},BodyCommandStatus::Rejected);

        BodyInstance body{2,asset,&profile,false};
        command(body,StartIdle{});
        const auto& s0=body.snapshot();
        require(s0.hands[0].relaxed && s0.hands[1].relaxed && !s0.hands[0].enabled && !s0.hands[1].enabled,"Idle did not engage the relaxed hands");
        for (const auto& j:layer.joints())
            require(s0.pose.locals[j.node]==j.before*glm::mat4_cast(j.rest)*j.after,"Idle finger differs from the calibrated rest");
        for (unsigned side=0;side<2;++side) {
            const auto m=measuredFingers(body.control()->binding(),layer,s0.pose.globals,side);
            const auto target=relaxedHandPose();
            for (unsigned f=0;f<4;++f) require(std::abs(m[f][2]-target.fingers[f].pipFlexion)<glm::radians(.1f),"Idle PIP differs from the relaxed pose");
        }
        while (body.snapshot().idle.phase!=IdlePhase::Idle) body.step();
        // Rest -> open -> rest through the application command, every tick and between ticks.
        command(body,SetIdleHands{true});
        command(body,SetIdleHands{false},BodyCommandStatus::Rejected); // stroke active
        float maxRate=0,maxAccel=0,maxDisplayStep=0,startRate=0;
        std::vector<float> previousRate(asset->nodes().size(),0);
        auto previous=body.previousSnapshot().pose.locals;
        unsigned samples=0;
        const auto track=[&](bool first) {
            const auto& s=body.snapshot();
            for (const auto& j:layer.joints()) {
                const float rate=angularRate(previous[j.node],s.pose.locals[j.node]);
                maxRate=std::max(maxRate,rate); maxAccel=std::max(maxAccel,std::abs(rate-previousRate[j.node])*120);
                if (first) startRate=std::max(startRate,rate);
                previousRate[j.node]=rate;
            }
            previous=s.pose.locals;
            const auto mid=body.display(.5);
            for (const auto& j:layer.joints()) {
                maxDisplayStep=std::max(maxDisplayStep,angularRate(body.previousSnapshot().pose.locals[j.node],mid.pose.locals[j.node])/120);
                maxDisplayStep=std::max(maxDisplayStep,angularRate(mid.pose.locals[j.node],s.pose.locals[j.node])/120);
            }
            if (s.tick%2==0) { requireClean(skin,s.pose.palette,"transition"); requireClean(skin,mid.pose.palette,"transition_display"); samples+=2; }
        };
        track(true);
        for (unsigned t=0;t<130;++t) { body.step(); track(false); }
        const auto& open=body.snapshot();
        require(open.hands[0].enabled && open.hands[1].enabled && open.hands[0].blend==1 && open.hands[0].closure==0 &&
            body.snapshot().hands[0].velocity==0,"Hands did not open to rest");
        requireClean(skin,open.pose.palette,"idle_open",&report,.05f);
        command(body,SetIdleHands{false}); track(false);
        for (unsigned t=0;t<130;++t) { body.step(); track(false); }
        const auto& back=body.snapshot();
        require(!back.hands[0].enabled && !back.hands[1].enabled && back.hands[0].relaxed && back.hands[1].relaxed,"Hands did not return to the relaxed rest");
        for (const auto& j:layer.joints()) require(back.pose.locals[j.node]==j.before*glm::mat4_cast(j.rest)*j.after,"Fade did not return to the calibrated rest");
        require(startRate<.02f && maxRate<3 && maxAccel<30 && maxDisplayStep<.015f,"Hand transition is not smooth");
        report << "rest_open_rest_samples=" << samples << " max_joint_rate_rad_s=" << maxRate << " max_joint_acceleration_rad_s2=" << maxAccel
            << " start_rate_rad_s=" << startRate << " max_display_half_step_rad=" << maxDisplayStep << '\n';
        // Ready keeps its own ownership rules; the empty-hand command belongs to moving Idle.
        command(body,SetIdleStance{true});
        command(body,SetIdleHands{true},BodyCommandStatus::Rejected);
        while (body.snapshot().idle.phase!=IdlePhase::Ready) body.step();
        command(body,SetIdleStance{false});
        while (body.snapshot().idle.phase!=IdlePhase::Idle) body.step();
        require(body.snapshot().hands[0].relaxed && !body.snapshot().hands[0].enabled,"Ready round trip lost the relaxed rest");
        command(body,ResetBody{});
        require(!body.snapshot().hands[0].relaxed && body.snapshot().pose.palette==asset->skinningMatrices(),"Reset did not restore the exact bind pose");
        report << "idle_rest_bind_separation_commands=PASS\n";
    }
}

int main(int argc,char** argv)
{
    try {
        if (argc!=2) throw std::invalid_argument("Expected output directory");
        const std::filesystem::path output{argv[1]};
        std::filesystem::create_directories(output);
        std::ofstream report(output/"hand-calibration.txt");
        if (!report) throw std::runtime_error("Cannot open hand calibration report");
        report << std::setprecision(6);
        const auto asset=model_import::ModelImporter::loadFromFile(
            std::filesystem::path{JULIA_TEST_ASSET_DIR}/"models/Raphtalia_prepared.fbx",
            {.rebuildSmoothNormals=true,.omitPreparedGloves=true});
        const auto profile=raphtaliaBodyProfile();
        const BodyRigBinding binding(profile,*asset);
        const HandLayer layer(binding);
        hand_pose_audit::Skin skin(*asset);
        report << "skin_vertices=" << skin.vertexCount() << " skin_triangles=" << skin.triangleCount() << '\n';
        anatomy(*asset,binding,layer,report);
        staticPoses(*asset,layer,skin,report);
        idle(asset,profile,layer,skin,report);
        report << "result=PASS\n";
        return 0;
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
