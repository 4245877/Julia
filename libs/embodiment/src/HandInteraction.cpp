#include "julia/embodiment/HandInteraction.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace julia::embodiment
{
    namespace
    {
        constexpr std::uint64_t objectNamespace=1ull<<63;
        // Absolute phalanx angles in the palm frame, calibrated on bare skin.
        // Fingers meet the distal face; the thumb opposes
        // along the adjacent side face.
        constexpr float cubeBend[5][3]{
            {1.385f,1.515f,2.580f}, {.685f,1.673f,2.943f}, {.680f,1.649f,2.620f},
            {1.210f,2.601f,3.010f}, {1.131f,1.484f,1.548f}
        };
        constexpr float cubeSpread[5][3]{
            {.216f,.218f,.224f}, {-.059f,-.059f,-.059f}, {.121f,.121f,.121f},
            {-.330f,-.460f,-.364f}, {.160f,.566f,.772f}
        };
        // Calibrated against the 22 mm sphere throughout the closing stroke.
        constexpr float sphereBend[5][3]{
            {1.521f,1.918f,2.791f}, {.710f,1.760f,2.985f}, {.741f,1.884f,2.735f},
            {1.158f,2.491f,2.980f}, {.658f,1.048f,1.626f}
        };
        constexpr float sphereSpread[5][3]{
            {.154f,.130f,.122f}, {-.059f,-.058f,-.057f}, {.107f,.122f,.131f},
            {-.306f,-.253f,-.244f}, {.157f,.268f,.377f}
        };
        // Thumb pad directions in the hand frame (radial, distal, palmar), measured on the
        // modelled skin as the thickest tissue around each phalanx. They fix the thumb's own
        // flexion plane, which differs from the plane of the four fingers.
        constexpr glm::vec3 thumbMcpPad{-.60f,0,.80f}, thumbIpPad{-.70f,0,.71f};
        constexpr float thumbTipLength=.012f; // IP centre to the pad of the tip, body metres
        glm::vec3 unit(const glm::vec3& v) { return glm::normalize(v); }
        glm::vec3 perpendicular(const glm::vec3& v,const glm::vec3& axis) { return glm::normalize(v-glm::dot(v,axis)*axis); }
        float signedAngle(const glm::vec3& from,const glm::vec3& to,const glm::vec3& axis)
        {
            const auto u=perpendicular(from,axis), v=perpendicular(to,axis);
            return std::atan2(glm::dot(glm::cross(u,v),axis),glm::dot(u,v));
        }
        std::size_t index(ArmSide arm)
        { if (arm!=ArmSide::Left && arm!=ArmSide::Right) throw std::invalid_argument("Invalid hand"); return arm==ArmSide::Left ? 0:1; }
        EffectorId effector(ArmSide arm) { return arm==ArmSide::Left ? EffectorId::LeftGrip:EffectorId::RightGrip; }
        bool active(ReachState s) { return s==ReachState::Moving || s==ReachState::Planning || s==ReachState::Settling; }
        glm::mat4 proper(const glm::mat4& m)
        {
            const auto y=glm::normalize(glm::vec3{m[1]});
            const auto z=glm::normalize(glm::vec3{m[2]}-glm::dot(glm::vec3{m[2]},y)*y);
            glm::mat4 f{1}; f[0]=glm::vec4{glm::cross(y,z),0}; f[1]=glm::vec4{y,0}; f[2]=glm::vec4{z,0}; f[3]=m[3]; return f;
        }
        RigidPose checked(RigidPose p)
        {
            for (int k=0;k<3;++k) if (!std::isfinite(p.position[k])) throw std::invalid_argument("Nonfinite object pose");
            if (glm::length(p.position)>100) throw std::invalid_argument("Object pose outside 100 metre range");
            p.rotation=normalizedReachOrientation(p.rotation); return p;
        }
    }
    glm::mat4 rigidMatrix(const RigidPose& p) { return glm::translate(glm::mat4{1},p.position)*glm::mat4_cast(p.rotation); }
    RigidPose rigidPose(const glm::mat4& m) { return {glm::vec3{m[3]},normalizedReachOrientation(glm::quat_cast(glm::mat3{m}))}; }
    world::SceneObject InteractiveObject::collider() const
    {
        world::SceneObject collider{objectNamespace|handle.id,worldPose.position,glm::normalize(glm::dquat{worldPose.rotation}),{},{}};
        std::visit([&](auto volume) { collider.shape=volume; },shape);
        return collider;
    }
    const char* interactionPhaseName(InteractionPhase p)
    {
        switch(p) {
        case InteractionPhase::Idle:return "Idle"; case InteractionPhase::Opening:return "Opening";
        case InteractionPhase::Approach:return "Approach"; case InteractionPhase::Align:return "Align";
        case InteractionPhase::Closing:return "Closing"; case InteractionPhase::Holding:return "Holding";
        case InteractionPhase::Transport:return "Transport"; case InteractionPhase::ReleaseOpening:return "ReleaseOpening";
        case InteractionPhase::Retreat:return "Retreat"; case InteractionPhase::Cancelling:return "Cancelling";
        case InteractionPhase::Succeeded:return "Succeeded"; case InteractionPhase::Failed:return "Failed";
        case InteractionPhase::Cancelled:return "Cancelled"; case InteractionPhase::ReleaseBlocked:return "ReleaseBlocked";
        case InteractionPhase::Handoff:return "Handoff"; case InteractionPhase::ReadyToGrasp:return "ReadyToGrasp";
        case InteractionPhase::PreparationBlocked:return "PreparationBlocked";
        } return "Invalid";
    }
    HandPose relaxedHandPose()
    {
        // Resting cascade: flexion grows from index to little finger; the thumb rests
        // slightly palmar and pronated, its pad towards the index finger.
        const auto r=[](float degrees) { return glm::radians(degrees); };
        HandPose p;
        p.fingers={FingerAngles{r(12),r(-3.5f),r(22),r(10)},FingerAngles{r(16),r(-5.5f),r(28),r(12)},
            FingerAngles{r(20),r(-8),r(33),r(13)},FingerAngles{r(24),r(-11.5f),r(36),r(14)}};
        p.thumb={r(12),r(-5),r(35),r(10),r(14)};
        return p;
    }
    HandPose openHandPose()
    {
        // Extended, softly fanned fingers; thumb abducted away from the index finger.
        const auto r=[](float degrees) { return glm::radians(degrees); };
        HandPose p;
        p.fingers={FingerAngles{r(2),r(6),r(4),r(2)},FingerAngles{r(2),r(-3),r(4),r(2)},
            FingerAngles{r(3),r(-11),r(5),r(3)},FingerAngles{r(3),r(-19),r(5),r(3)}};
        p.thumb={r(10),r(30),r(-5),r(0),r(-5)};
        return p;
    }
    HandPose oppositionHandPose()
    {
        // Opposition onset: CMC palmar abduction and axial pronation turn the thumb nail from
        // radial-dorsal to palmar and aim the pad at the index tip. This is 45 % of the way to
        // tip contact (palmar abduction 60, pronation 120); beyond about half way the thenar
        // skin folds into the palm with linear blend skinning on this mesh.
        const auto r=[](float degrees) { return glm::radians(degrees); };
        HandPose p;
        p.fingers={FingerAngles{r(23),r(2),r(29),r(13)},FingerAngles{r(24),r(-4),r(30),r(14)},
            FingerAngles{r(25),r(-10),r(32),r(14)},FingerAngles{r(26),r(-16),r(32),r(14)}};
        p.thumb={r(32),r(14),r(51),r(0),r(0)};
        return p;
    }
    HandLayer::HandLayer(const BodyRigBinding& binding)
    {
        const auto& nodes=binding.asset().nodes();
        nodeCount_=nodes.size();
        const auto& skeleton=binding.asset().skeleton();
        const auto& bodyFromResource=binding.profile().bodyFromResource;
        std::vector<glm::mat4> bind;
        for (const auto& node:skeleton.nodes()) bind.push_back(node.globalBindTransform);
        for (unsigned side=0;side<2;++side)
        {
            const auto wrist=side==0 ? JointId::LeftWrist:JointId::RightWrist;
            const auto wristNode=binding.jointNode(wrist);
            const auto palm=binding.jointFrame(wrist,bind);
            std::array<std::array<glm::vec3,3>,5> head{}, axis{};
            for (unsigned finger=0;finger<5;++finger)
            {
                const std::array<const char*,5> names{"index","middle","ring","pinky","thumb"};
                std::uint32_t parent=wristNode;
                auto parentClosed=bodyFromResource*bind[parent],parentCube=parentClosed,parentAperture=parentClosed;
                for (unsigned phalanx=0;phalanx<3;++phalanx)
                {
                    const auto prefix=std::string{"c_"}+names[finger]+std::to_string(phalanx+1)+(side==0 ? ".l_":".r_");
                    auto node=animation::NoNode;
                    for (std::uint32_t n=0;n<nodes.size();++n) if (nodes[n].name.starts_with(prefix))
                    { if (node!=animation::NoNode) throw std::invalid_argument("Ambiguous finger node"); node=n; }
                    if (node==animation::NoNode || nodes[node].parent!=parent) throw std::invalid_argument("Unsupported finger chain");
                    const auto global=bodyFromResource*bind[node];
                    const auto frame=proper(global);
                    const auto before=glm::inverse(bodyFromResource*bind[parent])*frame;
                    const auto after=glm::inverse(frame)*global;
                    head[finger][phalanx]=glm::vec3{global[3]}; axis[finger][phalanx]=unit(glm::vec3{global[1]});
                    // Grasp shapes: absolute phalanx orientations in the profile wrist frame,
                    // unchanged from the calibrated cube and sphere holds.
                    const auto target=[&](float bend,float opposition,const glm::mat4& parentGlobal)
                    {
                        auto f=palm*glm::rotate(glm::mat4{1},opposition,glm::vec3{0,0,1})*
                            glm::rotate(glm::mat4{1},bend,glm::vec3{1,0,0});
                        f[3]=parentGlobal*nodes[node].localTransform[3];
                        return f*after;
                    };
                    const auto sideSign=side==0 ? 1.f:-1.f;
                    const float apertureSpread=finger==4 ? -.65f*sideSign:(static_cast<float>(finger)-1.4f)*.045f*sideSign;
                    const auto apertureGlobal=target(0,apertureSpread,parentAperture);
                    const auto closedGlobal=target(sphereBend[finger][phalanx],
                        sphereSpread[finger][phalanx]*sideSign,parentClosed);
                    const auto cubeGlobal=target(cubeBend[finger][phalanx],
                        cubeSpread[finger][phalanx]*sideSign,parentCube);
                    const auto rotation=[&](const glm::mat4& targetGlobal,const glm::mat4& parentGlobal)
                    { return normalizedReachOrientation(glm::quat_cast(glm::mat3{glm::inverse(before)*glm::inverse(parentGlobal)*targetGlobal*glm::inverse(after)})); };
                    Joint joint{node,side,finger,phalanx,before,after};
                    joint.frame=glm::normalize(glm::quat_cast(glm::mat3{frame}));
                    joint.graspAperture=rotation(apertureGlobal,parentAperture);
                    joint.closed=rotation(closedGlobal,parentClosed); joint.cubeClosed=rotation(cubeGlobal,parentCube);
                    joints_.push_back(joint);
                    parent=node; parentAperture=apertureGlobal; parentClosed=closedGlobal; parentCube=cubeGlobal;
                }
            }
            // Anatomical frame from the bind skeleton. Flexion axes are perpendicular to each
            // proximal phalanx and the palm normal. The strongly curled left-hand fingers lie in
            // these planes within 1.5 degrees; the nearly straight right hand defines no plane.
            auto& a=anatomy_[side];
            a.distal=unit(glm::vec3{palm[1]}); a.palmar=-unit(glm::vec3{palm[2]});
            a.radial=side==0 ? unit(glm::vec3{palm[0]}):-unit(glm::vec3{palm[0]});
            a.spread=unit(glm::cross(a.distal,a.radial));
            for (unsigned f=0;f<4;++f) {
                const auto v1=unit(head[f][1]-head[f][0]), v2=unit(head[f][2]-head[f][1]), v3=axis[f][2];
                a.flexion[f]=unit(glm::cross(v1,a.palmar));
                a.bind.fingers[f]={std::asin(std::clamp(glm::dot(v1,a.palmar),-1.f,1.f)),
                    std::atan2(glm::dot(v1,a.radial),glm::dot(v1,a.distal)),
                    signedAngle(v1,v2,a.flexion[f]),signedAngle(v2,v3,a.flexion[f])};
            }
            const auto handVector=[&](const glm::vec3& v) { return unit(v.x*a.radial+v.y*a.distal+v.z*a.palmar); };
            const auto metacarpal=unit(head[4][1]-head[4][0]), proximal=unit(head[4][2]-head[4][1]), distal=axis[4][2];
            a.thumbPalmar=unit(glm::cross(metacarpal,a.palmar));
            a.thumbRadial=a.spread;
            // Axial rotation about the whole thumb ray (CMC to tip): the modelled MCP is deviated
            // towards the index, so a turn about the short metacarpal alone would swing the tip.
            // Positive pronation turns the nail from radial towards palmar and the pad towards the fingers.
            const auto ray=unit(head[4][2]+distal*thumbTipLength-head[4][0]);
            const auto radialAcross=perpendicular(a.radial,ray);
            a.thumbAxial=glm::dot(glm::cross(ray,radialAcross),a.palmar)>0 ? ray:-ray;
            a.thumbMcp=unit(glm::cross(proximal,perpendicular(handVector(thumbMcpPad),proximal)));
            a.thumbIp=unit(glm::cross(distal,perpendicular(handVector(thumbIpPad),distal)));
            a.bind.thumb={0,0,0,signedAngle(metacarpal,proximal,a.thumbMcp),signedAngle(proximal,distal,a.thumbIp)};
        }
        const auto relaxed=relaxedHandPose(), open=openHandPose();
        for (unsigned side=0;side<2;++side) {
            const auto rest=rotations(side,relaxed), wide=rotations(side,open);
            for (std::size_t k=0;k<15;++k) { joints_[side*15+k].rest=rest[k]; joints_[side*15+k].open=wide[k]; }
        }
    }
    std::array<glm::quat,15> HandLayer::rotations(unsigned side,const HandPose& pose) const
    {
        const auto& a=anatomy_.at(side);
        const auto turn=[](float angle,const glm::vec3& axis) { return glm::angleAxis(angle,axis); };
        // Rotations about each joint centre in body space, bind configuration.
        std::array<glm::quat,15> body{};
        for (unsigned f=0;f<4;++f) {
            const auto& t=pose.fingers[f]; const auto& b=a.bind.fingers[f];
            body[f*3]=turn(t.mcpSpread-b.mcpSpread,a.spread)*turn(t.mcpFlexion-b.mcpFlexion,a.flexion[f]);
            body[f*3+1]=turn(t.pipFlexion-b.pipFlexion,a.flexion[f]);
            body[f*3+2]=turn(t.dipFlexion-b.dipFlexion,a.flexion[f]);
        }
        const auto& t=pose.thumb; const auto& b=a.bind.thumb;
        body[12]=turn(t.radialAbduction-b.radialAbduction,a.thumbRadial)*turn(t.palmarAbduction-b.palmarAbduction,a.thumbPalmar)*
            turn(t.pronation-b.pronation,a.thumbAxial);
        body[13]=turn(t.mcpFlexion-b.mcpFlexion,a.thumbMcp);
        body[14]=turn(t.ipFlexion-b.ipFlexion,a.thumbIp);
        std::array<glm::quat,15> local{};
        for (std::size_t k=0;k<15;++k) {
            const auto& frame=joints_.at(side*15+k).frame;
            local[k]=normalizedReachOrientation(glm::conjugate(frame)*body[k]*frame);
        }
        return local;
    }
    void HandLayer::applyPose(std::span<glm::mat4> locals,unsigned side,const HandPose& pose) const
    {
        if (locals.size()!=nodeCount_) throw std::invalid_argument("Hand layer local buffer size mismatch");
        const auto q=rotations(side,pose);
        for (std::size_t k=0;k<15;++k) {
            const auto& j=joints_[side*15+k];
            locals[j.node]=j.before*glm::mat4_cast(q[k])*j.after;
        }
    }
    void HandLayer::set(ArmSide arm,float closure,bool enabled,std::optional<HandGrip> grip)
    {
        const auto s=index(arm);
        if (!std::isfinite(closure) || closure<0 || closure>1) throw std::invalid_argument("Hand closure outside [0,1]");
        if (!ready(arm)) throw std::invalid_argument("Hand trajectory active; wait for rest");
        if (!enabled) { states_[s]=HandState{.relaxed=states_[s].relaxed}; from_[s]=goal_[s]=0; elapsed_[s]=duration_[s]=0; return; }
        if (grip) states_[s].grip=*grip;
        blendFrom_[s]=states_[s].blend; blendGoal_[s]=1;
        states_[s].enabled=enabled; from_[s]=states_[s].closure; goal_[s]=closure; elapsed_[s]=0;
        // Quintic: |dc/dt| <= 1.875/s, |d2c/dt2| <= 5.78/s2 for a full stroke.
        // One second for every stroke is conservative even for partial travel.
        duration_[s]=from_[s]==closure && states_[s].blend==1 ? 0:120;
    }
    void HandLayer::fadeOut(ArmSide arm)
    {
        const auto s=index(arm);
        if (!states_[s].enabled) return;
        set(arm,states_[s].closure);
        blendGoal_[s]=0;
        duration_[s]=120;
    }
    void HandLayer::advance()
    {
        for (std::size_t s=0;s<2;++s) if (elapsed_[s]<duration_[s])
        {
            const double t=static_cast<double>(++elapsed_[s])/duration_[s];
            const double weight=t*t*t*(10+t*(-15+6*t));
            const auto old=states_[s].closure;
            states_[s].closure=static_cast<float>(from_[s]+(goal_[s]-from_[s])*weight);
            states_[s].blend=static_cast<float>(blendFrom_[s]+(blendGoal_[s]-blendFrom_[s])*weight);
            states_[s].velocity=(states_[s].closure-old)*120;
        } else {
            states_[s].velocity=0;
            if (blendGoal_[s]==0) states_[s]=HandState{.relaxed=states_[s].relaxed};
        }
    }
    bool HandLayer::ready(ArmSide arm) const
    { const auto s=index(arm); return elapsed_[s]>=duration_[s] && states_[s].velocity==0; }
    void HandLayer::apply(std::span<glm::mat4> locals,const std::array<HandState,2>& states) const
    {
        if (locals.size()!=nodeCount_) throw std::invalid_argument("Hand layer local buffer size mismatch");
        for (const auto& s:states) if (!std::isfinite(s.closure) || !std::isfinite(s.blend) ||
            s.closure<0 || s.closure>1 || s.blend<0 || s.blend>1) throw std::invalid_argument("Invalid hand layer state");
        for (const auto& j:joints_) {
            const auto& s=states[j.side];
            const auto rest=s.relaxed ? j.rest:glm::quat{1,0,0,0};
            if (!s.enabled || s.blend<=0) {
                if (s.relaxed) locals[j.node]=j.before*glm::mat4_cast(rest)*j.after;
                continue;
            }
            const auto closed=s.grip==HandGrip::Cube ? j.cubeClosed:j.closed;
            // Quintic stage weights keep closing C2-smooth; from ApertureEnd the stroke visits
            // exactly the calibrated aperture-to-hold path.
            const auto stage=[](float from,float to,float x) {
                const float u=std::clamp((x-from)/(to-from),0.f,1.f); return u*u*u*(10+u*(-15+6*u)); };
            const auto aperture=glm::slerp(j.open,j.graspAperture,stage(0,ApertureEnd,s.closure));
            const auto shaped=glm::slerp(aperture,closed,stage(StrokeStart,1,s.closure));
            locals[j.node]=j.before*glm::mat4_cast(glm::slerp(rest,shaped,s.blend))*j.after;
        }
    }
    HandInteraction::HandInteraction(std::uint64_t owner,DirectBodyControl& control,ArmReach& reach)
        :hand(control.binding()),owner_(owner),control_(control),reach_(reach) {}
    bool HandInteraction::locked() const
    {
        if (object_ && object_->state==ObjectState::Attached) return true;
        const auto p=observation_.phase;
        return p==InteractionPhase::Opening || p==InteractionPhase::Approach || p==InteractionPhase::Align ||
            p==InteractionPhase::Handoff || p==InteractionPhase::ReadyToGrasp || p==InteractionPhase::PreparationBlocked ||
            p==InteractionPhase::Closing || p==InteractionPhase::ReleaseOpening || p==InteractionPhase::Retreat || p==InteractionPhase::Cancelling;
    }
    void HandInteraction::requireObject(ObjectHandle h) const
    { if (!object_ || object_->handle!=h) throw std::invalid_argument("Missing object or stale generation"); }
    void HandInteraction::phase(InteractionPhase p,bool geometry)
    {
        observation_.phase=p;
        if (geometry) {
            contactAllowed_=p==InteractionPhase::Align || p==InteractionPhase::Closing || p==InteractionPhase::Holding ||
                p==InteractionPhase::Transport || p==InteractionPhase::ReleaseOpening || p==InteractionPhase::Retreat || p==InteractionPhase::ReadyToGrasp;
            ++geometryRevision_;
            if (geometryChanged) geometryChanged();
        }
    }
    void HandInteraction::create(const CreateInteractionSphere& c,const world::CollisionScene* scene,double minimumGap)
    {
        createObject(c.id,c.pose,world::Sphere{c.radius},c.localGrip,scene,minimumGap);
    }
    void HandInteraction::create(const CreateInteractionBox& c,const world::CollisionScene* scene,double minimumGap)
    {
        createObject(c.id,c.pose,c.box,interactionCubeLocalGrip,scene,minimumGap);
    }
    void HandInteraction::createObject(std::uint64_t id,RigidPose pose,InteractionShape shape,RigidPose localGrip,
        const world::CollisionScene* scene,double minimumGap)
    {
        if (object_ || locked() || reach_.anyActive()) throw std::invalid_argument("One object only; creation requires idle body");
        if (!id || id>=objectNamespace) throw std::invalid_argument("Invalid interactive object ID");
        const auto p=checked(pose),g=checked(localGrip);
        if (const auto* box=std::get_if<world::Box>(&shape)) {
            if (box->halfExtent!=glm::dvec3{InteractionCubeHalfExtent})
                throw std::invalid_argument("Cube dimensions outside the hand calibration");
        } else {
            const double radius=std::get<world::Sphere>(shape).radius;
            if (!std::isfinite(radius) || std::abs(radius-.022f)>1e-7f)
                throw std::invalid_argument("Demo supports exactly one 22 mm radius sphere");
            if (glm::length(g.position)>radius) throw std::invalid_argument("Grip must lie inside the sphere");
        }
        if (generation_==std::numeric_limits<std::uint64_t>::max()) throw std::overflow_error("Object generations exhausted");
        const InteractiveObject candidate{{id,generation_+1},p,shape,g};
        if (scene) {
            const auto collider=candidate.collider();
            for (const auto& obstacle:scene->objects())
                if (world::clearance(collider,obstacle).gap<=minimumGap)
                    throw std::invalid_argument("Object overlaps environment");
        }
        object_=candidate; ++generation_;
        observation_={}; observation_.object=object_->handle; phase(InteractionPhase::Idle,true);
    }
    void HandInteraction::erase(ObjectHandle h)
    {
        requireObject(h);
        if (locked() || reach_.anyActive()) throw std::invalid_argument("Cannot delete an owned/active object");
        object_.reset(); observation_={}; phase(InteractionPhase::Idle,true);
    }
    void HandInteraction::resetFree(std::uint64_t epoch)
    {
        if (locked()) throw std::invalid_argument("Cannot reset owned interaction");
        hand.reset(); observation_={}; observation_.epoch=epoch;
        if (object_) observation_.object=object_->handle;
        cancelBeforeAttachment_=contactAllowed_=false; reachAction_=0;
    }
    void HandInteraction::startReach(const RigidPose& goal,const glm::mat4& root)
    {
        ReachTarget t; t.arm=observation_.arm; t.effector=effector(t.arm); t.space=TargetSpace::World;
        t.task=ReachTask::PositionAndOrientation; t.position=goal.position; t.orientation=goal.rotation;
        t.strategy=MovementStrategy::AllowDetour; t.detour.maxConfigurations=1;
        if (observation_.phase==InteractionPhase::Align && std::holds_alternative<world::Box>(object_->shape)) {
            t.tolerance=.0007f;
            t.angularTolerance=glm::radians(.3f);
        }
        reachAction_=observation_.action;
        if (freezeReach) freezeReach(observation_.arm);
        reach_.start(reachAction_,observation_.epoch,t,root,false,reachAction_);
    }
    void HandInteraction::prepare(std::uint64_t action,std::uint64_t epoch,const PrepareGrasp& c,PathCheck check)
    {
        // BodyInstance has transactionally checked source, ownership and prospective envelope.
        requireObject(c.object); (void)index(c.arm);
        observation_={}; observation_.action=action; observation_.epoch=epoch; observation_.object=c.object; observation_.arm=c.arm;
        observation_.preparation=true; observation_.sourceReach=c.reach; observation_.sourceEpoch=c.reachEpoch;
        observation_.geometryCheck=std::move(check);
        observation_.phaseCheckPoses=observation_.geometryCheck.poses; observation_.phaseCheckQueries=observation_.geometryCheck.queries;
        observation_.phaseCheckMicroseconds=observation_.geometryCheck.microseconds;
        observation_.phaseCheckScratchBytes=observation_.geometryCheck.scratchBytes;
        grip_=rigidPose(rigidMatrix(object_->worldPose)*rigidMatrix(object_->localGrip));
        pregrip_=rigidPose(rigidMatrix(grip_)*glm::translate(glm::mat4{1},glm::vec3{0,0,-.075f}));
        cancelBeforeAttachment_=true; finishingPreparation_=false; handoffWait_=true;
        phase(InteractionPhase::Handoff,true);
    }
    void HandInteraction::confirm(std::uint64_t action,const ConfirmGrasp& c)
    {
        requireObject(c.object);
        if (!c.action || observation_.action!=c.action || observation_.epoch!=c.epoch)
            throw std::invalid_argument("Stale grasp preparation action or epoch");
        if (!observation_.preparation || observation_.phase!=InteractionPhase::ReadyToGrasp ||
            observation_.object!=c.object || object_->state!=ObjectState::Free || object_->owner || reach_.anyActive())
            throw std::invalid_argument("Confirmation requires the hand in ReadyToGrasp with its Free object");
        const auto& state=hand.states()[index(observation_.arm)];
        if (!hand.ready(observation_.arm) || !state.enabled || state.closure!=0 || state.blend!=1)
            throw std::invalid_argument("Confirmation requires a fully open, stationary hand");
        hand.set(observation_.arm,1);
        observation_.action=action;
        observation_.result.clear(); observation_.reason.clear();
        phase(InteractionPhase::Closing); // Same envelope and allowed pair as ReadyToGrasp.
    }
    void HandInteraction::finishPreparation(std::uint64_t action)
    {
        if (!observation_.preparation || observation_.phase!=InteractionPhase::ReadyToGrasp)
            throw std::invalid_argument("Finish requires ReadyToGrasp");
        cancel(action); finishingPreparation_=true;
    }
    void HandInteraction::abandonPreparation()
    {
        if (!observation_.preparation || !object_ || object_->state!=ObjectState::Free)
            throw std::invalid_argument("Only Free preparation may be reset");
        observation_.cancellationPending=false; observation_.result="Reset";
        phase(InteractionPhase::Cancelled,true);
    }
    void HandInteraction::grasp(std::uint64_t action,std::uint64_t epoch,const GraspObject& c,const glm::mat4& root)
    {
        requireObject(c.object); (void)index(c.arm);
        if (c.arm!=ArmSide::Left && std::holds_alternative<world::Box>(object_->shape))
            throw std::invalid_argument("Cube grip is calibrated for the left hand");
        if (!control_.enabled() || locked() || reach_.anyActive() || !hand.ready(ArmSide::Left) || !hand.ready(ArmSide::Right)) throw std::invalid_argument("Grasp requires idle direct body and hand");
        observation_={}; observation_.action=action; observation_.epoch=epoch; observation_.object=c.object; observation_.arm=c.arm;
        cancelBeforeAttachment_=false;
        grip_=rigidPose(rigidMatrix(object_->worldPose)*rigidMatrix(object_->localGrip));
        pregrip_=rigidPose(rigidMatrix(grip_)*glm::translate(glm::mat4{1},glm::vec3{0,0,-.075f}));
        phase(InteractionPhase::Opening,true);
        if (!restClear(collisionContext())) { fail("Opening envelope intersects scene/body/object"); return; }
        hand.set(c.arm,0,true,std::holds_alternative<world::Box>(object_->shape) ? HandGrip::Cube:HandGrip::Sphere);
        (void)root;
    }
    void HandInteraction::move(std::uint64_t action,std::uint64_t epoch,const MoveHeldObject& c,const glm::mat4& root)
    {
        requireObject(c.object); const auto p=checked(c.targetWorld);
        if (object_->state!=ObjectState::Attached || object_->owner!=owner_ || object_->arm!=observation_.arm ||
            observation_.object!=c.object || observation_.epoch!=epoch || reach_.anyActive() ||
            (observation_.phase!=InteractionPhase::Holding && observation_.phase!=InteractionPhase::Failed))
            throw std::invalid_argument("Move requires a stationary attached object");
        const auto goal=rigidPose(rigidMatrix(p)*glm::inverse(object_->effectorFromObject));
        const auto previous=observation_; const auto previousReachAction=reachAction_;
        observation_.action=action; observation_.epoch=epoch; observation_.result.clear(); observation_.reason.clear();
        try { startReach(goal,root); phase(InteractionPhase::Transport); }
        catch (...) { observation_=previous; reachAction_=previousReachAction; throw; }
    }
    void HandInteraction::release(std::uint64_t action,std::uint64_t epoch,ObjectHandle h)
    {
        requireObject(h);
        if (object_->state!=ObjectState::Attached || object_->owner!=owner_ || object_->arm!=observation_.arm ||
            observation_.object!=h || observation_.epoch!=epoch || reach_.anyActive() || !hand.ready(observation_.arm) ||
            (observation_.phase!=InteractionPhase::Holding && observation_.phase!=InteractionPhase::Failed))
            throw std::invalid_argument("Release requires a stationary attached object");
        observation_.action=action; observation_.epoch=epoch; observation_.result.clear(); observation_.reason.clear();
        hand.set(observation_.arm,0); phase(InteractionPhase::ReleaseOpening);
    }
    void HandInteraction::cancel(std::uint64_t action)
    {
        if (!action || observation_.action!=action || !locked()) throw std::invalid_argument("Stale interaction action");
        if (observation_.phase==InteractionPhase::ReleaseOpening || observation_.phase==InteractionPhase::Retreat)
            throw std::invalid_argument("Release is atomic through retreat; cancellation unsupported");
        observation_.cancellationPending=true;
        finishingPreparation_=false;
        cancelBeforeAttachment_=object_->state==ObjectState::Free;
        const auto& r=reach_.observations()[index(observation_.arm)];
        if (active(r.state)) reach_.cancel(observation_.arm,reachAction_);
        phase(InteractionPhase::Cancelling);
    }
    void HandInteraction::fail(const std::string& why)
    {
        observation_.result="Failed"; observation_.reason=why;
        observation_.cancellationPending=false;
        // A failed transport leaves the same payload and collision policy at rest.
        phase(observation_.preparation ? InteractionPhase::PreparationBlocked:InteractionPhase::Failed,object_->state==ObjectState::Free);
    }
    void HandInteraction::retreatBlocked(const std::string& reason)
    {
        observation_.result="ReleaseBlocked"; observation_.reason=reason;
        observation_.cancellationPending=false;
        phase(observation_.preparation ? InteractionPhase::PreparationBlocked:InteractionPhase::ReleaseBlocked,true);
    }
    bool HandInteraction::restClear(CollisionContext context)
    {
        if (!validateRest) return false;
        const auto check=validateRest(observation_.arm,std::move(context));
        observation_.phaseCheckPoses+=check.poses; observation_.phaseCheckQueries+=check.queries;
        observation_.phaseCheckMicroseconds+=check.microseconds;
        observation_.phaseCheckScratchBytes=check.scratchBytes;
        observation_.geometryCheck=check;
        return check.status==PathStatus::Clear;
    }
    CollisionContext HandInteraction::collisionContext() const
    {
        CollisionContext c;
        for (std::size_t k=0;k<2;++k) c.handEnvelopes[k]=hand.states()[k].enabled;
        if (!object_) return c;
        if (locked()) c.handEnvelopes[index(observation_.arm)]=true;
        if (contactAllowed_ || object_->state==ObjectState::Attached) {
            c.contactHand=observation_.arm==ArmSide::Left ? 3u:6u; c.contactObject=objectNamespace|object_->handle.id;
        }
        if (object_->state==ObjectState::Attached)
            c.carried=CarriedObject{objectNamespace|object_->handle.id,effector(object_->arm),object_->effectorFromObject,object_->shape};
        return c;
    }
    std::optional<world::SceneObject> HandInteraction::freeCollider() const
    {
        if (!object_ || object_->state==ObjectState::Attached) return {};
        return object_->collider();
    }
    void HandInteraction::advance(std::span<const glm::mat4> globals,const glm::mat4& root)
    {
        if (!object_) return;
        const auto s=index(observation_.arm); const auto f=root*control_.binding().effectorFrame(effector(observation_.arm),globals);
        if (object_->state==ObjectState::Attached) object_->worldPose=rigidPose(f*object_->effectorFromObject);
        observation_.hold=object_->state==ObjectState::Attached ? HoldType::AssistedAttachment:HoldType::None;
        const auto goal=rigidMatrix(object_->worldPose)*rigidMatrix(object_->localGrip);
        observation_.frameError=glm::length(glm::vec3{goal[3]-f[3]});
        observation_.angularError=static_cast<float>(glm::length(orientationResidual(rigidPose(goal).rotation,rigidPose(f).rotation)));
        const auto r=reach_.observations()[s];
        const auto p=observation_.phase;
        try
        {
            if (locked() && stanceValid && !stanceValid()) {
                if (active(r.state)) return; // ArmReach's context gate stops invalid execution first.
                if (p==InteractionPhase::Retreat) retreatBlocked("Support/object/frozen geometry changed during retreat");
                else if (p!=InteractionPhase::PreparationBlocked && p!=InteractionPhase::Failed)
                    fail("Interaction support/object/frozen geometry changed");
                return;
            }
            if (p==InteractionPhase::Handoff) {
                if (handoffWait_) { handoffWait_=false; return; }
                if (!restClear(collisionContext())) { fail("Opening envelope no longer clear"); return; }
                phase(InteractionPhase::Opening);
                hand.set(observation_.arm,0,true,std::holds_alternative<world::Box>(object_->shape) ? HandGrip::Cube:HandGrip::Sphere);
            } else if (p==InteractionPhase::Opening && hand.ready(observation_.arm)) {
                phase(InteractionPhase::Approach); startReach(pregrip_,root);
            } else if (p==InteractionPhase::Approach || p==InteractionPhase::Align || p==InteractionPhase::Transport) {
                if (active(r.state)) return;
                if (r.action!=reachAction_ || r.state!=ReachState::Succeeded) { fail(reachReasonName(r.reason)); return; }
                if (p==InteractionPhase::Approach) {
                    phase(InteractionPhase::Align,true); startReach(grip_,root);
                } else if (p==InteractionPhase::Align) {
                    if (observation_.preparation) {
                        if (!hand.ready(observation_.arm) || !hand.states()[s].enabled || hand.states()[s].closure!=0 ||
                            hand.states()[s].blend!=1 || observation_.frameError>r.target.tolerance || observation_.angularError>r.target.angularTolerance ||
                            !r.collisionChecked || object_->state!=ObjectState::Free || !restClear(collisionContext()))
                        { fail("ReadyToGrasp conditions not reached"); return; }
                        // Same geometry/contact policy as Align; no hidden context mutation.
                        observation_.result="ReadyToGrasp"; phase(InteractionPhase::ReadyToGrasp);
                    } else { hand.set(observation_.arm,1); phase(InteractionPhase::Closing); }
                } else {
                    observation_.result="Moved"; phase(InteractionPhase::Holding);
                }
            } else if (p==InteractionPhase::Closing && hand.ready(observation_.arm)) {
                // Timers/closure alone can NEVER attach. Recheck actual FK, settled reach,
                // calibrated envelope against every unpermitted pair and object availability.
                if (observation_.frameError>r.target.tolerance || observation_.angularError>r.target.angularTolerance ||
                    r.action!=reachAction_ || r.epoch!=observation_.epoch || r.state!=ReachState::Succeeded || !r.collisionChecked ||
                    !hand.states()[s].enabled || hand.states()[s].blend!=1 || hand.states()[s].closure!=1 ||
                    object_->state!=ObjectState::Free || object_->owner || object_->handle!=observation_.object)
                { fail("Attachment conditions not reached"); return; }
                // The free object was checked against the body during Align. Verify
                // the prospective payload against the environment too, before committing.
                auto context=collisionContext();
                context.carried=CarriedObject{objectNamespace|object_->handle.id,effector(observation_.arm),
                    glm::inverse(f)*rigidMatrix(object_->worldPose),object_->shape};
                if (!restClear(context)) { fail("Attachment geometry rejected"); return; }
                object_->effectorFromObject=context.carried->effectorFromObject;
                object_->state=ObjectState::Attached; object_->owner=owner_; object_->arm=observation_.arm;
                observation_.preparation=false;
                cancelBeforeAttachment_=false;
                observation_.hold=HoldType::AssistedAttachment; observation_.result="Grasped";
                phase(InteractionPhase::Holding,true);
            } else if (p==InteractionPhase::ReleaseOpening && hand.ready(observation_.arm)) {
                // Pose above is the actual current pose; no snap to the ideal grip.
                object_->state=ObjectState::Free; object_->owner=0; observation_.hold=HoldType::None;
                const auto retreat=rigidPose(f*glm::translate(glm::mat4{1},glm::vec3{0,0,-.075f}));
                phase(InteractionPhase::Retreat,true); startReach(retreat,root);
            } else if (p==InteractionPhase::Retreat && !active(r.state)) {
                auto context=collisionContext(); context.contactHand=0; context.contactObject=0;
                if (r.action!=reachAction_ || r.epoch!=observation_.epoch || r.state!=ReachState::Succeeded) {
                    retreatBlocked(reachReasonName(r.reason)); return;
                }
                if (!restClear(context)) { retreatBlocked("Contact remains after retreat"); return; }
                observation_.result=finishingPreparation_ ? "Prepared":cancelBeforeAttachment_ ? "Cancelled":"Released";
                observation_.reason.clear();
                observation_.cancellationPending=false;
                phase(cancelBeforeAttachment_ && !finishingPreparation_ ? InteractionPhase::Cancelled:InteractionPhase::Succeeded,true);
            } else if (p==InteractionPhase::Cancelling && !active(r.state) && hand.ready(observation_.arm)) {
                if (!cancelBeforeAttachment_) {
                    observation_.result="CancelledHolding"; observation_.cancellationPending=false; phase(InteractionPhase::Holding);
                } else {
                    if (hand.states()[s].closure>0) { hand.set(observation_.arm,0); return; }
                    auto context=collisionContext(); context.contactHand=0; context.contactObject=0;
                    if (restClear(context)) {
                        observation_.result=finishingPreparation_ ? "Prepared":"Cancelled"; observation_.cancellationPending=false;
                        phase(finishingPreparation_ ? InteractionPhase::Succeeded:InteractionPhase::Cancelled,true); return;
                    }
                    phase(InteractionPhase::Retreat,true); startReach(pregrip_,root);
                }
            }
        }
        catch (const std::exception& e) {
            if (observation_.phase==InteractionPhase::Retreat) retreatBlocked(e.what());
            else fail(e.what());
        }
    }
}
