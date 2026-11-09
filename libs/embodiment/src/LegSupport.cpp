#include "julia/embodiment/LegSupport.hpp"
#include "PoseIK.hpp"
#include "julia/runtime/FixedStepScheduler.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace julia::embodiment
{
    namespace
    {
        constexpr double dt=runtime::BodyStepSeconds;
        constexpr std::array<unsigned,6> slot{0,0,0,1,2,2}, component{0,1,2,0,0,1};
        constexpr float numericSlack=.00003f;
        LegSide side(unsigned s) { return s==0 ? LegSide::Left:LegSide::Right; }
        EffectorId sole(unsigned s) { return s==0 ? EffectorId::LeftSole:EffectorId::RightSole; }
        bool finite(glm::vec3 v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }
        void rigid(const glm::mat4& m)
        {
            for (int c=0;c<4;++c) for (int r=0;r<4;++r)
                if (!std::isfinite(m[c][r])) throw std::invalid_argument("Non-finite support frame");
            if (glm::length(glm::vec3{m[3]})>100 || glm::length(glm::vec4{m[0][3],m[1][3],m[2][3],m[3][3]-1})>1e-5f)
                throw std::invalid_argument("Support frame must be affine within 100 world metres");
            const auto r=glm::mat3{m}, e=glm::transpose(r)*r-glm::mat3{1};
            float error=0; for (int c=0;c<3;++c) for (int k=0;k<3;++k) error=std::max(error,std::abs(e[c][k]));
            if (error>1e-4f || std::abs(glm::determinant(r)-1)>1e-4f) throw std::invalid_argument("Support frame must be proper rigid");
        }
        LegCoordinates coordinates(const DirectBodyControl& c,unsigned s)
        {
            LegCoordinates q{}; const auto ids=legJoints(side(s));
            for (unsigned k=0;k<6;++k) q[k]=c.coordinate(ids[slot[k]],component[k]);
            return q;
        }
        void write(DirectBodyControl& c,unsigned s,const LegCoordinates& q)
        {
            const auto ids=legJoints(side(s));
            for (unsigned k=0;k<6;++k)
                if (c.setCoordinate(ids[slot[k]],component[k],q[k]).status==CommandStatus::Rejected)
                    throw std::logic_error("Unvalidated leg coordinate");
        }
        // Semantic leg adapter. Twist branches are deliberately not chain joints.
        struct LegChain
        {
            std::array<const JointDefinition*,3> definitions;
            std::array<double,6> initial{},lower{},upper{};
            glm::mat4 base{1}, hipToKnee{1}, kneeToAnkle{1}, ankleToContact{1};
            float radius=0;
            mutable unsigned evaluations=0;
            LegChain(const DirectBodyControl& c,unsigned s,const SoleContact& contact)
            {
                const auto& b=c.binding(); const auto ids=legJoints(side(s));
                for (unsigned j=0;j<3;++j) definitions[j]=&b.profile().joints[b.jointIndex(ids[j])];
                for (unsigned k=0;k<6;++k) {
                    const auto& d=definitions[slot[k]]->coordinates.at(component[k]);
                    initial[k]=c.coordinate(ids[slot[k]],component[k]); lower[k]=d.lower; upper[k]=d.upper;
                }
                animation::Pose p(b.asset().skeleton()); c.apply(p.locals);
                animation::computeGlobalTransforms(b.asset().skeleton(),p.locals,p.globals);
                base=b.jointFrame(ids[0],p.globals)*glm::inverse(rotation(0,initial));
                const auto a=definitions[0]->frameInBodyBind, k=definitions[1]->frameInBodyBind, f=definitions[2]->frameInBodyBind;
                // Keep the current knee branch. At extension choose the calibrated
                // bind half-plane; no alternate seeds can silently reverse the knee.
                const auto axis=glm::mat3{k}*definitions[1]->coordinates[0].axis;
                const auto thigh=glm::normalize(glm::vec3{k[3]-a[3]}), shank=glm::normalize(glm::vec3{f[3]-k[3]});
                const double bend=std::atan2(glm::dot(axis,glm::cross(thigh,shank)),glm::dot(thigh,shank));
                if (bend+initial[3]>=-1e-5) lower[3]=std::max(lower[3],-bend);
                else upper[3]=std::min(upper[3],-bend);
                hipToKnee=glm::inverse(a)*k; kneeToAnkle=glm::inverse(k)*f;
                const auto it=std::find_if(b.profile().effectors.begin(),b.profile().effectors.end(),[&](const auto& e) { return e.id==sole(s); });
                if (it==b.profile().effectors.end()) throw std::invalid_argument("Leg has no sole effector");
                ankleToContact=glm::inverse(f)*it->frameInBodyBind*contact.soleFromContact;
                radius=glm::length(glm::vec3{hipToKnee[3]})+glm::length(glm::vec3{kneeToAnkle[3]})+glm::length(glm::vec3{ankleToContact[3]});
                float extent=0; for (const auto pnt:contact.points) extent=std::max(extent,glm::length(pnt));
                radius+=extent+numericSlack;
            }
            glm::mat4 rotation(unsigned j,const std::array<double,6>& q) const
            {
                glm::mat4 r{1};
                for (unsigned k=0;k<6;++k) if (slot[k]==j) r=glm::rotate(r,static_cast<float>(q[k]),definitions[j]->coordinates[component[k]].axis);
                return r;
            }
            glm::mat4 frame(const std::array<double,6>& q) const
            { ++evaluations; return base*rotation(0,q)*hipToKnee*rotation(1,q)*kneeToAnkle*rotation(2,q)*ankleToContact; }
        };
        glm::mat4 mixFrame(const glm::mat4& a,const glm::mat4& b,float u)
        {
            auto f=glm::mat4_cast(glm::normalize(glm::slerp(glm::quat_cast(glm::mat3{a}),glm::quat_cast(glm::mat3{b}),u)));
            f[3]=glm::vec4{glm::mix(glm::vec3{a[3]},glm::vec3{b[3]},u),1}; return f;
        }
        bool reached(const LegChain& chain,const std::array<double,6>& q,const glm::mat4& goal)
        {
            const auto f=chain.frame(q);
            return glm::length(glm::vec3{f[3]-goal[3]})<1e-6f &&
                glm::length(orientationResidual(glm::quat_cast(glm::mat3{goal}),glm::quat_cast(glm::mat3{f})))<1e-5;
        }
        void measureFeet(SupportObservation& o,const BodyRigBinding& b,const std::array<SoleContact,2>& contacts,
            std::span<const glm::mat4> globals,const glm::mat4& root)
        {
            for (unsigned s=0;s<2;++s) {
                auto& a=o.anchors[s]; a.actualWorld=root*b.effectorFrame(sole(s),globals)*contacts[s].soleFromContact;
                a.positionError=glm::length(glm::vec3{a.actualWorld[3]-a.desiredWorld[3]});
                a.angularError=static_cast<float>(glm::length(orientationResidual(glm::quat_cast(glm::mat3{a.desiredWorld}),glm::quat_cast(glm::mat3{a.actualWorld}))));
                a.minGap=1e6f; a.maxGap=-1e6f;
                // Anchor's historical surface remains immutable even after deletion/version change.
                const float height=a.surfaceHeight;
                for (unsigned k=0;k<4;++k) {
                    a.worldPoints[k]=glm::vec3{a.actualWorld*glm::vec4{contacts[s].points[k],1}};
                    const float gap=a.worldPoints[k].y-height; a.minGap=std::min(a.minGap,gap); a.maxGap=std::max(a.maxGap,gap);
                }
                const auto ids=legJoints(side(s));
                for (unsigned k=0;k<3;++k) a.chain[k]=glm::vec3{root*b.jointFrame(ids[k],globals)[3]};
            }
        }
    }
    std::array<JointId,3> legJoints(LegSide s)
    {
        if (s!=LegSide::Left && s!=LegSide::Right) throw std::invalid_argument("Invalid leg side");
        return s==LegSide::Left ? std::array{JointId::LeftHip,JointId::LeftKnee,JointId::LeftAnkle}:
            std::array{JointId::RightHip,JointId::RightKnee,JointId::RightAnkle};
    }
    const char* supportPhaseName(SupportPhase p)
    {
        switch (p) {
        case SupportPhase::Disabled:return "Disabled"; case SupportPhase::Unaligned:return "Unaligned";
        case SupportPhase::Planning:return "Planning"; case SupportPhase::Aligning:return "Aligning";
        case SupportPhase::Holding:return "Holding"; case SupportPhase::Moving:return "Moving"; case SupportPhase::Lost:return "Lost";
        } return "Invalid";
    }
    std::array<SoleContact,2> calibrateSoleContacts(const BodyRigBinding& b)
    {
        if (!b.profile().name.starts_with("Raphtalia_prepared/")) throw std::invalid_argument("Shoe calibration requires audited Raphtalia profile");
        std::array<SoleContact,2> result;
        for (unsigned s=0;s<2;++s) {
            const auto foot=b.jointNode(s==0 ? JointId::LeftAnkle:JointId::RightAnkle);
            std::vector<glm::vec3> points;
            const auto e=std::find_if(b.profile().effectors.begin(),b.profile().effectors.end(),[&](const auto& f) { return f.id==sole(s); });
            const auto toSole=glm::inverse(e->frameInBodyBind)*b.profile().bodyFromResource;
            float minimum=1e6f;
            for (const auto& mesh:b.asset().meshes()) for (const auto& v:mesh.vertices) {
                float weight=0;
                for (int k=0;k<4;++k) if (v.boneIds[k]>=0) {
                    auto node=b.asset().bones().at(static_cast<std::size_t>(v.boneIds[k])).nodeIndex;
                    while (node!=animation::NoNode && node!=foot) node=b.asset().nodes()[node].parent;
                    if (node==foot) weight+=v.boneWeights[k];
                }
                if (weight>.5f) { const auto p=glm::vec3{toSole*glm::vec4{v.position,1}}; points.push_back(p); minimum=std::min(minimum,p.y); }
            }
            auto& c=result[s]; c.shoeVertices=static_cast<unsigned>(points.size());
            c.sampledMin=glm::vec3{1e6f}; c.sampledMax=glm::vec3{-1e6f};
            for (const auto p:points) if (p.y<=minimum+c.lowBand) {
                c.sampledMin=glm::min(c.sampledMin,p); c.sampledMax=glm::max(c.sampledMax,p); ++c.lowVertices;
            }
            if (c.lowVertices<4) throw std::invalid_argument("Insufficient low shoe samples");
            const auto center=glm::vec3{(c.sampledMin.x+c.sampledMax.x)*.5f,minimum,(c.sampledMin.z+c.sampledMax.z)*.5f};
            c.soleFromContact=glm::translate(glm::mat4{1},center);
            const auto half=(c.sampledMax-c.sampledMin)*.5f;
            c.points={glm::vec3{-half.x,0,-half.z},glm::vec3{half.x,0,-half.z},glm::vec3{half.x,0,half.z},glm::vec3{-half.x,0,half.z}};
        }
        return result;
    }
    LegSolution solveLegPose(const DirectBodyControl& c,LegSide s,const SoleContact& contact,const glm::mat4& root,const glm::mat4& target)
    {
        (void)legJoints(s); rigid(root); rigid(target);
        rigid(contact.soleFromContact);
        for (const auto p:contact.points) if (!finite(p) || glm::length(p)>1 || std::abs(p.y)>1e-6f)
            throw std::invalid_argument("Invalid coplanar metric contact points");
        if (!c.enabled()) throw std::invalid_argument("Leg IK requires direct mode");
        const unsigned si=s==LegSide::Left ? 0u:1u; LegChain chain(c,si,contact);
        const auto goal=glm::inverse(root)*target; auto q=chain.initial;
        auto error=pose_ik::poseError(chain,q,glm::vec3{goal[3]},glm::quat_cast(glm::mat3{goal}));
        LegSolution out;
        for (;out.iterations<80 && !reached(chain,q,goal);++out.iterations)
            if (!pose_ik::iteratePose(chain,glm::vec3{goal[3]},glm::quat_cast(glm::mat3{goal}),q,error)) break;
        for (unsigned k=0;k<6;++k) out.coordinates[k]=static_cast<float>(q[k]);
        DirectBodyControl trial(c); write(trial,si,out.coordinates); animation::Pose p(c.binding().asset().skeleton());
        trial.apply(p.locals); animation::computeGlobalTransforms(c.binding().asset().skeleton(),p.locals,p.globals);
        const auto actual=root*c.binding().effectorFrame(sole(si),p.globals)*contact.soleFromContact;
        out.positionError=glm::length(glm::vec3{actual[3]-target[3]});
        out.angularError=static_cast<float>(glm::length(orientationResidual(glm::quat_cast(glm::mat3{target}),glm::quat_cast(glm::mat3{actual}))));
        out.success=out.positionError<=SupportPositionTolerance && out.angularError<=SupportAngularTolerance;
        out.evaluations=chain.evaluations; out.reason=out.success ? "Solved":"LocalNoConvergenceOrJointLimit"; return out;
    }

    struct LegSupport::Job
    {
        DirectBodyControl trial;
        animation::Pose pose;
        std::array<LegChain,2> chains;
        glm::mat4 root{1}; glm::vec3 start{0},end{0};
        std::array<glm::mat4,2> from{},to{};
        std::array<std::array<double,6>,2> q{};
        std::vector<Sample> samples;
        unsigned sample=1,leg=0,legIterations=0,check=0;
        bool validating=false, alignment=false;
        explicit Job(DirectBodyControl& c,const std::array<SoleContact,2>& contacts):trial(c),pose(c.binding().asset().skeleton()),
            chains{LegChain(c,0,contacts[0]),LegChain(c,1,contacts[1])} {}
        float progress() const { const double t=std::min(sample,SupportMotionTicks)/static_cast<double>(SupportMotionTicks); return static_cast<float>(t*t*t*(10+t*(-15+6*t))); }
    };
    std::span<const ColliderCalibration> lowerBodyColliderCalibration()
    {
        // Metres, proper semantic frames. Trimmed bone axes avoid joint bulges;
        // these small proxies do not enclose the skirt or all skin vertices.
        static const std::array<ColliderCalibration,5> values{{
            {11,SegmentId::Pelvis,JointId::Pelvis,JointId::Pelvis,0,0,.060f,{-.040f,-.010f,0},{.040f,-.010f,0}},
            {12,SegmentId::LeftThigh,JointId::LeftHip,JointId::LeftKnee,.10f,.90f,.045f},
            {13,SegmentId::LeftShank,JointId::LeftKnee,JointId::LeftAnkle,.10f,.90f,.035f},
            {15,SegmentId::RightThigh,JointId::RightHip,JointId::RightKnee,.10f,.90f,.045f},
            {16,SegmentId::RightShank,JointId::RightKnee,JointId::RightAnkle,.10f,.90f,.035f}
        }};
        return values;
    }
    std::vector<BodyCollider> lowerBodyColliders(const BodyRigBinding& b,std::span<const glm::mat4> globals,
        const glm::mat4& root,const std::array<SoleContact,2>& contacts)
    {
        std::vector<BodyCollider> result; result.reserve(7);
        for (const auto& c:lowerBodyColliderCalibration()) {
            const auto f=b.jointFrame(c.first,globals),g=b.jointFrame(c.second,globals);
            const auto a=c.first==c.second ? glm::vec3{f*glm::vec4{c.localA,1}}:glm::mix(glm::vec3{f[3]},glm::vec3{g[3]},c.from);
            const auto z=c.first==c.second ? glm::vec3{f*glm::vec4{c.localB,1}}:glm::mix(glm::vec3{f[3]},glm::vec3{g[3]},c.to);
            result.push_back({c.id,c.segment,{glm::dvec3{root*glm::vec4{a,1}},glm::dvec3{root*glm::vec4{z,1}},c.radius}});
        }
        for (unsigned side=0;side<2;++side) {
            const auto f=root*b.effectorFrame(side==0 ? EffectorId::LeftSole:EffectorId::RightSole,globals)*contacts[side].soleFromContact;
            double radius=.0;
            for (const auto point:contacts[side].points) radius=std::max(radius,static_cast<double>(glm::length(point)));
            radius+=.002; // Measured contact rectangle circumradius + explicit 2 mm envelope.
            const auto center=glm::dvec3{f*glm::vec4{0,static_cast<float>(radius),0,1}};
            const auto ankle=glm::dvec3{(root*b.jointFrame(side==0 ? JointId::LeftAnkle:JointId::RightAnkle,globals))[3]};
            result.push_back({side==0 ? 14u:17u,side==0 ? SegmentId::LeftFoot:SegmentId::RightFoot,{center,ankle,radius}});
        }
        return result;
    }
    LegSupport::LegSupport(DirectBodyControl& c):control_(c),contacts_(calibrateSoleContacts(c.binding())) {}
    LegSupport::~LegSupport()=default;
    void LegSupport::beginTick()
    { observation_.tickIterations=0; observation_.tickEvaluations=0; observation_.tickFkChecks=0; observation_.tickPlanningMicroseconds=0; }
    void LegSupport::setSurface(SupportSurface s)
    {
        if (!s.id || !s.version || !std::isfinite(s.height) || std::abs(s.height)>100 || s.version<=observation_.surface.version)
            throw std::invalid_argument("Support surface requires increasing version, finite height, nonzero ID");
        observation_.surface=s;
        if (enabled()) {
            job_.reset(); lost_=true; observation_.kind=SupportKind::None;
            for (auto& a:observation_.anchors) a.bound=false;
            if (motion_.empty()) observation_.phase=SupportPhase::Lost;
            observation_.result="SupportLost"; observation_.reason="Surface changed/deleted; finish saved segment against historical anchors, then rest";
        }
    }
    void LegSupport::enable(const glm::mat4& root)
    {
        rigid(root);
        if (!control_.enabled() || !motion_.empty() || job_ || (enabled() && !lost_) || !observation_.surface.available)
            throw std::invalid_argument("Support enable requires idle direct pose and available surface; disable before recapture");
        auto proposed=observation_;
        proposed.enabled=true; proposed.phase=SupportPhase::Unaligned; proposed.kind=SupportKind::None;
        proposed.result="Unaligned"; proposed.reason.clear();
        proposed.cancellationPending=false; proposed.disablePending=false;
        animation::Pose p(control_.binding().asset().skeleton()); control_.apply(p.locals);
        animation::computeGlobalTransforms(control_.binding().asset().skeleton(),p.locals,p.globals);
        for (unsigned s=0;s<2;++s) {
            auto& a=proposed.anchors[s]; a={}; a.surface=proposed.surface.id; a.version=proposed.surface.version; a.surfaceHeight=proposed.surface.height;
            auto f=root*control_.binding().effectorFrame(sole(s),p.globals)*contacts_[s].soleFromContact;
            glm::vec3 z{f[2].x,0,f[2].z};
            if (glm::length(z)<.1f) throw std::invalid_argument("Sole forward projection degenerate");
            z=glm::normalize(z); f[0]=glm::vec4{glm::cross(glm::vec3{0,1,0},z),0}; f[1]={0,1,0,0}; f[2]=glm::vec4{z,0};
            f[3].y=observation_.surface.height; a.desiredWorld=f;
            a.surfaceFromSole=glm::translate(glm::mat4{1},glm::vec3{0,-observation_.surface.height,0})*f;
        }
        observation_=std::move(proposed); lost_=false; origin_=glm::vec3{root[3]};
        ++observation_.fkChecks; ++observation_.tickFkChecks;
        measure(p.globals,root);
    }
    void LegSupport::plan(std::uint64_t action,std::uint64_t epoch,const glm::mat4& root,std::optional<glm::vec3> position)
    {
        const auto setupStart=std::chrono::steady_clock::now();
        observation_.attempt=action;
        const auto reject=[&](const char* why) { observation_.reason=why; observation_.result="Rejected"; throw std::invalid_argument(why); };
        if (!enabled() || lost_ || job_ || !motion_.empty()) reject("Support planning requires enabled, available, resting state");
        if (position && observation_.kind!=SupportKind::KinematicSupport) reject("Align explicitly before root movement");
        if (!position && observation_.kind==SupportKind::KinematicSupport) reject("Already aligned; disable before recapture");
        rigid(root);
        const auto end=position.value_or(glm::vec3{root[3]});
        if (!finite(end) || glm::length(end)>100 || glm::length(end-glm::vec3{root[3]})>.025f || glm::length(end-origin_)>.04f)
            reject("Root target outside finite 25 mm command / 40 mm stance envelope (policy, not global infeasibility)");
        if (!position) for (const auto& a:observation_.anchors) if (a.minGap < -SupportSurfaceTolerance)
            reject("Initial contact proxy penetrates floor; correct pose/floor explicitly before alignment");
        auto j=std::make_unique<Job>(control_,contacts_); j->root=root; j->start=glm::vec3{root[3]}; j->end=end; j->alignment=!position;
        Sample first; first.root=j->start;
        for (unsigned s=0;s<2;++s) {
            first.q[s]=coordinates(control_,s); for (unsigned k=0;k<6;++k) j->q[s][k]=first.q[s][k];
            j->from[s]=observation_.anchors[s].actualWorld; j->to[s]=observation_.anchors[s].desiredWorld;
            if (!position && glm::length(glm::vec3{j->from[s][3]-j->to[s][3]})>.03f) reject("Explicit alignment limited to 30 mm per sole");
        }
        j->samples.reserve(SupportMotionTicks+3); j->samples.push_back(first);
        job_=std::move(j); observation_.phase=SupportPhase::Planning; observation_.action=action; observation_.epoch=epoch;
        observation_.requestedRoot=end; observation_.result="Planning"; observation_.reason.clear();
        observation_.iterations=observation_.evaluations=observation_.fkChecks=observation_.plannedStates=observation_.executedStates=0;
        const double setup=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-setupStart).count();
        observation_.planningMicroseconds=setup; observation_.tickPlanningMicroseconds+=setup; observation_.cancellationPending=false;
        observation_.fkChecks=2; observation_.tickFkChecks+=2; // Two frozen proximal-frame FK preparations.
    }
    void LegSupport::fail(const char* reason)
    {
        observation_.result="PlanRejected"; observation_.reason=reason;
        observation_.phase=observation_.kind==SupportKind::KinematicSupport ? SupportPhase::Holding:SupportPhase::Unaligned;
        job_.reset();
    }
    void LegSupport::cancel(bool disable)
    {
        if (!enabled()) throw std::invalid_argument("Support mode is disabled");
        job_.reset(); observation_.cancellationPending=!motion_.empty(); observation_.disablePending=disable;
        if (!motion_.empty()) { observation_.result=disable ? "DisablePending":"CancelPending"; return; }
        if (disable) interrupt("Support disabled at rest");
        else { observation_.phase=lost_ ? SupportPhase::Lost:observation_.kind==SupportKind::KinematicSupport ? SupportPhase::Holding:SupportPhase::Unaligned; observation_.result="Cancelled"; }
    }
    void LegSupport::interrupt(const char* reason)
    {
        job_.reset(); motion_.clear(); cursor_=0; lost_=false; observation_.enabled=false;
        observation_.phase=SupportPhase::Disabled; observation_.kind=SupportKind::None;
        observation_.cancellationPending=observation_.disablePending=false;
        for (auto& a:observation_.anchors) a.bound=false;
        observation_.result="Interrupted"; observation_.reason=reason;
    }
    void LegSupport::advance(glm::vec3& rootPosition)
    {
        if (!motion_.empty()) {
            const auto& s=motion_.at(cursor_++); rootPosition=s.root;
            for (unsigned leg=0;leg<2;++leg) write(control_,leg,s.q[leg]);
            observation_.executedStates=static_cast<unsigned>(cursor_);
            if (cursor_==motion_.size()) {
                motion_.clear(); cursor_=0;
                if (!lost_) {
                    observation_.kind=SupportKind::KinematicSupport;
                    for (auto& a:observation_.anchors) a.bound=true;
                }
                observation_.phase=lost_ ? SupportPhase::Lost:SupportPhase::Holding;
                observation_.result=lost_ ? "SupportLost":observation_.cancellationPending ? "Cancelled":"Succeeded";
                observation_.cancellationPending=false;
                if (observation_.disablePending) interrupt("Support disabled after saved segment reached rest");
            }
            return;
        }
        if (!job_) return;
        const auto start=std::chrono::steady_clock::now(); auto& j=*job_;
        unsigned iterations=0,checks=0;
        const unsigned checkBudget=4-std::min(4u,observation_.tickFkChecks);
        while (job_ && iterations<4 && checks<checkBudget) {
            if (!j.validating) {
                const float u=j.progress(); auto root=j.root; root[3]=glm::vec4{glm::mix(j.start,j.end,u),1};
                const auto target=j.alignment ? mixFrame(j.from[j.leg],j.to[j.leg],u):j.to[j.leg];
                const auto goal=glm::inverse(root)*target; auto& chain=j.chains[j.leg]; auto& q=j.q[j.leg];
                const unsigned before=chain.evaluations;
                bool done=reached(chain,q,goal);
                if (!done) {
                    auto error=pose_ik::poseError(chain,q,glm::vec3{goal[3]},glm::quat_cast(glm::mat3{goal}));
                    const bool improved=pose_ik::iteratePose(chain,glm::vec3{goal[3]},glm::quat_cast(glm::mat3{goal}),q,error);
                    ++iterations; ++j.legIterations;
                    done=reached(chain,q,goal);
                    // Float metric can stagnate below strict convergence. The full FK
                    // acceptance below remains authoritative, including motion derivatives.
                    if (!improved || j.legIterations>=80) done=true;
                }
                observation_.evaluations+=chain.evaluations-before; observation_.tickEvaluations+=chain.evaluations-before;
                if (done) { ++j.leg; j.legIterations=0; }
                if (j.leg==2) {
                    Sample s; s.root=glm::vec3{root[3]};
                    for (unsigned leg=0;leg<2;++leg) for (unsigned k=0;k<6;++k) s.q[leg][k]=static_cast<float>(j.q[leg][k]);
                    const auto& prev=j.samples.back(); const auto& prev2=j.samples.size()>1 ? j.samples[j.samples.size()-2]:prev;
                    const auto dr=glm::dvec3{s.root}-glm::dvec3{prev.root};
                    const auto dr0=glm::dvec3{prev.root}-glm::dvec3{prev2.root};
                    if (glm::length(dr)/dt>SupportRootSpeed || glm::length(dr-dr0)/(dt*dt)>SupportRootAcceleration) { fail("Root velocity/acceleration check failed"); break; }
                    bool valid=true;
                    for (unsigned leg=0;leg<2;++leg) for (unsigned k=0;k<6;++k) {
                        const double v=(static_cast<double>(s.q[leg][k])-prev.q[leg][k])/dt;
                        const double v0=(static_cast<double>(prev.q[leg][k])-prev2.q[leg][k])/dt;
                        if (std::abs(v)>SupportJointSpeed || std::abs(v-v0)/dt>SupportJointAcceleration) valid=false;
                    }
                    if (!valid) { fail("Leg velocity/acceleration check failed; saved state unchanged"); break; }
                    j.samples.push_back(s); j.validating=true; j.check=0;
                }
            } else {
                // 9 FK probes plus a Lipschitz enclosure certify every display alpha.
                // R includes BOTH links, ankle offset and the furthest contact corner.
                const float a=static_cast<float>(j.check)/8;
                const auto& lo=j.samples[j.samples.size()-2]; const auto& hi=j.samples.back();
                auto root=j.root; root[3]=glm::vec4{glm::mix(lo.root,hi.root,a),1};
                for (unsigned leg=0;leg<2;++leg) {
                    LegCoordinates q; for (unsigned k=0;k<6;++k) q[k]=glm::mix(lo.q[leg][k],hi.q[leg][k],a);
                    write(j.trial,leg,q);
                }
                j.trial.apply(j.pose.locals); animation::computeGlobalTransforms(control_.binding().asset().skeleton(),j.pose.locals,j.pose.globals);
                ++checks; ++observation_.fkChecks;
                auto obs=observation_;
                // Desired alignment interpolation is linear between its stored tick goals.
                const auto progress=[](unsigned sample) { const double x=std::min(sample,SupportMotionTicks)/static_cast<double>(SupportMotionTicks); return static_cast<float>(x*x*x*(10+x*(-15+6*x))); };
                for (unsigned leg=0;leg<2;++leg) if (j.alignment)
                    obs.anchors[leg].desiredWorld=mixFrame(mixFrame(j.from[leg],j.to[leg],progress(j.sample-1)),mixFrame(j.from[leg],j.to[leg],progress(j.sample)),a);
                measureFeet(obs,control_.binding(),contacts_,j.pose.globals,root);
                bool valid=true;
                for (unsigned leg=0;leg<2;++leg) {
                    double sum=0; for (unsigned k=0;k<6;++k) sum+=std::abs(static_cast<double>(hi.q[leg][k])-lo.q[leg][k]);
                    const auto from=mixFrame(j.from[leg],j.to[leg],progress(j.sample-1));
                    const auto to=mixFrame(j.from[leg],j.to[leg],progress(j.sample));
                    const double goalTravel=j.alignment ? glm::length(glm::dvec3{to[3]-from[3]}):0;
                    const double goalAngle=j.alignment ? glm::length(orientationResidual(glm::quat_cast(glm::mat3{to}),glm::quat_cast(glm::mat3{from}))):0;
                    const double pointBound=(glm::length(glm::dvec3{hi.root}-glm::dvec3{lo.root})+j.chains[leg].radius*sum+goalTravel)/16+numericSlack;
                    const double angularBound=(sum+goalAngle)/16+numericSlack;
                    const auto& foot=obs.anchors[leg];
                    // Alignment is explicitly unbound. Only endpoint residuals are
                    // promised there; plane enclosure applies once both anchors bind.
                    if (foot.positionError+pointBound>SupportPositionTolerance || foot.angularError+angularBound>SupportAngularTolerance) valid=false;
                    if (foot.minGap-pointBound < -SupportSurfaceTolerance) valid=false;
                    if ((!j.alignment || j.sample>=SupportMotionTicks) && foot.maxGap+pointBound>SupportSurfaceTolerance) valid=false;
                }
                if (!valid) { fail("Full float FK / all-alpha sole enclosure failed (local plan rejected)"); break; }
                if (++j.check==9) {
                    observation_.plannedStates=static_cast<unsigned>(j.samples.size());
                    if (j.sample==SupportMotionTicks+2) {
                        // Both terminal hold ticks were differentiated and checked.
                        aligning_=j.alignment; motion_=std::move(j.samples); cursor_=1;
                        observation_.phase=aligning_ ? SupportPhase::Aligning:SupportPhase::Moving; observation_.result="Accepted";
                        job_.reset(); break;
                    }
                    ++j.sample; j.leg=0; j.validating=false;
                }
            }
        }
        observation_.iterations+=iterations; observation_.tickIterations=iterations; observation_.tickFkChecks+=checks;
        const double elapsed=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
        observation_.tickPlanningMicroseconds+=elapsed; observation_.planningMicroseconds+=elapsed;
        observation_.storageBytes=motion_.capacity()*sizeof(Sample)+(job_ ? sizeof(Job)+job_->samples.capacity()*sizeof(Sample)+
            (job_->pose.locals.capacity()+job_->pose.globals.capacity()+job_->pose.palette.capacity())*sizeof(glm::mat4)+
            job_->trial.coordinates().size()*sizeof(glm::vec3):0);
    }
    void LegSupport::measure(std::span<const glm::mat4> globals,const glm::mat4& root)
    { if (enabled()) measureFeet(observation_,control_.binding(),contacts_,globals,root); }
    SupportObservation LegSupport::displayObservation(SupportObservation o,std::span<const glm::mat4> globals,const glm::mat4& root) const
    { if (o.enabled) measureFeet(o,control_.binding(),contacts_,globals,root); return o; }
}
