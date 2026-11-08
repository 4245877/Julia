#include "julia/embodiment/ArmReach.hpp"
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
        constexpr double dt = runtime::BodyStepSeconds;
        std::size_t index(ArmSide side)
        {
            if (side != ArmSide::Left && side != ArmSide::Right) throw std::invalid_argument("Invalid arm");
            return side == ArmSide::Left ? 0 : 1;
        }
        bool active(ReachState s) { return s == ReachState::Moving || s == ReachState::Settling; }
        std::array<JointId,3> joints(std::size_t s)
        { return s == 0 ? std::array{JointId::LeftShoulder,JointId::LeftElbow,JointId::LeftWrist} :
            std::array{JointId::RightShoulder,JointId::RightElbow,JointId::RightWrist}; }
        constexpr std::array<int,6> jointSlot{0,0,0,1,2,2}, coordinateSlot{0,1,2,0,0,1};
        bool finite(glm::vec3 v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }
        void validate(const ReachTarget& t)
        {
            const auto s = index(t.arm);
            const bool effector = s == 0 ? t.effector == EffectorId::LeftWrist || t.effector == EffectorId::LeftPalm || t.effector == EffectorId::LeftGrip :
                t.effector == EffectorId::RightWrist || t.effector == EffectorId::RightPalm || t.effector == EffectorId::RightGrip;
            if (!effector || !finite(t.position) || glm::length(t.position)>10000 || (t.space != TargetSpace::Body && t.space != TargetSpace::World) ||
                !std::isfinite(t.tolerance) || t.tolerance < .0005f || t.tolerance > .02f ||
                !std::isfinite(t.maxSpeed) || t.maxSpeed < .1f || t.maxSpeed > 4 ||
                !std::isfinite(t.maxAcceleration) || t.maxAcceleration < .2f || t.maxAcceleration > 20 ||
                !std::isfinite(t.settleSeconds) || t.settleSeconds < .05f || t.settleSeconds > 2)
                throw std::invalid_argument("Invalid position reach target/limits (orientation is unsupported)");
        }
        struct Chain
        {
            std::array<const JointDefinition*,3> definitions;
            std::array<double,6> initial{}, lower{}, upper{};
            glm::mat4 base{1}, upperToElbow{1}, elbowToWrist{1}, wristToEffector{1};
            float length1 = 0, length2 = 0, offset = 0;
            explicit Chain(const DirectBodyControl& c, const ReachTarget& t)
            {
                const auto& b = c.binding();
                const auto ids = joints(index(t.arm));
                for (std::size_t j=0;j<3;++j) definitions[j] = &b.profile().joints[b.jointIndex(ids[j])];
                for (std::size_t k=0;k<6;++k)
                {
                    const auto j = static_cast<std::size_t>(jointSlot[k]), d = static_cast<std::size_t>(coordinateSlot[k]);
                    const auto& coord = definitions[j]->coordinates.at(d);
                    initial[k] = c.coordinate(ids[j],d); lower[k] = coord.lower; upper[k] = coord.upper;
                }
                // Frozen torso/clavicle are sampled once. Subsequent trials use just 3 rigid frames.
                animation::Pose scratch(b.asset().skeleton());
                c.apply(scratch.locals);
                animation::computeGlobalTransforms(b.asset().skeleton(),scratch.locals,scratch.globals);
                base = b.jointFrame(ids[0],scratch.globals) * glm::inverse(rotation(0,initial));
                const auto f0 = definitions[0]->frameInBodyBind, f1 = definitions[1]->frameInBodyBind,
                    f2 = definitions[2]->frameInBodyBind;
                upperToElbow = glm::inverse(f0)*f1; elbowToWrist = glm::inverse(f1)*f2;
                for (const auto& e : b.profile().effectors)
                    if (e.id == t.effector) wristToEffector = glm::inverse(f2)*e.frameInBodyBind;
                length1 = glm::length(glm::vec3{upperToElbow[3]});
                length2 = glm::length(glm::vec3{elbowToWrist[3]});
                offset = glm::length(glm::vec3{wristToEffector[3]});
            }
            glm::mat4 rotation(int joint, const std::array<double,6>& q) const
            {
                glm::mat4 r{1};
                for (std::size_t k=0;k<6;++k) if (jointSlot[k] == joint)
                    r = glm::rotate(r,static_cast<float>(q[k]),definitions[static_cast<std::size_t>(joint)]->coordinates[static_cast<std::size_t>(coordinateSlot[k])].axis);
                return r;
            }
            glm::vec3 position(const std::array<double,6>& q) const
            { return glm::vec3{(base*rotation(0,q)*upperToElbow*rotation(1,q)*elbowToWrist*rotation(2,q)*wristToEffector)[3]}; }
        };
        double polynomial(const std::array<double,6>& c, double u)
        { return ((((c[5]*u+c[4])*u+c[3])*u+c[2])*u+c[1])*u+c[0]; }
    }
    const char* reachStateName(ReachState s)
    {
        switch (s)
        {
        case ReachState::Idle: return "idle";
        case ReachState::Moving: return "moving";
        case ReachState::Settling: return "settling";
        case ReachState::Succeeded: return "succeeded";
        case ReachState::Failed: return "failed";
        case ReachState::Cancelled: return "cancelled";
        }
        return "unknown";
    }
    const char* reachReasonName(ReachReason r)
    {
        switch (r)
        {
        case ReachReason::None: return "none";
        case ReachReason::Reached: return "reached";
        case ReachReason::GeometricRange: return "outside geometric shell";
        case ReachReason::JointLimited: return "joint-limited local minimum (not global proof)";
        case ReachReason::LocalNoConvergence: return "local solver did not converge";
        case ReachReason::MotionPlanUnavailable: return "no bounded motion segment within 50 seconds";
        case ReachReason::UserCancelled: return "user cancelled; held after smooth stop";
        case ReachReason::Superseded: return "superseded";
        case ReachReason::ModeChanged: return "mode changed";
        case ReachReason::CoordinatesReset: return "coordinates reset";
        case ReachReason::BodyReset: return "body reset";
        case ReachReason::RootChanged: return "root teleported";
        }
        return "unknown";
    }
    ArmSolution solveArmPosition(const DirectBodyControl& control, const ReachTarget& target, glm::vec3 goal)
    {
        validate(target);
        if (!control.enabled() || !finite(goal)) throw std::invalid_argument("Reach requires calibrated direct mode and finite body goal");
        Chain chain(control,target);
        const float distance = glm::length(goal-glm::vec3{chain.base[3]});
        const bool geometric = distance > chain.length1+chain.length2+chain.offset+target.tolerance ||
            distance < std::max(0.0f,std::abs(chain.length1-chain.length2)-chain.offset)-target.tolerance;
        const bool wrist = target.effector == EffectorId::LeftWrist || target.effector == EffectorId::RightWrist;
        const std::size_t dimensions = wrist ? 4 : 6;
        ArmSolution best; best.coordinates = chain.initial;
        best.error = glm::length(chain.position(best.coordinates)-goal);
        // Previous valid pose is ALWAYS first. Deterministic bounded restarts only on local failure.
        for (unsigned seed=0;seed<9 && best.error > target.tolerance*.2f;++seed)
        {
            auto q = chain.initial;
            if (seed)
                for (std::size_t k=0;k<dimensions;++k)
                {
                    const double fraction = seed == 1 ? .5 : ((seed*(k*2+3))%11+1)/12.0;
                    q[k] = chain.lower[k]+fraction*(chain.upper[k]-chain.lower[k]);
                }
            float error = glm::length(chain.position(q)-goal);
            for (unsigned iteration=0;iteration<160;++iteration)
            {
                ++best.iterations;
                if (error < best.error) { best.error = error; best.coordinates = q; }
                if (error <= target.tolerance*.2f) break;
                const auto p = chain.position(q), residual = goal-p;
                std::array<glm::vec3,6> jacobian{};
                glm::mat3 normal{0};
                constexpr double h = .001;
                for (std::size_t k=0;k<dimensions;++k)
                {
                    auto a=q,b=q; a[k]=std::max(q[k]-h,chain.lower[k]); b[k]=std::min(q[k]+h,chain.upper[k]);
                    if (b[k]==a[k]) continue; // A locked coordinate contributes no Jacobian column.
                    jacobian[k]=(chain.position(b)-chain.position(a))/static_cast<float>(b[k]-a[k]);
                    // Active-set projection: do not let an outward-bound column consume the step.
                    const float gradient=glm::dot(jacobian[k],residual);
                    if ((q[k]<=chain.lower[k]+1e-7 && gradient<0) || (q[k]>=chain.upper[k]-1e-7 && gradient>0)) jacobian[k]={0,0,0};
                    normal += glm::outerProduct(jacobian[k],jacobian[k]);
                }
                normal += glm::mat3{.000025f}; // 5 mm/rad damping: finite at extension/degenerate directions.
                const auto dual=glm::inverse(normal)*residual;
                std::array<double,6> delta{};
                double largest=.15;
                for (std::size_t k=0;k<dimensions;++k) { delta[k]=glm::dot(jacobian[k],dual); largest=std::max(largest,std::abs(delta[k])); }
                bool improved=false;
                for (double step= .15/largest;step>=.0001;step*=.5)
                {
                    auto candidate=q;
                    for (std::size_t k=0;k<dimensions;++k) candidate[k]=std::clamp(q[k]+step*delta[k],chain.lower[k],chain.upper[k]);
                    const float e=glm::length(chain.position(candidate)-goal);
                    if (e < error-1e-8f) { q=candidate; error=e; improved=true; break; }
                }
                if (!improved) break;
            }
            if (error < best.error) { best.error=error; best.coordinates=q; }
        }
        bool atLimit=false;
        for (std::size_t k=0;k<dimensions;++k)
            atLimit |= best.coordinates[k]-chain.lower[k]<.002 || chain.upper[k]-best.coordinates[k]<.002;
        best.reason = best.error <= target.tolerance*.5f ? ReachReason::Reached : geometric ? ReachReason::GeometricRange :
            atLimit ? ReachReason::JointLimited : ReachReason::LocalNoConvergence;
        return best;
    }
    ArmReach::ArmReach(DirectBodyControl& c) : control_(c) { events_.reserve(8); }
    void ArmReach::finish(std::size_t s, ReachState state, ReachReason reason)
    {
        auto& o=observations_[s]; o.state=state; o.reason=reason;
        o.cancellationPending=false; o.replanningDeferred=false; events_.push_back(o);
    }
    bool ArmReach::owns(JointId joint) const
    {
        for (std::size_t s=0;s<2;++s) if (active(observations_[s].state))
        {
            const auto ids=joints(s);
            if (std::find(ids.begin(),ids.end(),joint)!=ids.end() || joint==(s==0 ? JointId::LeftClavicle:JointId::RightClavicle)) return true;
            // Ancestors must stay frozen while either arm owns a trajectory.
            if (joint<=JointId::Chest) return true;
        }
        return false;
    }
    void ArmReach::interrupt(ReachReason reason)
    { for (std::size_t s=0;s<2;++s) if (active(observations_[s].state)) { finish(s,ReachState::Cancelled,reason); motion_[s]={}; } }
    void ArmReach::reset() { interrupt(ReachReason::BodyReset); observations_={}; motion_={}; }
    void ArmReach::start(std::uint64_t action, std::uint64_t epoch, const ReachTarget& target,
        const glm::mat4& world, bool retarget)
    {
        validate(target);
        if (!control_.enabled()) throw std::invalid_argument("IK requires explicit calibrated direct mode; clip pose cannot be inferred");
        const auto s=index(target.arm);
        auto& o=observations_[s]; auto& m=motion_[s];
        if (retarget && (o.action!=action || o.epoch!=epoch || !active(o.state) || m.cancelling))
            throw std::invalid_argument("No matching active reach action");
        if (active(o.state) && (target.maxSpeed!=o.target.maxSpeed || target.maxAcceleration!=o.target.maxAcceleration))
            throw std::invalid_argument("Retarget/replacement must preserve active motion limits");
        const auto goal=target.space==TargetSpace::World ? glm::vec3{glm::inverse(world)*glm::vec4{target.position,1}} : target.position;
        if (!finite(goal) || glm::length(goal)>10000) throw std::invalid_argument("Invalid transformed body target");
        const auto begin=std::chrono::steady_clock::now();
        const auto solution=solveArmPosition(control_,target,goal);
        const auto microseconds=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-begin).count();
        if (!retarget && active(o.state)) finish(s,ReachState::Cancelled,ReachReason::Superseded);
        if (!retarget) { o.maxMeasuredSpeed=0; o.maxMeasuredAcceleration=0; }
        if (!active(o.state))
        {
            // A replacement of an active action retains q/v/a below.
            if (m.duration==0 || m.tick>=m.duration) m={};
        }
        const auto ids=joints(s);
        const Chain reference(control_,target);
        m.frames={reference.base,reference.upperToElbow,reference.elbowToWrist,reference.wristToEffector};
        for (std::size_t k=0;k<6;++k) m.q[k]=control_.coordinate(ids[static_cast<std::size_t>(jointSlot[k])],static_cast<std::size_t>(coordinateSlot[k]));
        o.action=action; o.epoch=epoch; o.target=target; o.goalBody=goal; o.state=ReachState::Moving; o.reason=ReachReason::None;
        o.cancellationPending=false;
        o.referenceBody=reference.position(m.q);
        o.iterations=solution.iterations; o.solveMicroseconds=microseconds;
        m.solution=solution; m.settled=0; m.pending=true; m.cancelling=false;
        // A rare unsafe reversal near a bound keeps the valid old segment until rest, then replans.
        if (plan(s)) m.pending=false;
        o.replanningDeferred=m.pending;
    }
    bool ArmReach::plan(std::size_t s)
    {
        auto& m=motion_[s]; const auto& o=observations_[s]; const auto ids=joints(s);
        for (unsigned ticks=30;ticks<=6000;ticks+=12)
        {
            const double seconds=ticks*dt;
            decltype(m.polynomial) candidate{};
            for (std::size_t k=0;k<6;++k)
            {
                const double p=m.q[k], v=m.velocity[k]*seconds, a=m.acceleration[k]*seconds*seconds;
                const double d=m.solution.coordinates[k]-p;
                candidate[k]={p,v,a*.5,10*d-6*v-1.5*a,-15*d+8*v+1.5*a,6*d-3*v-.5*a};
            }
            bool valid=true;
            auto last=m.q, previousVelocity=m.lastStepVelocity;
            for (unsigned t=1;t<=ticks+2 && valid;++t)
                for (std::size_t k=0;k<6;++k)
                {
                    const double q=polynomial(candidate[k],std::min(1.0,static_cast<double>(t)/ticks));
                    const double v=(q-last[k])/dt, a=(v-previousVelocity[k])/dt;
                    const auto& limit=control_.binding().profile().joints[control_.binding().jointIndex(ids[static_cast<std::size_t>(jointSlot[k])])].coordinates[static_cast<std::size_t>(coordinateSlot[k])];
                    if (q<limit.lower-1e-9 || q>limit.upper+1e-9 || std::abs(v)>o.target.maxSpeed*.995 || std::abs(a)>o.target.maxAcceleration*.99) { valid=false; break; }
                    last[k]=q; previousVelocity[k]=v;
                }
            if (valid) { m.polynomial=candidate; m.duration=ticks; m.tick=0; return true; }
        }
        return false;
    }
    void ArmReach::cancel(ArmSide arm, std::uint64_t action)
    {
        const auto s=index(arm);
        if (observations_[s].action!=action || !active(observations_[s].state)) throw std::invalid_argument("No matching active reach action");
        // Finish the already bounded segment at zero velocity/acceleration, then hold q.
        motion_[s].cancelling=true; motion_[s].pending=false;
        observations_[s].cancellationPending=true; observations_[s].replanningDeferred=false;
    }
    void ArmReach::advance()
    {
        for (std::size_t s=0;s<2;++s)
        {
            auto& m=motion_[s]; auto& o=observations_[s];
            if (!active(o.state)) continue;
            if (m.tick>=m.duration)
            {
                if (m.pending) { if (plan(s)) { m.pending=false; o.replanningDeferred=false; } else { finish(s,ReachState::Failed,ReachReason::MotionPlanUnavailable); continue; } }
                else { m.lastStepVelocity={}; o.state=ReachState::Settling; continue; }
            }
            ++m.tick;
            const double u=static_cast<double>(m.tick)/m.duration, seconds=m.duration*dt;
            const auto ids=joints(s);
            for (std::size_t k=0;k<6;++k)
            {
                const auto& c=m.polynomial[k];
                const double q=polynomial(c,u), v=(q-m.q[k])/dt, a=(v-m.lastStepVelocity[k])/dt;
                o.maxMeasuredSpeed=std::max(o.maxMeasuredSpeed,static_cast<float>(std::abs(v)));
                o.maxMeasuredAcceleration=std::max(o.maxMeasuredAcceleration,static_cast<float>(std::abs(a)));
                m.q[k]=q; m.lastStepVelocity[k]=v;
                m.velocity[k]=(c[1]+u*(2*c[2]+u*(3*c[3]+u*(4*c[4]+u*5*c[5]))))/seconds;
                m.acceleration[k]=(2*c[2]+u*(6*c[3]+u*(12*c[4]+u*20*c[5])))/(seconds*seconds);
                control_.setCoordinate(ids[static_cast<std::size_t>(jointSlot[k])],static_cast<std::size_t>(coordinateSlot[k]),static_cast<float>(q),LimitPolicy::Clamp);
            }
            if (m.tick==m.duration) { m.velocity={}; m.acceleration={}; }
            glm::mat4 reference=m.frames[0];
            for (std::size_t j=0;j<3;++j)
            {
                const auto& definition=control_.binding().profile().joints[control_.binding().jointIndex(ids[j])];
                for (std::size_t k=0;k<6;++k) if (jointSlot[k]==static_cast<int>(j))
                    reference=glm::rotate(reference,static_cast<float>(m.q[k]),definition.coordinates[static_cast<std::size_t>(coordinateSlot[k])].axis);
                reference*=m.frames[j+1];
            }
            o.referenceBody=glm::vec3{reference[3]};
        }
    }
    void ArmReach::measure(std::span<const glm::mat4> globals, const glm::mat4& world)
    {
        const auto& b=control_.binding();
        for (std::size_t s=0;s<2;++s)
        {
            auto& o=observations_[s]; auto& m=motion_[s]; if (!o.action) continue;
            if (o.target.space==TargetSpace::World)
                o.goalBody=glm::vec3{glm::inverse(world)*glm::vec4{o.target.position,1}};
            const auto ids=joints(s);
            o.actualBody=glm::vec3{b.effectorFrame(o.target.effector,globals)[3]};
            o.actualWorld=glm::vec3{world*glm::vec4{o.actualBody,1}};
            o.shoulderBody=glm::vec3{b.jointFrame(ids[0],globals)[3]}; o.elbowBody=glm::vec3{b.jointFrame(ids[1],globals)[3]};
            const auto wrist=glm::vec3{b.jointFrame(ids[2],globals)[3]};
            const auto normal=glm::cross(o.elbowBody-o.shoulderBody,wrist-o.elbowBody);
            // At extension the last valid plane is retained; no normalization of a zero vector.
            if (glm::length(normal)>1e-5f)
            {
                const auto n=glm::normalize(normal);
                o.bendNormalBody=glm::dot(n,o.bendNormalBody)<0 ? -n:n; // A plane has two equivalent normals.
            }
            o.error=glm::length(o.goalBody-o.actualBody);
            if (o.state==ReachState::Settling)
            {
                ++m.settled;
                if (m.settled>=static_cast<unsigned>(std::ceil(o.target.settleSeconds/dt)))
                    finish(s,m.cancelling ? ReachState::Cancelled : o.error<=o.target.tolerance ? ReachState::Succeeded:ReachState::Failed,
                        m.cancelling ? ReachReason::UserCancelled : o.error<=o.target.tolerance ? ReachReason::Reached :
                        m.solution.reason==ReachReason::Reached ? ReachReason::LocalNoConvergence:m.solution.reason);
            }
        }
    }
}
