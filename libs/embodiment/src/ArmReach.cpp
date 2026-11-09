#include "julia/embodiment/ArmReach.hpp"
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
        constexpr double dt = runtime::BodyStepSeconds;
        std::size_t index(ArmSide side)
        {
            if (side != ArmSide::Left && side != ArmSide::Right) throw std::invalid_argument("Invalid arm");
            return side == ArmSide::Left ? 0 : 1;
        }
        bool active(ReachState s) { return s == ReachState::Moving || s == ReachState::Settling || s == ReachState::Planning; }
        std::array<JointId,3> joints(std::size_t s)
        { return s == 0 ? std::array{JointId::LeftShoulder,JointId::LeftElbow,JointId::LeftWrist} :
            std::array{JointId::RightShoulder,JointId::RightElbow,JointId::RightWrist}; }
        constexpr std::array<int,6> jointSlot{0,0,0,1,2,2}, coordinateSlot{0,1,2,0,0,1};
        bool finite(glm::vec3 v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }
        void validate(const ReachTarget& t)
        {
            const auto s = index(t.arm);
            if ((t.task!=ReachTask::PositionOnly && t.task!=ReachTask::PositionAndOrientation) ||
                !std::isfinite(t.angularTolerance) || t.angularTolerance<.001745329f || t.angularTolerance>.17453293f)
                throw std::invalid_argument("Invalid reach task/angular tolerance (0.1 to 10 degrees required)");
            (void)normalizedReachOrientation(t.orientation);
            const bool effector = s == 0 ? t.effector == EffectorId::LeftWrist || t.effector == EffectorId::LeftPalm || t.effector == EffectorId::LeftGrip :
                t.effector == EffectorId::RightWrist || t.effector == EffectorId::RightPalm || t.effector == EffectorId::RightGrip;
            const auto weight=[](double v) { return std::isfinite(v) && v>=0 && v<=1000; };
            if ((t.selection!=RouteSelection::FirstValid && t.selection!=RouteSelection::Ranked) ||
                (t.selection==RouteSelection::Ranked && t.strategy!=MovementStrategy::AllowDetour) ||
                !weight(t.ranking.elbowWeight) || !weight(t.ranking.travelWeight) || !weight(t.ranking.durationWeight) ||
                t.ranking.additionalTicks>4096 || !t.ranking.maxEvaluations || t.ranking.maxEvaluations>36)
                throw std::invalid_argument("Invalid route selection/finite ranking parameters; Ranked requires AllowDetour");
            if (t.elbowPreference) {
                const auto& p=*t.elbowPreference;
                const double length=glm::length(glm::dvec3{p.direction});
                if (!finite(p.direction) || length<1e-6 || length>1e6 ||
                    (p.space!=TargetSpace::Body && p.space!=TargetSpace::World))
                    throw std::invalid_argument("Invalid elbow direction/space (dimensionless nonzero direction required)");
            }
            if (!effector || !finite(t.position) || glm::length(t.position)>10000 || (t.space != TargetSpace::Body && t.space != TargetSpace::World) ||
                !std::isfinite(t.tolerance) || t.tolerance < .0005f || t.tolerance > .02f ||
                !std::isfinite(t.maxSpeed) || t.maxSpeed < .1f || t.maxSpeed > 4 ||
                !std::isfinite(t.maxAcceleration) || t.maxAcceleration < .2f || t.maxAcceleration > 20 ||
                !std::isfinite(t.settleSeconds) || t.settleSeconds < .05f || t.settleSeconds > 2 ||
                (t.strategy!=MovementStrategy::DirectOnly && t.strategy!=MovementStrategy::AllowDetour) ||
                t.detour.maxCandidates>32 || !t.detour.posesPerTick || t.detour.posesPerTick>256 ||
                !t.detour.maxConfigurations || t.detour.maxConfigurations>4 ||
                !t.detour.iterationsPerTick || t.detour.iterationsPerTick>128 ||
                !t.detour.seedsPerTick || t.detour.seedsPerTick>4 ||
                !t.detour.maxIkIterations || t.detour.maxIkIterations>16384 ||
                !t.detour.maxSeedAttempts || t.detour.maxSeedAttempts>30 ||
                !t.detour.maxTotalPoses || t.detour.maxTotalPoses>131072 ||
                !t.detour.queriesPerTick || t.detour.queriesPerTick>16384 || !t.detour.maxTotalQueries || t.detour.maxTotalQueries>4000000)
                throw std::invalid_argument("Invalid reach target/limits");
        }
        struct Chain
        {
            std::array<const JointDefinition*,3> definitions;
            std::array<double,6> initial{}, lower{}, upper{};
            glm::mat4 base{1}, upperToElbow{1}, elbowToWrist{1}, wristToEffector{1};
            float length1 = 0, length2 = 0, offset = 0;
            mutable unsigned evaluations = 0;
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
            { return glm::vec3{frame(q)[3]}; }
            glm::mat4 frame(const std::array<double,6>& q) const
            { ++evaluations; return base*rotation(0,q)*upperToElbow*rotation(1,q)*elbowToWrist*rotation(2,q)*wristToEffector; }
        };
        glm::quat bodyOrientation(const ReachTarget& t,const std::optional<glm::quat>& body)
        {
            if (t.task==ReachTask::PositionAndOrientation && t.space==TargetSpace::World && !body)
                throw std::invalid_argument("World IK requires frozen Body orientation");
            return normalizedReachOrientation(body.value_or(t.orientation));
        }
        using pose_ik::poseError;
        using pose_ik::poseReached;
        using pose_ik::iteratePose;
        // One bounded DLS iteration; shared by legacy and resumable solvers.
        bool iterate(const Chain& chain, glm::vec3 goal, std::size_t dimensions,
            std::array<double,6>& q, float& error)
        {
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
                return improved;
        }
        double polynomial(const std::array<double,6>& c, double u)
        { return ((((c[5]*u+c[4])*u+c[3])*u+c[2])*u+c[1])*u+c[0]; }
    }
    const char* reachTaskName(ReachTask task)
    { return task==ReachTask::PositionOnly ? "PositionOnly":task==ReachTask::PositionAndOrientation ? "PositionAndOrientation":"invalid"; }
    glm::quat normalizedReachOrientation(glm::quat value)
    {
        const glm::dquat q{value}; const double norm=glm::length(q);
        if (!std::isfinite(norm) || norm<1e-6 || norm>1e6) throw std::invalid_argument("Invalid finite orientation quaternion norm [1e-6,1e6]");
        auto unit=q/norm;
        // Lexicographic tie at pi makes antipodal input canonical even when w==0.
        if (unit.w<0 || (unit.w==0 && (unit.x<0 || (unit.x==0 && (unit.y<0 || (unit.y==0 && unit.z<0)))))) unit=-unit;
        return glm::quat{unit};
    }
    glm::dvec3 orientationResidual(glm::quat target,glm::quat actual)
    {
        auto relative=glm::normalize(glm::dquat{normalizedReachOrientation(target)}*glm::conjugate(glm::dquat{normalizedReachOrientation(actual)}));
        if (relative.w<0 || (relative.w==0 && (relative.x<0 || (relative.x==0 && (relative.y<0 || (relative.y==0 && relative.z<0)))))) relative=-relative;
        const glm::dvec3 vector{relative.x,relative.y,relative.z}; const double sine=glm::length(vector);
        if (sine<1e-10) return 2.0*vector;
        return vector*(2*std::atan2(sine,std::max(0.0,relative.w))/sine);
    }
    const char* selectionCompletionName(SelectionCompletion c)
    {
        switch (c) {
        case SelectionCompletion::Running: return "running";
        case SelectionCompletion::FirstValid: return "first valid";
        case SelectionCompletion::FiniteSet: return "finite route set exhausted";
        case SelectionCompletion::RankingLimit: return "additional ranking tick limit";
        case SelectionCompletion::BudgetExhausted: return "shared operation budget exhausted";
        case SelectionCompletion::CheckIncomplete: return "segment verification incomplete";
        case SelectionCompletion::InvalidInitial: return "invalid initial pose";
        case SelectionCompletion::IkFailed: return "no generated IK solution";
        case SelectionCompletion::Cancelled: return "cancelled";
        case SelectionCompletion::MotionUnavailable: return "motion unavailable";
        }
        return "unknown";
    }
    const char* elbowPreferenceStatusName(ElbowPreferenceStatus s)
    {
        switch (s) {
        case ElbowPreferenceStatus::Absent: return "absent";
        case ElbowPreferenceStatus::Defined: return "defined";
        case ElbowPreferenceStatus::StraightArm: return "straight arm";
        case ElbowPreferenceStatus::ParallelDirection: return "parallel direction";
        case ElbowPreferenceStatus::CollapsedAxis: return "collapsed shoulder-wrist axis";
        }
        return "unknown";
    }
    ElbowEvaluation evaluateElbowPreference(glm::vec3 shoulder,glm::vec3 elbow,glm::vec3 wrist,
        const std::optional<glm::vec3>& direction)
    {
        ElbowEvaluation result; result.shoulder=shoulder; result.elbow=elbow; result.wrist=wrist;
        if (!direction) return result;
        result.penalty=1; // Undefined geometry never receives a free good score.
        const auto axis=glm::dvec3{wrist}-glm::dvec3{shoulder};
        const double length=glm::length(axis);
        if (!finite(shoulder) || !finite(elbow) || !finite(wrist) || !std::isfinite(length) || length<1e-6) {
            result.status=ElbowPreferenceStatus::CollapsedAxis; return result;
        }
        const auto unit=axis/length, arm=glm::dvec3{elbow}-glm::dvec3{shoulder};
        const auto bend=arm-unit*glm::dot(arm,unit);
        if (glm::length(bend)<.001) { result.status=ElbowPreferenceStatus::StraightArm; return result; }
        const double directionLength=glm::length(glm::dvec3{*direction});
        if (!finite(*direction) || directionLength<1e-6) { result.status=ElbowPreferenceStatus::ParallelDirection; return result; }
        const auto desired=glm::dvec3{*direction}/directionLength;
        const auto projected=desired-unit*glm::dot(desired,unit);
        if (glm::length(projected)<.001) { result.status=ElbowPreferenceStatus::ParallelDirection; return result; }
        result.status=ElbowPreferenceStatus::Defined;
        result.penalty=(1-std::clamp(glm::dot(glm::normalize(bend),glm::normalize(projected)),-1.0,1.0))*.5;
        return result;
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
        case ReachState::Planning: return "planning";
        }
        return "unknown";
    }
    const char* configurationOutcomeName(ConfigurationOutcome outcome)
    {
        switch (outcome) {
        case ConfigurationOutcome::Pending: return "pending";
        case ConfigurationOutcome::Blocked: return "path blocked";
        case ConfigurationOutcome::QuotaExhausted: return "configuration budget share exhausted";
        case ConfigurationOutcome::CheckIncomplete: return "path check incomplete";
        case ConfigurationOutcome::MotionUnavailable: return "bounded segment unavailable";
        case ConfigurationOutcome::CandidatesExhausted: return "finite waypoint candidates exhausted";
        case ConfigurationOutcome::Accepted: return "accepted";
        case ConfigurationOutcome::Verified: return "fully verified route found";
        }
        return "unknown";
    }
    const char* ikSeedOutcomeName(IkSeedOutcome outcome)
    {
        switch (outcome) {
        case IkSeedOutcome::None: return "none";
        case IkSeedOutcome::Accepted: return "unique FK-verified solution";
        case IkSeedOutcome::Duplicate: return "duplicate solution";
        case IkSeedOutcome::LocalFailure: return "local IK did not converge";
        case IkSeedOutcome::FkRejected: return "actual FK/limits rejected solution";
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
        case ReachReason::PathBlocked: return "blocked path";
        case ReachReason::InitialIntersection: return "initial intersection";
        case ReachReason::CheckIncomplete: return "collision verification incomplete";
        case ReachReason::DetourExhausted: return "finite waypoint set exhausted (not a no-path proof)";
        case ReachReason::PlanningBudgetExhausted: return "total planning operation budget exhausted";
        case ReachReason::OrientationNotReached: return "final orientation outside angular tolerance";
        case ReachReason::GeometryContextChanged: return "frozen geometry/support dependencies changed";
        }
        return "unknown";
    }
    ArmSolution solveArmPosition(const DirectBodyControl& control, const ReachTarget& target, glm::vec3 goal,
        std::optional<glm::quat> orientation)
    {
        validate(target);
        if (!control.enabled() || !finite(goal)) throw std::invalid_argument("Reach requires calibrated direct mode and finite body goal");
        Chain chain(control,target);
        if (target.task==ReachTask::PositionAndOrientation) {
            auto single=target; single.detour.maxConfigurations=1;
            ArmPositionSearchJob job(control,single,goal,bodyOrientation(target,orientation));
            while (!job.advance(128,1)) {}
            ArmSolution result;
            if (!job.solutions().empty()) result=job.solutions().front();
            else {
                result.coordinates=chain.initial; const auto frame=chain.frame(chain.initial);
                result.error=glm::length(glm::vec3{frame[3]}-goal);
                result.angularError=static_cast<float>(glm::length(orientationResidual(bodyOrientation(target,orientation),glm::quat_cast(glm::mat3{frame}))));
                const auto distance=glm::length(goal-glm::vec3{chain.base[3]});
                if (distance>chain.length1+chain.length2+chain.offset+target.tolerance ||
                    distance<std::max(0.0f,std::abs(chain.length1-chain.length2)-chain.offset)-target.tolerance)
                    result.reason=ReachReason::GeometricRange;
                else if (job.budgetExhausted()) result.reason=ReachReason::PlanningBudgetExhausted;
            }
            result.iterations=job.observation().ikIterations; result.evaluations=job.observation().ikEvaluations;
            return result;
        }
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
                if (!iterate(chain,goal,dimensions,q,error)) break;
            }
            if (error < best.error) { best.error=error; best.coordinates=q; }
        }
        bool atLimit=false;
        for (std::size_t k=0;k<dimensions;++k)
            atLimit |= best.coordinates[k]-chain.lower[k]<.002 || chain.upper[k]-best.coordinates[k]<.002;
        best.reason = best.error <= target.tolerance*.5f ? ReachReason::Reached : geometric ? ReachReason::GeometricRange :
            atLimit ? ReachReason::JointLimited : ReachReason::LocalNoConvergence;
        best.evaluations=chain.evaluations;
        return best;
    }
    struct ArmPositionSearchJob::Impl
    {
        DirectBodyControl frozen;
        ReachTarget target;
        glm::vec3 goal;
        glm::quat orientation;
        Chain chain;
        PlanningObservation diagnostic;
        std::vector<ArmSolution> accepted;
        ArmSolution best;
        std::array<double,6> q{};
        float error=0;
        unsigned legacySeed=0, extraSeed=0, seedIterations=0;
        bool first=true, running=false, complete=false, exhausted=false;
        std::size_t dimensions;
        Impl(const DirectBodyControl& c,const ReachTarget& t,glm::vec3 g,std::optional<glm::quat> o)
            : frozen(c),target(t),goal(g),orientation(bodyOrientation(t,o)),chain(frozen,t),
              dimensions(t.task==ReachTask::PositionOnly && (t.effector==EffectorId::LeftWrist || t.effector==EffectorId::RightWrist) ? 4:6)
        {
            validate(t);
            if (!c.enabled() || !finite(g)) throw std::invalid_argument("IK search requires direct mode and finite goal");
            accepted.reserve(4); best.coordinates=chain.initial;
            best.error=metric(best.coordinates);
            diagnostic.state=diagnostic.stage=PlanningState::GeneratingIK;
        }
        float metric(const std::array<double,6>& coordinates) const
        { return target.task==ReachTask::PositionOnly ? glm::length(chain.position(coordinates)-goal):poseError(chain,coordinates,goal,orientation); }
        bool reached(const std::array<double,6>& coordinates,float e,float fraction) const
        { return target.task==ReachTask::PositionOnly ? e<=target.tolerance*fraction:poseReached(chain,coordinates,goal,orientation,target,fraction); }
        bool admit(ArmSolution solution)
        {
            // Quantize before FK: these are the actual saved/executed endpoint q.
            const auto ids=joints(index(target.arm));
            for (std::size_t k=0;k<6;++k) {
                solution.coordinates[k]=static_cast<float>(solution.coordinates[k]);
                if (!std::isfinite(solution.coordinates[k]) || solution.coordinates[k]<chain.lower[k] || solution.coordinates[k]>chain.upper[k]) return false;
                frozen.setCoordinate(ids[static_cast<std::size_t>(jointSlot[k])],static_cast<std::size_t>(coordinateSlot[k]),static_cast<float>(solution.coordinates[k]));
            }
            animation::Pose actual(frozen.binding().asset().skeleton()); frozen.apply(actual.locals);
            animation::computeGlobalTransforms(frozen.binding().asset().skeleton(),actual.locals,actual.globals);
            ++diagnostic.fkChecks;
            const auto frame=frozen.binding().effectorFrame(target.effector,actual.globals);
            solution.error=glm::length(glm::vec3{frame[3]}-goal);
            solution.angularError=static_cast<float>(glm::length(orientationResidual(orientation,glm::quat_cast(glm::mat3{frame}))));
            if (!std::isfinite(solution.error) || solution.error>target.tolerance) return false;
            if (target.task==ReachTask::PositionAndOrientation && (!std::isfinite(solution.angularError) || solution.angularError>target.angularTolerance)) return false;
            // Distinct if at least one ACTIVE coordinate differs by >= 0.08 rad.
            // Inactive wrist coordinates never create a spurious wrist solution.
            for (const auto& previous:accepted) {
                double distance=0;
                for (std::size_t k=0;k<dimensions;++k) distance=std::max(distance,std::abs(previous.coordinates[k]-solution.coordinates[k]));
                if (distance<.08) { ++diagnostic.duplicates; diagnostic.lastSeedOutcome=IkSeedOutcome::Duplicate; return true; }
            }
            diagnostic.lastSeedOutcome=IkSeedOutcome::Accepted;
            solution.reason=ReachReason::Reached; accepted.push_back(solution);
            diagnostic.configurations=static_cast<unsigned>(accepted.size());
            diagnostic.budgetConfigurations=diagnostic.configurations;
            auto& entry=diagnostic.solutions[accepted.size()-1]; entry.error=solution.error; entry.angularError=solution.angularError;
            for (std::size_t k=0;k<6;++k) entry.coordinates[k]=static_cast<float>(solution.coordinates[k]);
            return true;
        }
        void finishSeed()
        {
            running=false; ++diagnostic.seedsProcessed; diagnostic.lastSeedOutcome=IkSeedOutcome::LocalFailure;
            if (error<best.error) { best.error=error; best.coordinates=q; }
            if (first) {
                ++legacySeed;
                if (reached(best.coordinates,best.error,.2f) || legacySeed==9) {
                    first=false;
                    best.iterations=diagnostic.ikIterations;
                    if (reached(best.coordinates,best.error,.5f)) { if (!admit(best)) { ++diagnostic.failedSeeds; diagnostic.lastSeedOutcome=IkSeedOutcome::FkRejected; } }
                    else ++diagnostic.failedSeeds;
                } else ++diagnostic.failedSeeds;
            } else {
                ArmSolution solution; solution.coordinates=q; solution.error=error; solution.iterations=seedIterations;
                if (!reached(q,error,.5f)) ++diagnostic.failedSeeds;
                else if (!admit(solution)) { ++diagnostic.failedSeeds; diagnostic.lastSeedOutcome=IkSeedOutcome::FkRejected; }
                ++extraSeed;
            }
            if (accepted.size()>=target.detour.maxConfigurations || (!first && (target.detour.maxConfigurations==1 || extraSeed==21))) complete=true;
        }
        bool advance(unsigned iterations,unsigned seeds)
        {
            const auto evaluationBegin=diagnostic.ikEvaluations;
            // Full 6D iterations evaluate six columns and rotation logs. Keep the
            // old configured iteration budget as an upper bound, cap work at four/tick.
            if (target.task==ReachTask::PositionAndOrientation) iterations=std::min(iterations,4u);
            diagnostic.tickIterations=diagnostic.tickSeeds=0;
            while (!complete) {
                if (!running) {
                    if (diagnostic.seedAttempts>=target.detour.maxSeedAttempts) { complete=exhausted=true; break; }
                    if (!seeds || !iterations) break;
                    --seeds; ++diagnostic.tickSeeds; ++diagnostic.seedAttempts;
                    q=chain.initial;
                    const unsigned seed=first ? legacySeed:extraSeed;
                    if (seed && seed<9) for (std::size_t k=0;k<dimensions;++k) {
                        const double fraction=seed==1 ? .5:((seed*(k*2+3))%11+1)/12.0;
                        q[k]=chain.lower[k]+fraction*(chain.upper[k]-chain.lower[k]);
                    }
                    else if (seed>=9) {
                        const std::size_t k=((seed-9)/2)%dimensions;
                        q[k]=std::clamp(q[k]+(seed%2 ? .3:-.3)*(chain.upper[k]-chain.lower[k]),chain.lower[k],chain.upper[k]);
                    }
                    error=metric(q); seedIterations=0; running=true;
                }
                if (diagnostic.ikIterations>=target.detour.maxIkIterations) { complete=exhausted=true; break; }
                if (!iterations) break;
                --iterations; ++diagnostic.ikIterations; ++diagnostic.tickIterations; ++seedIterations;
                if (first && error<best.error) { best.error=error; best.coordinates=q; }
                const bool success=reached(q,error,.2f);
                if (success && first) { best.error=error; best.coordinates=q; }
                if (success || !(target.task==ReachTask::PositionOnly ? iterate(chain,goal,dimensions,q,error):iteratePose(chain,goal,orientation,q,error)) || seedIterations==160) { finishSeed(); break; }
            }
            if (complete) diagnostic.ikCompletion=exhausted ? IkSearchCompletion::BudgetExhausted:
                accepted.size()>=target.detour.maxConfigurations ? IkSearchCompletion::ConfigurationLimit:IkSearchCompletion::SeedSetExhausted;
            diagnostic.ikEvaluations=chain.evaluations; diagnostic.tickIkEvaluations=chain.evaluations-evaluationBegin;
            return complete;
        }
    };
    ArmPositionSearchJob::ArmPositionSearchJob(const DirectBodyControl& c,const ReachTarget& t,glm::vec3 g,std::optional<glm::quat> o)
        : impl_(std::make_unique<Impl>(c,t,g,o)) {}
    ArmPositionSearchJob::~ArmPositionSearchJob()=default;
    bool ArmPositionSearchJob::advance(unsigned iterations,unsigned seeds) { return impl_->advance(iterations,seeds); }
    bool ArmPositionSearchJob::done() const { return impl_->complete; }
    bool ArmPositionSearchJob::budgetExhausted() const { return impl_->exhausted; }
    const PlanningObservation& ArmPositionSearchJob::observation() const { return impl_->diagnostic; }
    const std::vector<ArmSolution>& ArmPositionSearchJob::solutions() const { return impl_->accepted; }
    std::size_t ArmPositionSearchJob::scratchBytes() const { return sizeof(Impl)+impl_->accepted.capacity()*sizeof(ArmSolution)+impl_->frozen.binding().profile().joints.size()*sizeof(glm::vec3); }
    struct ArmReach::Planner
    {
        ReachObservation proposal, fallback;
        Motion fallbackMotion, first, second;
        std::optional<Motion> fallbackNext;
        struct Best { Motion first; std::optional<Motion> second; PlanningObservation route; };
        std::optional<Best> best;
        ArmSolution final;
        std::unique_ptr<ArmPositionSearchJob> ik;
        std::vector<ArmSolution> solutions;
        std::array<unsigned,4> candidates{};
        std::array<bool,4> retired{};
        unsigned configuration=0, direct=0, next=0;
        bool generated=false, quotaHit=false, incomplete=false;
        std::array<double,6> initial{};
        std::unique_ptr<ArmPathCheckJob> job;
        unsigned phase=0;
        bool hasFallback=false, waiting=false, retarget=false;
        static std::size_t buffers(const PlanningObservation& observation)
        {
            std::size_t bytes=observation.evaluations.capacity()*sizeof(RouteEvaluation);
            for (const auto* check:{&observation.directCheck,&observation.segmentChecks[0],&observation.segmentChecks[1]})
                bytes+=check->trace.capacity()*sizeof(std::array<glm::vec3,3>);
            return bytes;
        }
        std::size_t ownedBytes() const
        {
            return sizeof(Planner)+solutions.capacity()*sizeof(ArmSolution)+buffers(proposal.planning)+buffers(proposal.route)+
                buffers(fallback.planning)+buffers(fallback.route)+(best ? buffers(best->route):0);
        }
    };
    ArmReach::ArmReach(DirectBodyControl& c) : control_(c) { events_.reserve(8); }
    ArmReach::~ArmReach() = default;
    bool ArmReach::anyActive() const { return active(observations_[0].state) || active(observations_[1].state); }
    void ArmReach::finish(std::size_t s, ReachState state, ReachReason reason)
    {
        auto& o=observations_[s]; o.state=state; o.reason=reason;
        o.cancellationPending=false; o.replanningDeferred=false; events_.push_back(o);
    }
    bool ArmReach::owns(JointId joint) const
    {
        if (validator_ && anyActive()) return true; // Freeze all supported geometry, including the other arm/head.
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
    { for (std::size_t s=0;s<2;++s) if (active(observations_[s].state)) {
        observations_[s].collisionChecked=false; observations_[s].collisionSceneVersion=0;
        observations_[s].planning.state=PlanningState::Cancelled;
        observations_[s].planning.completion=SelectionCompletion::Cancelled;
        observations_[s].route={};
        finish(s,ReachState::Cancelled,reason); motion_[s]={}; planners_[s].reset(); nextSegments_[s].reset(); } }
    void ArmReach::reset() { interrupt(ReachReason::BodyReset); observations_={}; motion_={}; }
    void ArmReach::start(std::uint64_t action, std::uint64_t epoch, const ReachTarget& input,
        const glm::mat4& world, bool retarget, std::uint64_t command)
    {
        auto target=input;
        validate(target);
        if (target.task==ReachTask::PositionAndOrientation)
            (void)control_.binding().worldFromResource(world); // Validate proper rigid root before extracting rotation.
        target.orientation=normalizedReachOrientation(target.orientation);
        if (!control_.enabled()) throw std::invalid_argument("IK requires explicit calibrated direct mode; clip pose cannot be inferred");
        const auto s=index(target.arm);
        if (contextValid && !contextValid(s)) throw std::invalid_argument("Stale frozen Reach geometry/support context");
        auto& o=observations_[s]; auto& m=motion_[s];
        if (validator_ && active(observations_[1-s].state)) throw std::invalid_argument("Collision mode permits only one active arm");
        if (retarget && (o.action!=action || o.epoch!=epoch || !active(o.state) || m.cancelling))
            throw std::invalid_argument("No matching active reach action");
        if (active(o.state) && (target.maxSpeed!=o.target.maxSpeed || target.maxAcceleration!=o.target.maxAcceleration))
            throw std::invalid_argument("Retarget/replacement must preserve active motion limits");
        const auto goal=target.space==TargetSpace::World ? glm::vec3{glm::inverse(world)*glm::vec4{target.position,1}} : target.position;
        const auto orientation=normalizedReachOrientation(target.space==TargetSpace::World ?
            glm::conjugate(glm::quat_cast(glm::mat3{world}))*target.orientation:target.orientation);
        if (!finite(goal) || glm::length(goal)>10000) throw std::invalid_argument("Invalid transformed body target");
        if (target.strategy==MovementStrategy::AllowDetour)
        {
            if (!validator_ || !planningValidator_) throw std::invalid_argument("AllowDetour requires an explicit static collision scene");
            // Necessary geometric shell test is independent of the arm endpoint at
            // the pending segment boundary; it does not run a multi-start solver.
            const Chain shell(control_,target);
            const float distance=glm::length(goal-glm::vec3{shell.base[3]});
            if (distance>shell.length1+shell.length2+shell.offset+target.tolerance ||
                distance<std::max(0.0f,std::abs(shell.length1-shell.length2)-shell.offset)-target.tolerance)
                throw ReachTargetUnreachable(reachReasonName(ReachReason::GeometricRange));
            auto proposal=o; proposal.action=action; proposal.epoch=epoch; proposal.target=target; proposal.goalBody=goal;
            proposal.goalOrientationBody=orientation;
            proposal.elbowDirectionBody.reset();
            if (target.elbowPreference) {
                const auto& preference=*target.elbowPreference;
                const auto direction=preference.space==TargetSpace::World ?
                    glm::vec3{glm::inverse(world)*glm::vec4{preference.direction,0}}:preference.direction;
                if (!finite(direction) || glm::length(direction)<1e-6f) throw std::invalid_argument("Invalid transformed elbow direction");
                proposal.elbowDirectionBody=glm::normalize(direction);
            }
            proposal.reason=ReachReason::None; proposal.cancellationPending=false; proposal.replanningDeferred=false;
            proposal.iterations=0; proposal.evaluations=0; proposal.solveMicroseconds=0;
            proposal.collisionChecked=false; proposal.collisionSceneVersion=0; proposal.planning={};
            proposal.planning.command=command ? command:action;
            proposal.planning.selection=target.selection; proposal.planning.ranking=target.ranking;
            proposal.planning.elbowDirectionBody=proposal.elbowDirectionBody;
            if (!retarget) { proposal.maxMeasuredSpeed=0; proposal.maxMeasuredAcceleration=0; }
            requestDetour(s,std::move(proposal),retarget); return;
        }
        const auto begin=std::chrono::steady_clock::now();
        const auto solution=solveArmPosition(control_,target,goal,orientation);
        const auto microseconds=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-begin).count();
        if (validator_ && solution.reason!=ReachReason::Reached) throw ReachTargetUnreachable(reachReasonName(solution.reason));
        if (planners_[s]) throw std::invalid_argument("DirectOnly replacement conflicts with pending planning; use AllowDetour or Cancel");
        // Transactional replacement. Failed validation restores the running action and pending target.
        const auto savedObservation=o; const auto savedMotion=m; const auto savedEvents=events_.size();
        try {
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
        o.goalOrientationBody=orientation;
        o.elbowDirectionBody.reset(); // DirectOnly has no selection stage.
        o.cancellationPending=false;
        o.referenceBody=reference.position(m.q);
        o.iterations=solution.iterations; o.evaluations=solution.evaluations; o.solveMicroseconds=microseconds;
        m.solution=solution; m.settled=0; m.pending=true; m.cancelling=false;
        // A rare unsafe reversal near a bound keeps the valid old segment until rest, then replans.
        // Checked replacements deliberately switch at rest. This freezes the
        // certificate's start pose and avoids promising an immediate retarget.
        const bool deferForCertificate=validator_ && active(savedObservation.state) && savedMotion.tick<savedMotion.duration;
        if (!deferForCertificate && plan(s)) m.pending=false;
        if (validator_)
        {
            PathStatus status;
            if (m.pending)
            {
                const auto pending=m;
                if (!active(savedObservation.state) || savedMotion.duration==0)
                    throw std::invalid_argument("No bounded motion segment for deferred collision check");
                for (std::size_t k=0;k<6;++k) m.q[k]=savedMotion.certifiedRoute ? savedMotion.certifiedEndpoint[k]:polynomial(savedMotion.polynomial[k],1);
                m.velocity={}; m.acceleration={}; m.lastStepVelocity={};
                if (!plan(s)) throw std::invalid_argument("No bounded deferred motion segment");
                status=verify(s,savedMotion.duration-savedMotion.tick); m=pending; // Keep the old certified segment until rest.
            }
            else status=verify(s);
            if (status!=PathStatus::Clear) throw std::invalid_argument(pathStatusName(status));
        }
        o.replanningDeferred=m.pending;
        o.collisionChecked=static_cast<bool>(validator_); o.collisionSceneVersion=validator_ ? validationSceneVersion_:0;
        nextSegments_[s].reset(); o.planning={}; o.route={};
        } catch (...) { o=savedObservation; m=savedMotion; events_.resize(savedEvents); throw; }
    }
    const char* planningStateName(PlanningState state)
    {
        switch (state) {
        case PlanningState::Idle: return "idle";
        case PlanningState::GeneratingIK: return "generating IK configurations";
        case PlanningState::WaitingForStop: return "waiting for segment stop";
        case PlanningState::CheckingDirect: return "checking direct";
        case PlanningState::CheckingFirst: return "checking first segment";
        case PlanningState::CheckingSecond: return "checking second segment";
        case PlanningState::Paused: return "paused until next tick";
        case PlanningState::RouteReady: return "route ready";
        case PlanningState::Exhausted: return "candidate set exhausted";
        case PlanningState::BudgetExhausted: return "total budget exhausted";
        case PlanningState::CheckIncomplete: return "segment check incomplete";
        case PlanningState::InvalidInitial: return "invalid initial pose";
        case PlanningState::IkFailed: return "IK failed";
        case PlanningState::Cancelled: return "cancelled";
        case PlanningState::MotionUnavailable: return "bounded motion unavailable";
        }
        return "unknown";
    }
    std::vector<ArmCoordinates> ArmReach::samples(const Motion& m)
    {
        std::vector<ArmCoordinates> path(m.duration+1);
        for (unsigned t=0;t<=m.duration;++t) for (std::size_t k=0;k<6;++k)
            path[t][k]=static_cast<float>(m.certifiedRoute && t==m.duration ? m.certifiedEndpoint[k]:polynomial(m.polynomial[k],static_cast<double>(t)/m.duration));
        return path;
    }
    ArmReach::Motion ArmReach::restSegment(std::size_t s,const std::array<double,6>& from,
        const std::array<double,6>& to,const ReachObservation& proposal) const
    {
        Motion m; m.q=from; m.solution.coordinates=to; m.solution.reason=ReachReason::Reached; m.certifiedRoute=true;
        m.certifiedEndpoint=to;
        const Chain reference(control_,proposal.target);
        m.frames={reference.base,reference.upperToElbow,reference.elbowToWrist,reference.wristToEffector};
        const auto ids=joints(s);
        // Rest-to-rest quintic. Conservative continuous extrema give a bounded starting
        // duration; validate the actual float samples and both hold ticks as executed.
        double seconds=.25;
        for (std::size_t k=0;k<6;++k) {
            const double distance=std::abs(to[k]-from[k]);
            seconds=std::max({seconds,1.875*distance/(proposal.target.maxSpeed*.995),
                std::sqrt(5.773503*distance/(proposal.target.maxAcceleration*.99))});
        }
        unsigned ticks=30;
        if (seconds/dt>30) ticks+=12*static_cast<unsigned>(std::ceil((seconds/dt-30)/12));
        for (;ticks<=6000;ticks+=12) {
            for (std::size_t k=0;k<6;++k) {
                const double d=to[k]-from[k]; m.polynomial[k]={from[k],0,0,10*d,-15*d,6*d};
            }
            bool valid=true; auto last=from; std::array<double,6> lastVelocity{};
            for (unsigned t=1;t<=ticks+2 && valid;++t) for (std::size_t k=0;k<6;++k) {
                const double q=static_cast<float>(t>=ticks ? to[k]:polynomial(m.polynomial[k],static_cast<double>(t)/ticks));
                const double v=(q-last[k])/dt, a=(v-lastVelocity[k])/dt;
                const auto& limit=control_.binding().profile().joints[control_.binding().jointIndex(ids[static_cast<std::size_t>(jointSlot[k])])].coordinates[static_cast<std::size_t>(coordinateSlot[k])];
                if (q<limit.lower || q>limit.upper || std::abs(v)>proposal.target.maxSpeed || std::abs(a)>proposal.target.maxAcceleration) { valid=false; break; }
                last[k]=q; lastVelocity[k]=v;
            }
            if (valid) { m.duration=ticks; return m; }
        }
        throw std::invalid_argument("No bounded rest-to-rest segment within 50 seconds");
    }
    void ArmReach::requestDetour(std::size_t s,ReachObservation proposal,bool retarget)
    {
        auto p=std::make_unique<Planner>(); p->proposal=std::move(proposal); p->retarget=retarget;
        if (planners_[s]) {
            p->fallback=planners_[s]->fallback; p->fallbackMotion=planners_[s]->fallbackMotion;
            p->fallbackNext=planners_[s]->fallbackNext; p->hasFallback=planners_[s]->hasFallback;
            p->waiting=planners_[s]->waiting;
        } else {
            p->fallback=observations_[s]; p->fallbackMotion=motion_[s]; p->fallbackNext=nextSegments_[s];
            p->hasFallback=active(observations_[s].state);
            p->waiting=p->hasFallback && motion_[s].tick<motion_[s].duration;
        }
        p->proposal.state=p->waiting ? ReachState::Moving:ReachState::Planning;
        if (!p->hasFallback) p->proposal.route={};
        p->proposal.planning.state=p->waiting ? PlanningState::WaitingForStop:PlanningState::GeneratingIK;
        p->proposal.replanningDeferred=p->waiting;
        observations_[s]=p->proposal;
        planners_[s]=std::move(p);
    }
    void ArmReach::endPlanning(std::size_t s,PlanningState state,ReachReason reason)
    {
        const auto completion=state==PlanningState::BudgetExhausted ? SelectionCompletion::BudgetExhausted:
            state==PlanningState::Exhausted ? SelectionCompletion::FiniteSet:
            state==PlanningState::CheckIncomplete ? SelectionCompletion::CheckIncomplete:
            state==PlanningState::InvalidInitial ? SelectionCompletion::InvalidInitial:
            state==PlanningState::IkFailed ? SelectionCompletion::IkFailed:SelectionCompletion::MotionUnavailable;
        if (planners_[s]->best && (state==PlanningState::BudgetExhausted || state==PlanningState::Exhausted || state==PlanningState::CheckIncomplete)) {
            admitBest(s,completion); return;
        }
        auto p=std::move(planners_[s]);
        auto diagnostic=observations_[s].planning; diagnostic.state=state;
        diagnostic.completion=completion;
        if (state==PlanningState::BudgetExhausted && diagnostic.budgetStage!=PlanningState::Idle) diagnostic.stage=diagnostic.budgetStage;
        // A refused Retarget is an attempt outcome, not a terminal event for the
        // still-authorized action with the same ID. New failed actions may terminate.
        if (!p->hasFallback || !p->retarget) {
            observations_[s]=p->proposal; observations_[s].planning=diagnostic;
            finish(s,ReachState::Failed,reason);
        }
        if (p->hasFallback) {
            observations_[s]=p->fallback; observations_[s].planning=std::move(diagnostic);
            motion_[s]=p->fallbackMotion; nextSegments_[s]=p->fallbackNext;
        } else { motion_[s]={}; nextSegments_[s].reset(); }
    }
    void ArmReach::admitBest(std::size_t s,SelectionCompletion completion)
    {
        if (contextValid && !contextValid(s)) { interrupt(ReachReason::GeometryContextChanged); return; }
        auto& p=*planners_[s]; auto& o=observations_[s]; auto& diagnostic=o.planning;
        const auto& chosen=p.best->route;
        diagnostic.state=diagnostic.stage=PlanningState::RouteReady; diagnostic.completion=completion;
        diagnostic.segmentCount=chosen.segmentCount; diagnostic.currentSegment=0;
        diagnostic.segmentDurations=chosen.segmentDurations; diagnostic.segmentChecks=chosen.segmentChecks;
        diagnostic.selectedCandidate=chosen.selectedCandidate; diagnostic.selectedConfiguration=chosen.selectedConfiguration;
        diagnostic.finalCoordinates=chosen.finalCoordinates; diagnostic.waypoint=chosen.waypoint; diagnostic.score=chosen.score;
        diagnostic.solutions[chosen.selectedConfiguration-1].outcome=ConfigurationOutcome::Accepted;
        diagnostic.segmentChecks[0].segmentStartTick=advancingTick_;
        if (p.best->second) diagnostic.segmentChecks[1].segmentStartTick=advancingTick_+p.best->first.duration+1;
        if (pathReporter_) pathReporter_(diagnostic.segmentChecks[p.best->second ? 1:0]);
        if (p.hasFallback && !p.retarget) {
            auto previous=p.fallback; previous.state=ReachState::Cancelled; previous.reason=ReachReason::Superseded; events_.push_back(std::move(previous));
        }
        o.state=ReachState::Moving; o.reason=ReachReason::None; o.collisionChecked=true;
        o.collisionSceneVersion=validationSceneVersion_; o.route=diagnostic; motion_[s]=p.best->first;
        nextSegments_[s]=p.best->second;
        planners_[s].reset();
    }
    void ArmReach::advancePlanning(std::size_t s)
    {
        auto& p=*planners_[s]; auto& o=observations_[s]; auto& diagnostic=o.planning;
        ++diagnostic.ticks;
        diagnostic.tickIterations=diagnostic.tickSeeds=diagnostic.tickQueries=diagnostic.tickPoses=diagnostic.tickCandidates=diagnostic.tickScoreEvaluations=diagnostic.tickIkEvaluations=0;
        const auto& limits=p.proposal.target.detour;
        if (p.best && p.proposal.target.selection==RouteSelection::Ranked &&
            diagnostic.ticks-diagnostic.firstClearTick>p.proposal.target.ranking.additionalTicks) {
            admitBest(s,SelectionCompletion::RankingLimit); return;
        }
        if (p.waiting) {
            p.waiting=false; p.fallbackMotion=motion_[s];
            p.fallbackMotion.lastStepVelocity={}; p.fallbackMotion.velocity={}; p.fallbackMotion.acceleration={};
            o.state=ReachState::Planning; o.replanningDeferred=false;
        }
        if (!p.generated) {
            diagnostic.stage=PlanningState::GeneratingIK;
            const auto begin=std::chrono::steady_clock::now();
            if (!p.ik) {
                const auto ids=joints(s);
                for (std::size_t k=0;k<6;++k) p.initial[k]=control_.coordinate(ids[static_cast<std::size_t>(jointSlot[k])],static_cast<std::size_t>(coordinateSlot[k]));
                p.ik=std::make_unique<ArmPositionSearchJob>(control_,p.proposal.target,p.proposal.goalBody,p.proposal.goalOrientationBody);
            }
            const bool done=p.ik->advance(limits.iterationsPerTick,limits.seedsPerTick);
            const auto& ik=p.ik->observation();
            diagnostic.seedAttempts=ik.seedAttempts; diagnostic.seedsProcessed=ik.seedsProcessed;
            diagnostic.ikIterations=ik.ikIterations; diagnostic.duplicates=ik.duplicates;
            diagnostic.ikEvaluations=ik.ikEvaluations; diagnostic.tickIkEvaluations=ik.tickIkEvaluations;
            diagnostic.failedSeeds=ik.failedSeeds; diagnostic.fkChecks=ik.fkChecks; diagnostic.lastSeedOutcome=ik.lastSeedOutcome;
            diagnostic.ikCompletion=ik.ikCompletion;
            diagnostic.configurations=ik.configurations; diagnostic.budgetConfigurations=ik.budgetConfigurations;
            diagnostic.solutions=ik.solutions;
            diagnostic.tickIterations=ik.tickIterations; diagnostic.tickSeeds=ik.tickSeeds;
            diagnostic.ikMicroseconds+=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-begin).count();
            diagnostic.scratchBytes=std::max(diagnostic.scratchBytes,p.ownedBytes()+p.ik->scratchBytes());
            o.iterations=diagnostic.ikIterations; o.evaluations=diagnostic.ikEvaluations; o.solveMicroseconds=diagnostic.ikMicroseconds;
            if (!done) { diagnostic.state=PlanningState::Paused; return; }
            if (p.ik->budgetExhausted()) {
                diagnostic.budgetReason=ik.ikIterations>=limits.maxIkIterations ? "IK iterations":"IK seed attempts";
                diagnostic.budgetStage=PlanningState::GeneratingIK;
                endPlanning(s,PlanningState::BudgetExhausted,ReachReason::PlanningBudgetExhausted); return;
            }
            p.solutions=p.ik->solutions(); p.ik.reset(); p.generated=true;
            if (p.solutions.empty()) {
                diagnostic.directCheck.status=PathStatus::Unreachable;
                endPlanning(s,PlanningState::IkFailed,ReachReason::LocalNoConvergence); return;
            }
            diagnostic.state=diagnostic.stage=PlanningState::CheckingDirect;
            return; // IK and path construction never share an unbounded command tick.
        }
        const auto count=static_cast<unsigned>(p.solutions.size());
        const auto quota=[&](unsigned total) { return total/count+(p.configuration<total%count ? 1u:0u); };
        const auto selectNext=[&]() {
            p.job.reset();
            if (p.direct<count) { ++p.direct; p.phase=0; }
            else p.phase=1;
            diagnostic.state=diagnostic.stage=p.direct<count ? PlanningState::CheckingDirect:PlanningState::CheckingFirst;
        };
        if (!p.job) {
            if (p.direct<count) { p.configuration=p.direct; p.phase=0; }
            else {
                if (diagnostic.candidates>=limits.maxCandidates) {
                    for (unsigned i=0;i<count;++i) if (!p.retired[i]) diagnostic.solutions[i].outcome=ConfigurationOutcome::CandidatesExhausted;
                    endPlanning(s,p.quotaHit ? PlanningState::BudgetExhausted:p.incomplete ? PlanningState::CheckIncomplete:PlanningState::Exhausted,
                        p.quotaHit ? ReachReason::PlanningBudgetExhausted:p.incomplete ? ReachReason::CheckIncomplete:ReachReason::DetourExhausted); return;
                }
                unsigned scanned=0;
                while (scanned<count && p.retired[p.next]) { p.next=(p.next+1)%count; ++scanned; }
                if (scanned==count) {
                    endPlanning(s,p.quotaHit ? PlanningState::BudgetExhausted:p.incomplete ? PlanningState::CheckIncomplete:PlanningState::Exhausted,
                        p.quotaHit ? ReachReason::PlanningBudgetExhausted:p.incomplete ? ReachReason::CheckIncomplete:ReachReason::DetourExhausted); return;
                }
                p.configuration=p.next; p.next=(p.next+1)%count; p.phase=1;
            }
            diagnostic.currentConfiguration=p.configuration+1;
            p.final=p.solutions[p.configuration];
            auto& entry=diagnostic.solutions[p.configuration];
            if (entry.queries>=quota(limits.maxTotalQueries) || entry.poses>=quota(limits.maxTotalPoses)) {
                p.quotaHit=true; p.retired[p.configuration]=true; entry.outcome=ConfigurationOutcome::QuotaExhausted;
                diagnostic.budgetReason=entry.queries>=quota(limits.maxTotalQueries) ? "configuration query share":"configuration pose share";
                diagnostic.budgetStage=diagnostic.stage;
                selectNext(); return;
            }
            const auto begin=std::chrono::steady_clock::now();
            try {
            if (p.phase==0) {
                p.first=restSegment(s,p.initial,p.final.coordinates,p.proposal);
                diagnostic.state=diagnostic.stage=PlanningState::CheckingDirect;
            } else {
                const auto candidate=p.candidates[p.configuration]++;
                ++diagnostic.candidates; ++diagnostic.tickCandidates; ++entry.candidates;
                diagnostic.budgetCandidates=diagnostic.candidates;
                auto waypoint=p.initial;
                for (std::size_t k=0;k<6;++k) waypoint[k]=static_cast<float>((p.initial[k]+p.final.coordinates[k])*.5);
                const auto ids=joints(s);
                const auto offset=[&](std::size_t k,double fraction) {
                    const auto& coordinate=control_.binding().profile().joints[control_.binding().jointIndex(ids[static_cast<std::size_t>(jointSlot[k])])].coordinates[static_cast<std::size_t>(coordinateSlot[k])];
                    waypoint[k]=static_cast<float>(std::clamp(waypoint[k]+fraction*(coordinate.upper-coordinate.lower),
                        static_cast<double>(coordinate.lower),static_cast<double>(coordinate.upper)));
                };
                // Intrinsic shoulder XYZ and elbow flexion are the same semantic chain
                // on either side. No obstacle-dependent pose list and no RNG.
                if (candidate<16) offset((candidate/2)%4,(candidate%2 ? -1:1)*(candidate<8 ? .18:.36));
                else {
                    const unsigned pair=(candidate-16)/4, signs=(candidate-16)%4;
                    constexpr std::array<std::array<std::size_t,2>,4> pairs{{{0,1},{0,2},{1,3},{2,3}}};
                    offset(pairs[pair][0],(signs&1 ? -1:1)*.25);
                    offset(pairs[pair][1],(signs&2 ? -1:1)*.25);
                }
                for (std::size_t k=0;k<6;++k) diagnostic.waypoint[k]=static_cast<float>(waypoint[k]);
                p.first=restSegment(s,p.initial,waypoint,p.proposal);
                p.second=restSegment(s,waypoint,p.final.coordinates,p.proposal);
                diagnostic.segmentChecks={}; diagnostic.state=PlanningState::CheckingFirst;

                diagnostic.stage=PlanningState::CheckingFirst;
            }
            } catch (const std::invalid_argument& error) {
                entry.outcome=ConfigurationOutcome::MotionUnavailable; entry.detail=error.what(); p.retired[p.configuration]=true; selectNext(); return;
            }
            diagnostic.segmentMicroseconds+=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-begin).count();
            const auto path=samples(p.first); p.job=planningValidator_(s,path);
        }
        auto& entry=diagnostic.solutions[p.configuration];
        const auto remainingQueries=std::min(limits.maxTotalQueries-diagnostic.queries,quota(limits.maxTotalQueries)-entry.queries);
        const auto remainingPoses=std::min(limits.maxTotalPoses-diagnostic.poses,quota(limits.maxTotalPoses)-entry.poses);
        const auto beforeQueries=p.job->result().queries, beforePoses=p.job->result().poses;
        const auto beforeMicroseconds=p.job->result().microseconds;
        const bool done=p.job->advance(std::min(limits.posesPerTick,remainingPoses),std::min(limits.queriesPerTick,remainingQueries));
        auto result=p.job->result(); result.command=diagnostic.command;
        diagnostic.tickQueries=result.queries-beforeQueries; diagnostic.tickPoses=result.poses-beforePoses;
        diagnostic.queries+=diagnostic.tickQueries; diagnostic.poses+=diagnostic.tickPoses;
        entry.queries+=diagnostic.tickQueries; entry.poses+=diagnostic.tickPoses; entry.lastStatus=result.status;
        entry.object=result.object; entry.bodyForm=result.bodyForm; entry.otherBodyForm=result.otherBodyForm;
        entry.gap=result.contact.gap; entry.detail=result.detail.empty() ? pathStatusName(result.status):result.detail;
        diagnostic.pathMicroseconds+=result.microseconds-beforeMicroseconds;
        diagnostic.scratchBytes=std::max(diagnostic.scratchBytes,result.scratchBytes+p.ownedBytes());
        if (p.phase==0) {
            entry.directStatus=result.status;
            if (p.configuration==0) diagnostic.directCheck=result;
        } else diagnostic.segmentChecks[p.phase-1]=result;
        if (pathReporter_) pathReporter_(result);
        if (!done) {
            diagnostic.state=PlanningState::Paused;
            if (entry.queries>=quota(limits.maxTotalQueries) || entry.poses>=quota(limits.maxTotalPoses)) {
                entry.outcome=ConfigurationOutcome::QuotaExhausted; p.retired[p.configuration]=true; p.quotaHit=true;
                diagnostic.budgetReason=entry.queries>=quota(limits.maxTotalQueries) ? "configuration query share":"configuration pose share";
                diagnostic.budgetStage=diagnostic.stage;
                selectNext();
            }
            return;
        }
        p.job.reset();
        if (p.phase==0 && result.status==PathStatus::InitialIntersection) { endPlanning(s,PlanningState::InvalidInitial,ReachReason::InitialIntersection); return; }
        if (result.status==PathStatus::Incomplete) {
            p.incomplete=true; p.retired[p.configuration]=true; entry.outcome=ConfigurationOutcome::CheckIncomplete;
            if (count==1) { endPlanning(s,PlanningState::CheckIncomplete,ReachReason::CheckIncomplete); return; }
            selectNext(); return;
        }
        if (result.status==PathStatus::Clear) {
            if (p.phase==1) {
                p.phase=2; diagnostic.state=diagnostic.stage=PlanningState::CheckingSecond;
                const auto path=samples(p.second); p.job=planningValidator_(s,path); return;
            }
            // Copy the concrete Clear motions and certificates before advancing
            // any search cursor. Scratch motions can now be overwritten safely.
            auto route=diagnostic;
            route.segmentCount=p.phase==0 ? 1u:2u; route.currentSegment=0;
            route.segmentDurations={p.first.duration,p.phase==0 ? 0u:p.second.duration};
            route.selectedCandidate=p.phase==0 ? 0:p.candidates[p.configuration];
            route.selectedConfiguration=p.configuration+1; route.finalCoordinates=entry.coordinates;
            if (p.phase==0) { route.segmentChecks={result,PathCheck{}}; route.waypoint={}; }
            if (p.proposal.target.selection==RouteSelection::Ranked) {
                const auto begin=std::chrono::steady_clock::now();
                route.score={};
                DirectBodyControl trial(control_); const auto ids=joints(s);
                for (std::size_t k=0;k<6;++k)
                    trial.setCoordinate(ids[static_cast<std::size_t>(jointSlot[k])],static_cast<std::size_t>(coordinateSlot[k]),entry.coordinates[k]);
                const auto& binding=control_.binding(); animation::Pose actual(binding.asset().skeleton());
                trial.apply(actual.locals); animation::computeGlobalTransforms(binding.asset().skeleton(),actual.locals,actual.globals);
                auto& score=route.score;
                score.elbow=evaluateElbowPreference(glm::vec3{binding.jointFrame(ids[0],actual.globals)[3]},
                    glm::vec3{binding.jointFrame(ids[1],actual.globals)[3]},glm::vec3{binding.jointFrame(ids[2],actual.globals)[3]},diagnostic.elbowDirectionBody);
                for (std::size_t k=0;k<6;++k) {
                    const auto& coordinate=binding.profile().joints[binding.jointIndex(ids[static_cast<std::size_t>(jointSlot[k])])].coordinates[static_cast<std::size_t>(coordinateSlot[k])];
                    const double range=static_cast<double>(coordinate.upper)-coordinate.lower;
                    const double travel=p.phase==0 ? std::abs(p.final.coordinates[k]-p.initial[k]):
                        std::abs(p.first.certifiedEndpoint[k]-p.initial[k])+std::abs(p.final.coordinates[k]-p.first.certifiedEndpoint[k]);
                    if (range>0) score.travel+=travel/range/6;
                }
                score.duration=(p.first.duration+(p.phase==0 ? 0:p.second.duration+1))*dt;
                const auto& weights=p.proposal.target.ranking;
                score.total=weights.elbowWeight*score.elbow.penalty+weights.travelWeight*score.travel+weights.durationWeight*score.duration;
                score.key=static_cast<std::int64_t>(std::llround(score.total*1e9));
                diagnostic.evaluations.push_back({route.selectedConfiguration,route.selectedCandidate,route.segmentCount,score});
                ++diagnostic.scoreEvaluations; ++diagnostic.tickScoreEvaluations;
                diagnostic.scoreMicroseconds+=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-begin).count();
                diagnostic.scratchBytes=std::max(diagnostic.scratchBytes,p.ownedBytes()+sizeof(actual)+sizeof(trial)+sizeof(route)+
                    (actual.locals.capacity()+actual.globals.capacity()+actual.palette.capacity())*sizeof(glm::mat4)+
                    trial.coordinates().size()*sizeof(glm::vec3)+Planner::buffers(route)+Planner::buffers(diagnostic));
            }
            entry.outcome=ConfigurationOutcome::Verified;
            if (!p.best) diagnostic.firstClearTick=diagnostic.ticks;
            if (!p.best || route.score.key<p.best->route.score.key) {
                p.best=Planner::Best{p.first,p.phase==0 ? std::nullopt:std::optional<Motion>{p.second},std::move(route)};
                diagnostic.score=p.best->route.score;
                diagnostic.selectedConfiguration=p.best->route.selectedConfiguration;
                diagnostic.selectedCandidate=p.best->route.selectedCandidate;
                diagnostic.finalCoordinates=p.best->route.finalCoordinates;
            }
            if (p.proposal.target.selection==RouteSelection::FirstValid) { admitBest(s,SelectionCompletion::FirstValid); return; }
            if (diagnostic.scoreEvaluations>=p.proposal.target.ranking.maxEvaluations) {
                diagnostic.budgetReason="route score evaluations"; diagnostic.budgetStage=diagnostic.stage;
                endPlanning(s,PlanningState::BudgetExhausted,ReachReason::PlanningBudgetExhausted); return;
            }
            selectNext(); return;
        }
        entry.outcome=ConfigurationOutcome::Blocked;
        selectNext();
    }
    PathStatus ArmReach::verify(std::size_t s,unsigned delayTicks)
    {
        if (!validator_) return PathStatus::Disabled;
        const auto& m=motion_[s];
        std::vector<ArmCoordinates> path(m.duration+1);
        for (unsigned t=0;t<=m.duration;++t) for (std::size_t k=0;k<6;++k)
            path[t][k]=static_cast<float>(polynomial(m.polynomial[k],static_cast<double>(t)/m.duration));
        return validator_(s,path,delayTicks);
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
            if (valid) { m.polynomial=candidate; m.duration=ticks; m.tick=0; m.certifiedRoute=false; return true; }
        }
        return false;
    }
    void ArmReach::cancel(ArmSide arm, std::uint64_t action)
    {
        const auto s=index(arm);
        if (observations_[s].action!=action || !active(observations_[s].state)) throw std::invalid_argument("No matching active reach action");
        // Finish the already bounded segment at zero velocity/acceleration, then hold q.
        if (planners_[s]) {
            const auto diagnostic=observations_[s].planning;
            auto p=std::move(planners_[s]);
            if (!p->waiting) { motion_[s].duration=motion_[s].tick=0; }
            observations_[s].planning=diagnostic; observations_[s].planning.state=PlanningState::Cancelled;
            observations_[s].planning.completion=SelectionCompletion::Cancelled;
        }
        nextSegments_[s].reset();
        if (observations_[s].route.segmentCount) observations_[s].route.segmentCount=observations_[s].route.currentSegment+1;
        motion_[s].cancelling=true; motion_[s].pending=false;
        observations_[s].cancellationPending=true; observations_[s].replanningDeferred=false;
    }
    void ArmReach::advance(std::uint64_t tick)
    {
        advancingTick_=tick;
        for (std::size_t s=0;s<2;++s) if (active(observations_[s].state) && contextValid && !contextValid(s)) {
            interrupt(ReachReason::GeometryContextChanged); return;
        }
        for (std::size_t s=0;s<2;++s)
        {
            auto& m=motion_[s]; auto& o=observations_[s];
            // Per-tick work belongs to this completed simulation tick. Keep the
            // admitted route's historical certificate/accounting copy immutable.
            auto& work=o.planning;
            work.tickIterations=work.tickSeeds=work.tickQueries=work.tickPoses=work.tickCandidates=work.tickScoreEvaluations=work.tickIkEvaluations=0;
            if (!active(o.state)) continue;
            if (planners_[s] && (!planners_[s]->waiting || m.tick>=m.duration)) {
                try { advancePlanning(s); }
                catch (const std::invalid_argument&) {
                    if (planners_[s]) endPlanning(s,PlanningState::MotionUnavailable,ReachReason::MotionPlanUnavailable);
                    else throw;
                }
                continue;
            }
            if (m.tick>=m.duration)
            {
                if (nextSegments_[s] && !m.cancelling) {
                    m=*nextSegments_[s]; nextSegments_[s].reset();
                    if (o.planning.state==PlanningState::RouteReady) ++o.planning.currentSegment;
                    ++o.route.currentSegment;
                    o.route.segmentChecks[1].segmentStartTick=advancingTick_;
                    if (pathReporter_) pathReporter_(o.route.segmentChecks[1]);
                    // One full stationary tick makes the two independently checked
                    // endpoint hold ticks the actual finite-difference junction.
                    continue;
                }
                else
                if (m.pending) {
                    if (plan(s)) {
                        const auto status=verify(s);
                        if (validator_ && status!=PathStatus::Clear) {
                            finish(s,ReachState::Failed,status==PathStatus::InitialIntersection ? ReachReason::InitialIntersection:
                                status==PathStatus::Incomplete ? ReachReason::CheckIncomplete:ReachReason::PathBlocked); continue;
                        }
                        m.pending=false; o.replanningDeferred=false;
                    } else { finish(s,ReachState::Failed,ReachReason::MotionPlanUnavailable); continue; }
                }
                else { m.lastStepVelocity={}; o.state=ReachState::Settling; continue; }
            }
            ++m.tick;
            const double u=static_cast<double>(m.tick)/m.duration, seconds=m.duration*dt;
            const auto ids=joints(s);
            for (std::size_t k=0;k<6;++k)
            {
                const auto& c=m.polynomial[k];
                const double q=m.certifiedRoute ? static_cast<float>(m.tick==m.duration ? m.certifiedEndpoint[k]:polynomial(c,u)):polynomial(c,u);
                const double v=(q-m.q[k])/dt, a=(v-m.lastStepVelocity[k])/dt;
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
            if (o.target.space==TargetSpace::World) {
                o.goalBody=glm::vec3{glm::inverse(world)*glm::vec4{o.target.position,1}};
                o.goalOrientationBody=normalizedReachOrientation(glm::conjugate(glm::quat_cast(glm::mat3{world}))*o.target.orientation);
            }
            const auto ids=joints(s);
            const auto actualFrame=b.effectorFrame(o.target.effector,globals);
            o.actualBody=glm::vec3{actualFrame[3]};
            o.actualOrientationBody=normalizedReachOrientation(glm::quat_cast(glm::mat3{actualFrame}));
            o.actualOrientationWorld=normalizedReachOrientation(glm::quat_cast(glm::mat3{world})*o.actualOrientationBody);
            o.angularError=static_cast<float>(glm::length(orientationResidual(o.goalOrientationBody,o.actualOrientationBody)));
            o.actualWorld=glm::vec3{world*glm::vec4{o.actualBody,1}};
            o.shoulderBody=glm::vec3{b.jointFrame(ids[0],globals)[3]}; o.elbowBody=glm::vec3{b.jointFrame(ids[1],globals)[3]};
            const auto wrist=glm::vec3{b.jointFrame(ids[2],globals)[3]};
            o.actualElbow=evaluateElbowPreference(o.shoulderBody,o.elbowBody,wrist,o.elbowDirectionBody);
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
                const bool reached=o.error<=o.target.tolerance &&
                    (o.target.task==ReachTask::PositionOnly || o.angularError<=o.target.angularTolerance);
                ++m.settled;
                if (m.settled>=static_cast<unsigned>(std::ceil(o.target.settleSeconds/dt)))
                    finish(s,m.cancelling ? ReachState::Cancelled : reached ? ReachState::Succeeded:ReachState::Failed,
                        m.cancelling ? ReachReason::UserCancelled : reached ? ReachReason::Reached :
                        o.error<=o.target.tolerance && o.target.task==ReachTask::PositionAndOrientation && o.angularError>o.target.angularTolerance ? ReachReason::OrientationNotReached:
                        m.solution.reason==ReachReason::Reached ? ReachReason::LocalNoConvergence:m.solution.reason);
            }
        }
    }
}
