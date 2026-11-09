#pragma once
#include "julia/embodiment/BodyProfile.hpp"
#include "julia/embodiment/BodyCollision.hpp"
#include <array>
#include <functional>
#include <stdexcept>
#include <optional>

namespace julia::embodiment
{
    class ReachTargetUnreachable : public std::invalid_argument { public: using std::invalid_argument::invalid_argument; };
    enum class ArmSide { Left, Right };
    enum class TargetSpace { Body, World };
    enum class ReachState { Idle, Moving, Settling, Succeeded, Failed, Cancelled, Planning };
    enum class MovementStrategy { DirectOnly, AllowDetour };
    enum class PlanningState { Idle, WaitingForStop, CheckingDirect, CheckingFirst, CheckingSecond,
        Paused, RouteReady, Exhausted, BudgetExhausted, CheckIncomplete, InvalidInitial, IkFailed, Cancelled, MotionUnavailable };
    const char* planningStateName(PlanningState state);
    struct DetourSettings
    {
        unsigned maxCandidates = 32, posesPerTick = 24, queriesPerTick = 2048, maxTotalQueries = 1000000;
    };
    struct PlanningObservation
    {
        PlanningState state = PlanningState::Idle;
        PathCheck directCheck;
        std::array<PathCheck,2> segmentChecks;
        unsigned candidates = 0, queries = 0, poses = 0, ticks = 0, selectedCandidate = 0;
        ArmCoordinates waypoint{};
        unsigned segmentCount = 0, currentSegment = 0;
        std::array<unsigned,2> segmentDurations{};
        std::uint64_t command = 0;
        std::size_t scratchBytes = 0;
    };
    // JointLimited is a constrained LOCAL result, never a global infeasibility proof.
    enum class ReachReason { None, Reached, GeometricRange, JointLimited, LocalNoConvergence,
        MotionPlanUnavailable, UserCancelled, Superseded, ModeChanged, CoordinatesReset, BodyReset, RootChanged,
        PathBlocked, InitialIntersection, CheckIncomplete, DetourExhausted, PlanningBudgetExhausted };
    const char* reachReasonName(ReachReason reason);
    const char* reachStateName(ReachState state);
    struct ReachTarget
    {
        ArmSide arm = ArmSide::Left;
        EffectorId effector = EffectorId::LeftWrist;
        glm::vec3 position{0}; // Metres. Orientation is deliberately unconstrained.
        TargetSpace space = TargetSpace::Body;
        float tolerance = .002f; // Measured AFTER FK, not solver prediction.
        float maxSpeed = 1.2f; // Each controlled coordinate, rad/s.
        float maxAcceleration = 4.0f; // Discrete second difference, rad/s^2.
        float settleSeconds = .1f;
        MovementStrategy strategy = MovementStrategy::DirectOnly;
        DetourSettings detour;
    };
    struct StartReach { ReachTarget target; }; // Action ID = this command's ID + epoch.
    struct RetargetReach { std::uint64_t action; ReachTarget target; };
    struct CancelReach { ArmSide arm; std::uint64_t action; };
    struct ReachObservation
    {
        std::uint64_t action = 0, epoch = 0;
        ReachState state = ReachState::Idle;
        ReachReason reason = ReachReason::None;
        bool cancellationPending = false, replanningDeferred = false;
        bool collisionChecked = false;
        std::uint64_t collisionSceneVersion = 0;
        ReachTarget target;
        glm::vec3 goalBody{0}, referenceBody{0}, actualBody{0}, actualWorld{0};
        glm::vec3 shoulderBody{0}, elbowBody{0}, bendNormalBody{0,0,1};
        float error = 0;
        float maxMeasuredSpeed = 0, maxMeasuredAcceleration = 0;
        unsigned iterations = 0;
        double solveMicroseconds = 0; // Telemetry only; never affects simulation.
        PlanningObservation planning;
        PlanningObservation route; // Admitted route; independent of the latest replacement attempt.
    };
    struct ArmSolution
    {
        std::array<double,6> coordinates{}; // Shoulder XYZ, elbow, wrist XY.
        float error = 0;
        ReachReason reason = ReachReason::LocalNoConvergence;
        unsigned iterations = 0;
    };
    ArmSolution solveArmPosition(const DirectBodyControl& control, const ReachTarget& target,
        glm::vec3 goalBody);

    // Two per-instance motion slots; event storage is proportional to commands in one tick.
    class ArmReach
    {
    public:
        explicit ArmReach(DirectBodyControl& control);
        ~ArmReach();
        void start(std::uint64_t action, std::uint64_t epoch, const ReachTarget& target,
            const glm::mat4& worldFromBody, bool retarget = false, std::uint64_t command = 0);
        void cancel(ArmSide arm, std::uint64_t action);
        void interrupt(ReachReason reason);
        void reset();
        bool owns(JointId joint) const;
        void advance(std::uint64_t tick = 0); // Before final player FK.
        void measure(std::span<const glm::mat4> globals, const glm::mat4& worldFromBody);
        const std::array<ReachObservation,2>& observations() const { return observations_; }
        const std::vector<ReachObservation>& events() const { return events_; }
        void clearEvents() { events_.clear(); }
        using PathValidator = std::function<PathStatus(std::size_t,std::span<const ArmCoordinates>,unsigned)>;
        void setPathValidator(PathValidator validator,std::uint64_t sceneVersion=0) { validator_=std::move(validator); validationSceneVersion_=sceneVersion; }
        using PlanningValidator = std::function<std::unique_ptr<ArmPathCheckJob>(std::size_t,std::span<const ArmCoordinates>)>;
        using PathReporter = std::function<void(PathCheck)>;
        void setPlanningValidator(PlanningValidator factory, PathReporter reporter)
        { planningValidator_=std::move(factory); pathReporter_=std::move(reporter); }
        bool anyActive() const;
    private:
        struct Motion
        {
            std::array<std::array<double,6>,6> polynomial{};
            std::array<double,6> q{}, velocity{}, acceleration{}, lastStepVelocity{};
            std::array<double,6> certifiedEndpoint{}; // Immutable while an old segment finishes a replacement.
            ArmSolution solution;
            std::array<glm::mat4,4> frames;
            unsigned tick = 0, duration = 0, settled = 0;
            bool pending = false, cancelling = false, certifiedRoute = false;
        };
        DirectBodyControl& control_;
        std::array<Motion,2> motion_;
        std::array<ReachObservation,2> observations_;
        std::vector<ReachObservation> events_; // At most a few events per command tick.
        PathValidator validator_;
        std::uint64_t validationSceneVersion_=0;
        std::uint64_t advancingTick_=0;
        struct Planner;
        std::array<std::unique_ptr<Planner>,2> planners_;
        std::array<std::optional<Motion>,2> nextSegments_;
        PlanningValidator planningValidator_;
        PathReporter pathReporter_;
        void requestDetour(std::size_t side, ReachObservation proposal, const ArmSolution& solution, bool retarget);
        void advancePlanning(std::size_t side);
        void endPlanning(std::size_t side, PlanningState state, ReachReason reason);
        Motion restSegment(std::size_t side, const std::array<double,6>& from,
            const std::array<double,6>& to, const ReachObservation& proposal) const;
        static std::vector<ArmCoordinates> samples(const Motion& motion);
        bool plan(std::size_t side);
        PathStatus verify(std::size_t side,unsigned delayTicks=0);
        void finish(std::size_t side, ReachState state, ReachReason reason);
    };
}
