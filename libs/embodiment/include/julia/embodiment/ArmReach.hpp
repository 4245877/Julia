#pragma once
#include "julia/embodiment/BodyProfile.hpp"
#include "julia/embodiment/BodyCollision.hpp"
#include <array>
#include <functional>
#include <stdexcept>
#include <optional>
#include <glm/gtc/quaternion.hpp>

namespace julia::embodiment
{
    class ReachTargetUnreachable : public std::invalid_argument { public: using std::invalid_argument::invalid_argument; };
    enum class ArmSide { Left, Right };
    enum class TargetSpace { Body, World };
    enum class ReachTask { PositionOnly, PositionAndOrientation };
    const char* reachTaskName(ReachTask task);
    // Proper frame rotations only. Canonical unit quaternion, rejects nonfinite or
    // norms outside [1e-6,1e6]. Log(target * inverse(actual)), expressed in Body.
    glm::quat normalizedReachOrientation(glm::quat rotation);
    glm::dvec3 orientationResidual(glm::quat target, glm::quat actual);
    enum class ReachState { Idle, Moving, Settling, Succeeded, Failed, Cancelled, Planning };
    enum class MovementStrategy { DirectOnly, AllowDetour };
    enum class RouteSelection { FirstValid, Ranked };
    enum class SelectionCompletion { Running, FirstValid, FiniteSet, RankingLimit, BudgetExhausted,
        CheckIncomplete, InvalidInitial, IkFailed, Cancelled, MotionUnavailable };
    const char* selectionCompletionName(SelectionCompletion completion);
    // A dimensionless direction, NOT a point. Normalized and frozen in Body at
    // command application; World uses inverse root rotation (no translation).
    struct ElbowPreference
    {
        glm::vec3 direction{0,0,1};
        TargetSpace space = TargetSpace::Body;
    };
    struct RankingSettings
    {
        double elbowWeight = 1, travelWeight = .05, durationWeight = .02;
        unsigned additionalTicks = 128; // Work ticks after the first fully Clear route.
        unsigned maxEvaluations = 36; // Shared attempt budget: 4 directs + 32 waypoints.
    };
    enum class ElbowPreferenceStatus { Absent, Defined, StraightArm, ParallelDirection, CollapsedAxis };
    const char* elbowPreferenceStatusName(ElbowPreferenceStatus status);
    struct ElbowEvaluation
    {
        ElbowPreferenceStatus status = ElbowPreferenceStatus::Absent;
        double penalty = 0; // [0,1], lower is better; degenerate geometry costs 1.
        glm::vec3 shoulder{0}, elbow{0}, wrist{0}; // Actual FK in Body metres.
    };
    ElbowEvaluation evaluateElbowPreference(glm::vec3 shoulder,glm::vec3 elbow,glm::vec3 wrist,
        const std::optional<glm::vec3>& directionBody);
    struct RouteScore
    {
        ElbowEvaluation elbow;
        double travel = 0; // Sum over segments of mean |dq| / coordinate range.
        double duration = 0; // Checked motion seconds / 1 second, including waypoint hold.
        double total = 0;
        std::int64_t key = 0; // round(total * 1e9); ties retain earlier Clear route.
    };
    struct RouteEvaluation
    {
        unsigned configuration = 0, candidate = 0, segments = 0;
        RouteScore score;
    };
    enum class PlanningState { Idle, WaitingForStop, CheckingDirect, CheckingFirst, CheckingSecond,
        Paused, RouteReady, Exhausted, BudgetExhausted, CheckIncomplete, InvalidInitial, IkFailed, Cancelled, MotionUnavailable,
        GeneratingIK };
    const char* planningStateName(PlanningState state);
    struct DetourSettings
    {
        unsigned maxCandidates = 32, posesPerTick = 24, queriesPerTick = 2048, maxTotalQueries = 1000000;
        unsigned maxConfigurations = 1; // Explicit 6B compatibility; opt in to 2..4.
        unsigned iterationsPerTick = 16, seedsPerTick = 1;
        unsigned maxIkIterations = 8192, maxSeedAttempts = 30, maxTotalPoses = 32768;
    };
    enum class ConfigurationOutcome { Pending, Blocked, QuotaExhausted, CheckIncomplete, MotionUnavailable, CandidatesExhausted, Accepted, Verified };
    const char* configurationOutcomeName(ConfigurationOutcome outcome);
    struct ConfigurationObservation
    {
        ArmCoordinates coordinates{};
        float error = 0;
        float angularError = 0;
        PathStatus directStatus = PathStatus::NotChecked, lastStatus = PathStatus::NotChecked;
        ConfigurationOutcome outcome = ConfigurationOutcome::Pending;
        unsigned queries = 0, poses = 0, candidates = 0;
        std::uint64_t object = 0;
        std::uint32_t bodyForm = 0, otherBodyForm = 0;
        double gap = 0;
        std::string detail;
    };
    enum class IkSeedOutcome { None, Accepted, Duplicate, LocalFailure, FkRejected };
    const char* ikSeedOutcomeName(IkSeedOutcome outcome);
    enum class IkSearchCompletion { Running, ConfigurationLimit, SeedSetExhausted, BudgetExhausted };
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
        PlanningState stage = PlanningState::Idle; // Remains meaningful while Paused.
        PlanningState budgetStage = PlanningState::Idle;
        unsigned seedAttempts = 0, seedsProcessed = 0, ikIterations = 0, duplicates = 0, failedSeeds = 0, fkChecks = 0;
        unsigned ikEvaluations = 0, tickIkEvaluations = 0; // Rigid chain FK, including Jacobian/line search.
        unsigned configurations = 0, currentConfiguration = 0, selectedConfiguration = 0;
        unsigned tickIterations = 0, tickSeeds = 0, tickQueries = 0, tickPoses = 0, tickCandidates = 0;
        unsigned budgetConfigurations = 0, budgetCandidates = 0;
        ArmCoordinates finalCoordinates{};
        std::array<ConfigurationObservation,4> solutions{};
        std::string budgetReason;
        IkSeedOutcome lastSeedOutcome = IkSeedOutcome::None;
        IkSearchCompletion ikCompletion = IkSearchCompletion::Running;
        double ikMicroseconds = 0, pathMicroseconds = 0, segmentMicroseconds = 0;
        RouteSelection selection = RouteSelection::FirstValid;
        SelectionCompletion completion = SelectionCompletion::Running;
        RankingSettings ranking;
        std::optional<glm::vec3> elbowDirectionBody;
        RouteScore score; // Best fully checked route, never an unverified endpoint.
        unsigned scoreEvaluations = 0, tickScoreEvaluations = 0, firstClearTick = 0;
        std::vector<RouteEvaluation> evaluations; // Value-owned, bounded by maxEvaluations <= 36.
        double scoreMicroseconds = 0;
    };
    // JointLimited is a constrained LOCAL result, never a global infeasibility proof.
    enum class ReachReason { None, Reached, GeometricRange, JointLimited, LocalNoConvergence,
        MotionPlanUnavailable, UserCancelled, Superseded, ModeChanged, CoordinatesReset, BodyReset, RootChanged,
        PathBlocked, InitialIntersection, CheckIncomplete, DetourExhausted, PlanningBudgetExhausted, OrientationNotReached, GeometryContextChanged };
    const char* reachReasonName(ReachReason reason);
    const char* reachStateName(ReachState state);
    struct ReachTarget
    {
        ArmSide arm = ArmSide::Left;
        EffectorId effector = EffectorId::LeftWrist;
        glm::vec3 position{0}; // Metres in space.
        TargetSpace space = TargetSpace::Body;
        float tolerance = .002f; // Measured AFTER FK, not solver prediction.
        float maxSpeed = 1.2f; // Each controlled coordinate, rad/s.
        float maxAcceleration = 4.0f; // Discrete second difference, rad/s^2.
        float settleSeconds = .1f;
        MovementStrategy strategy = MovementStrategy::DirectOnly;
        DetourSettings detour;
        RouteSelection selection = RouteSelection::FirstValid;
        std::optional<ElbowPreference> elbowPreference;
        RankingSettings ranking;
        ReachTask task = ReachTask::PositionOnly;
        glm::quat orientation{1,0,0,0}; // spaceFromCalibratedEffector, column vectors.
        float angularTolerance = .01745329252f; // 1 degree, radians; final pose only.
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
        glm::quat goalOrientationBody{1,0,0,0}, actualOrientationBody{1,0,0,0}, actualOrientationWorld{1,0,0,0};
        float angularError = 0; // Radians; diagnostic only in PositionOnly.
        std::optional<glm::vec3> elbowDirectionBody;
        ElbowEvaluation actualElbow; // Separate from original effector success.
        float maxMeasuredSpeed = 0, maxMeasuredAcceleration = 0;
        unsigned iterations = 0;
        unsigned evaluations = 0;
        double solveMicroseconds = 0; // Telemetry only; never affects simulation.
        PlanningObservation planning;
        PlanningObservation route; // Admitted route; independent of the latest replacement attempt.
    };
    struct ArmSolution
    {
        std::array<double,6> coordinates{}; // Shoulder[0..2], elbow[0], wrist[0..1], profile order.
        float error = 0;
        float angularError = 0;
        ReachReason reason = ReachReason::LocalNoConvergence;
        unsigned iterations = 0;
        unsigned evaluations = 0;
    };
    // Low-level goal is in Body. World full targets require explicit frozen Body rotation.
    ArmSolution solveArmPosition(const DirectBodyControl& control, const ReachTarget& target,
        glm::vec3 goalBody, std::optional<glm::quat> goalOrientationBody = std::nullopt);

    // Bounded, resumable production multi-start IK. Owns frozen control coordinates;
    // binding/asset must outlive it. A call never exceeds its iteration/start limits.
    class ArmPositionSearchJob
    {
    public:
        ArmPositionSearchJob(const DirectBodyControl& control, const ReachTarget& target, glm::vec3 goalBody,
            std::optional<glm::quat> goalOrientationBody = std::nullopt);
        ~ArmPositionSearchJob();
        bool advance(unsigned iterations, unsigned seeds);
        bool done() const;
        bool budgetExhausted() const;
        const PlanningObservation& observation() const;
        const std::vector<ArmSolution>& solutions() const;
        std::size_t scratchBytes() const;
    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };

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
        // Single-writer dependency gate, run before admission and segment execution.
        std::function<bool(std::size_t)> contextValid;
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
        void requestDetour(std::size_t side, ReachObservation proposal, bool retarget);
        void advancePlanning(std::size_t side);
        void endPlanning(std::size_t side, PlanningState state, ReachReason reason);
        void admitBest(std::size_t side, SelectionCompletion completion);
        Motion restSegment(std::size_t side, const std::array<double,6>& from,
            const std::array<double,6>& to, const ReachObservation& proposal) const;
        static std::vector<ArmCoordinates> samples(const Motion& motion);
        bool plan(std::size_t side);
        PathStatus verify(std::size_t side,unsigned delayTicks=0);
        void finish(std::size_t side, ReachState state, ReachReason reason);
    };
}
