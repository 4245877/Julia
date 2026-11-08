#pragma once
#include "julia/embodiment/BodyProfile.hpp"
#include <array>

namespace julia::embodiment
{
    enum class ArmSide { Left, Right };
    enum class TargetSpace { Body, World };
    enum class ReachState { Idle, Moving, Settling, Succeeded, Failed, Cancelled };
    // JointLimited is a constrained LOCAL result, never a global infeasibility proof.
    enum class ReachReason { None, Reached, GeometricRange, JointLimited, LocalNoConvergence,
        MotionPlanUnavailable, UserCancelled, Superseded, ModeChanged, CoordinatesReset, BodyReset, RootChanged };
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
        ReachTarget target;
        glm::vec3 goalBody{0}, referenceBody{0}, actualBody{0}, actualWorld{0};
        glm::vec3 shoulderBody{0}, elbowBody{0}, bendNormalBody{0,0,1};
        float error = 0;
        float maxMeasuredSpeed = 0, maxMeasuredAcceleration = 0;
        unsigned iterations = 0;
        double solveMicroseconds = 0; // Telemetry only; never affects simulation.
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
        void start(std::uint64_t action, std::uint64_t epoch, const ReachTarget& target,
            const glm::mat4& worldFromBody, bool retarget = false);
        void cancel(ArmSide arm, std::uint64_t action);
        void interrupt(ReachReason reason);
        void reset();
        bool owns(JointId joint) const;
        void advance(); // Before final player FK.
        void measure(std::span<const glm::mat4> globals, const glm::mat4& worldFromBody);
        const std::array<ReachObservation,2>& observations() const { return observations_; }
        const std::vector<ReachObservation>& events() const { return events_; }
        void clearEvents() { events_.clear(); }
    private:
        struct Motion
        {
            std::array<std::array<double,6>,6> polynomial{};
            std::array<double,6> q{}, velocity{}, acceleration{}, lastStepVelocity{};
            ArmSolution solution;
            std::array<glm::mat4,4> frames;
            unsigned tick = 0, duration = 0, settled = 0;
            bool pending = false, cancelling = false;
        };
        DirectBodyControl& control_;
        std::array<Motion,2> motion_;
        std::array<ReachObservation,2> observations_;
        std::vector<ReachObservation> events_; // At most a few events per command tick.
        bool plan(std::size_t side);
        void finish(std::size_t side, ReachState state, ReachReason reason);
    };
}
