#pragma once
#include "julia/embodiment/BodyCollision.hpp"
#include <array>
#include <memory>
#include <optional>

namespace julia::embodiment
{
    enum class LegSide { Left, Right };
    using LegCoordinates = std::array<float,6>;
    std::array<JointId,3> legJoints(LegSide side);
    struct SoleContact
    {
        glm::mat4 soleFromContact{1};
        std::array<glm::vec3,4> points{}; // Coplanar rectangle, contact-local metres.
        unsigned shoeVertices=0, lowVertices=0;
        float lowBand=.006f; // Calibration sampling band, NOT contact tolerance.
        glm::vec3 sampledMin{0}, sampledMax{0}; // In provisional sole frame.
    };
    std::array<SoleContact,2> calibrateSoleContacts(const BodyRigBinding& binding);
    std::span<const ColliderCalibration> lowerBodyColliderCalibration();
    std::vector<BodyCollider> lowerBodyColliders(const BodyRigBinding& binding,
        std::span<const glm::mat4> globals,const glm::mat4& root,const std::array<SoleContact,2>& contacts);
    struct LegSolution
    {
        bool success=false;
        LegCoordinates coordinates{};
        float positionError=0, angularError=0;
        unsigned iterations=0, evaluations=0;
        std::string reason;
    };
    // Root and target are proper rigid metric world frames. Local failure is not
    // proof of global infeasibility. Warm start only, bounded to 80 iterations.
    LegSolution solveLegPose(const DirectBodyControl& control,LegSide side,const SoleContact& contact,
        const glm::mat4& worldFromBody,const glm::mat4& worldFromContact);

    struct SupportSurface { std::uint64_t id=1, version=1; float height=0; bool available=true; };
    struct SetSupportSurface { SupportSurface surface; };
    struct EnableSupport {};
    struct AlignSupport {};
    struct MoveSupportRoot { glm::vec3 worldPosition{0}; };
    struct CancelSupportMotion {};
    struct DisableSupport {};
    enum class SupportPhase { Disabled, Unaligned, Planning, Aligning, Holding, Moving, Lost };
    const char* supportPhaseName(SupportPhase phase);
    enum class SupportKind { None, KinematicSupport };
    struct SupportAnchor
    {
        std::uint64_t surface=0, version=0;
        glm::mat4 surfaceFromSole{1}, desiredWorld{1}, actualWorld{1};
        bool bound=false;
        float surfaceHeight=0;
        float positionError=0, angularError=0, minGap=0, maxGap=0;
        std::array<glm::vec3,4> worldPoints{};
        std::array<glm::vec3,3> chain{}; // Hip, knee, ankle in world.
    };
    struct SupportObservation
    {
        bool enabled=false, cancellationPending=false, disablePending=false;
        SupportPhase phase=SupportPhase::Disabled;
        SupportKind kind=SupportKind::None;
        SupportSurface surface;
        std::array<SupportAnchor,2> anchors{};
        glm::vec3 requestedRoot{0};
        std::uint64_t action=0, attempt=0, epoch=0;
        unsigned plannedStates=0, executedStates=0, iterations=0, evaluations=0, fkChecks=0;
        unsigned tickIterations=0, tickEvaluations=0, tickFkChecks=0;
        std::size_t storageBytes=0;
        double planningMicroseconds=0, tickPlanningMicroseconds=0;
        std::string result, reason;
        // Forces, masses and physical stability are deliberately unavailable.
    };
    inline constexpr float SupportPositionTolerance=.0005f;
    inline constexpr float SupportAngularTolerance=.0034906585f; // 0.2 degrees.
    inline constexpr float SupportSurfaceTolerance=.0005f;
    inline constexpr float SupportRootSpeed=.03f, SupportRootAcceleration=.06f;
    inline constexpr float SupportJointSpeed=.8f, SupportJointAcceleration=6.f;
    inline constexpr unsigned SupportMotionTicks=360;

    class LegSupport
    {
    public:
        explicit LegSupport(DirectBodyControl& control);
        ~LegSupport();
        bool enabled() const { return observation_.enabled; }
        const SupportObservation& observation() const { return observation_; }
        const auto& contacts() const { return contacts_; }
        void beginTick();
        void setSurface(SupportSurface surface);
        void enable(const glm::mat4& root);
        void plan(std::uint64_t action,std::uint64_t epoch,const glm::mat4& root,
            std::optional<glm::vec3> position); // nullopt = explicit alignment.
        void cancel(bool disable=false);
        void interrupt(const char* reason); // Explicit diagnostic discontinuity.
        void advance(glm::vec3& rootPosition);
        void measure(std::span<const glm::mat4> globals,const glm::mat4& root);
        SupportObservation displayObservation(SupportObservation observation,
            std::span<const glm::mat4> globals,const glm::mat4& root) const;
    private:
        struct Job;
        struct Sample { glm::vec3 root{0}; std::array<LegCoordinates,2> q{}; };
        DirectBodyControl& control_;
        std::array<SoleContact,2> contacts_;
        SupportObservation observation_;
        std::unique_ptr<Job> job_;
        std::vector<Sample> motion_;
        std::size_t cursor_=0;
        bool aligning_=false, lost_=false;
        glm::vec3 origin_{0};
        void fail(const char* reason);
    };
}
