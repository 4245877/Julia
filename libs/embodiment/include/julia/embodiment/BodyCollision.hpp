#pragma once
#include "julia/embodiment/BodyProfile.hpp"
#include "julia/world/CollisionScene.hpp"
#include <array>
#include <memory>

namespace julia::embodiment
{
    using ArmCoordinates = std::array<float,6>;
    enum class PathStatus { Disabled, Clear, Blocked, InitialIntersection, Incomplete, Unreachable, Rejected, NotChecked, Invalidated };
    const char* pathStatusName(PathStatus status);
    struct CollisionSettings
    {
        double margin = .002, numericSlack = .00003; // metres; engineering guard for float FK.
        unsigned maxPoses = 8192, maxQueries = 250000, maxDepth = 18;
        world::CollisionFilter bodyFilter{2,~0u};
    };
    struct BodyCollider
    {
        std::uint32_t id;
        SegmentId segment;
        world::Capsule capsule; // World metres, generated from actual calibrated FK.
    };
    // Engineering calibration, independently inspectable from bind/import calibration.
    struct ColliderCalibration
    {
        std::uint32_t id;
        SegmentId segment;
        JointId first, second;
        float from, to, radius;
        glm::vec3 localA{0}, localB{0}; // Used if first == second.
    };
    std::span<const ColliderCalibration> bodyColliderCalibration();
    std::vector<BodyCollider> bodyColliders(const BodyRigBinding& binding,
        std::span<const glm::mat4> globals, const glm::mat4& worldFromBody);
    bool checksSelfPair(std::uint32_t a, std::uint32_t b);
    struct PathCheck
    {
        PathStatus status = PathStatus::Disabled;
        std::uint64_t sceneVersion = 0, command = 0, segmentStartTick = 0;
        std::uint32_t bodyForm = 0, otherBodyForm = 0;
        std::uint64_t object = 0; // Distinct namespace from body forms.
        world::Clearance contact;
        double fromTick = 0, toTick = 0, microseconds = 0;
        unsigned poses = 0, queries = 0;
        std::size_t scratchBytes = 0;
        std::string detail;
        std::vector<std::array<glm::vec3,3>> trace; // Bounded world-space upper/fore/hand centres.
    };
    void validateCollisionSettings(const CollisionSettings& settings);
    // Immutable path/control/scene snapshot, resumable at individual pair queries.
    // The calibration binding and its asset must outlive this job. No body mutation.
    // Paused is NotChecked; only a completed job can return Clear or Incomplete.
    class ArmPathCheckJob
    {
    public:
        ArmPathCheckJob(const DirectBodyControl& control, std::size_t side,
            std::span<const ArmCoordinates> tickCoordinates, const glm::mat4& worldFromBody,
            const world::CollisionScene& scene, const CollisionSettings& settings);
        ~ArmPathCheckJob();
        ArmPathCheckJob(ArmPathCheckJob&&) noexcept;
        ArmPathCheckJob& operator=(ArmPathCheckJob&&) noexcept;
        ArmPathCheckJob(const ArmPathCheckJob&) = delete;
        ArmPathCheckJob& operator=(const ArmPathCheckJob&) = delete;
        // Limits newly evaluated poses and individual geometric queries in this call.
        // Zero in either dimension pauses; counters/time are cumulative across calls.
        bool advance(unsigned maxPoses, unsigned maxQueries);
        bool done() const;
        const PathCheck& result() const;
    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
    PathCheck checkArmPath(const DirectBodyControl& control, std::size_t side,
        std::span<const ArmCoordinates> tickCoordinates, const glm::mat4& worldFromBody,
        const world::CollisionScene& scene, const CollisionSettings& settings);
}
