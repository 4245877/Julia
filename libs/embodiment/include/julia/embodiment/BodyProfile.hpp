#pragma once

#include "julia/model/ModelAsset.hpp"
#include <span>
#include <cstdint>
#include <string>
#include <vector>

namespace julia::embodiment
{
    // Semantic IDs are neither hierarchy indices nor skin palette indices.
    enum class JointId { Pelvis, Lumbar, Thorax, Chest, Neck, Head,
        LeftClavicle, LeftShoulder, LeftElbow, LeftWrist,
        RightClavicle, RightShoulder, RightElbow, RightWrist,
        LeftHip, LeftKnee, LeftAnkle, RightHip, RightKnee, RightAnkle };
    enum class SegmentId { Pelvis, Lumbar, Thorax, Chest, Neck, Head,
        LeftClavicle, LeftUpperArm, LeftForearm, LeftHand,
        RightClavicle, RightUpperArm, RightForearm, RightHand,
        LeftThigh, LeftShank, LeftFoot, RightThigh, RightShank, RightFoot };
    enum class EffectorId { LeftWrist, LeftPalm, LeftGrip, LeftSole,
        RightWrist, RightPalm, RightGrip, RightSole };

    struct NodeLink { std::string name, parentName; };
    struct JointCoordinate
    {
        std::string name, positiveDirection;
        glm::vec3 axis{1,0,0}; // Unit axis in the proper, right-handed joint frame.
        float lower = 0, upper = 0; // Radians, relative to control zero.
        // All limits in v1 are engineering settings, not anatomical measurements.
    };
    struct JointDefinition
    {
        JointId id{};
        SegmentId segment{};
        std::string name;
        NodeLink node;
        glm::mat4 frameInBodyBind{1}; // Proper rigid frame, metres; never imported TRS.
        std::vector<JointCoordinate> coordinates; // Intrinsic ordered product R0*R1*R2.
        NodeLink segmentEnd;
    };
    struct TwistFollower
    {
        JointId joint{};
        NodeLink node; // Must be a direct child of the controlled node.
        glm::vec3 fraction{1}; // Total rotation fraction, not an additional rotation.
    };
    struct EffectorDefinition
    {
        EffectorId id{};
        std::string name;
        NodeLink node;
        glm::mat4 frameInBodyBind{1}; // Independent of the visual bone orientation.
    };
    struct BodyProfile
    {
        std::string name;
        std::uint64_t rigFingerprint = 0; // Exact hierarchy + bind matrices + skin links.
        glm::mat4 bodyFromResource{1}; // Positive uniform similarity, metres/resource unit.
        std::vector<JointDefinition> joints;
        std::vector<TwistFollower> followers;
        std::vector<EffectorDefinition> effectors;
    };
    enum class BindingError { MissingNode, AmbiguousNode, WrongParent, InvalidProfile, IncompatibleRig };
    struct BindingIssue { BindingError code{}; std::string message; };
    std::uint64_t bodyRigFingerprint(const model::ModelAsset& asset);
    std::vector<BindingIssue> validateBodyProfile(const BodyProfile& profile, const model::ModelAsset& asset);
    BodyProfile raphtaliaBodyProfile();

    // Immutable calibration resolved against one immutable asset. Must outlive controls.
    class BodyRigBinding
    {
    public:
        BodyRigBinding(BodyProfile profile, const model::ModelAsset& asset);
        const BodyProfile& profile() const { return profile_; }
        const model::ModelAsset& asset() const { return asset_; }
        std::size_t jointIndex(JointId id) const;
        std::uint32_t jointNode(JointId id) const;
        // Direct-control frames are rigid. A scaled authored clip may stretch these queries.
        glm::mat4 jointFrame(JointId id, std::span<const glm::mat4> globals) const;
        glm::mat4 effectorFrame(EffectorId id, std::span<const glm::mat4> globals) const;
        // worldFromBody is a proper rigid pose in metres, unrelated to viewer fitting.
        glm::mat4 worldFromResource(const glm::mat4& worldFromBody) const;
        // Stateless visualization of measured coordinates. Checks sizes/finiteness,
        // never clamps to profile limits: physical soft-limit violation stays visible.
        void applyMeasuredCoordinates(std::span<glm::mat4> locals,
            std::span<const glm::vec3> coordinates) const;
    private:
        friend class DirectBodyControl;
        struct Adapter { std::uint32_t node; glm::mat4 parentToFrame, frameToNode; };
        BodyProfile profile_;
        const model::ModelAsset& asset_;
        std::vector<Adapter> joints_, followers_;
        std::vector<std::uint32_t> effectors_;
        glm::mat4 frame(std::uint32_t node, const glm::mat4& bindFrame,
            std::span<const glm::mat4> globals) const;
    };

    enum class LimitPolicy { Reject, Clamp };
    enum class CommandStatus { Applied, Clamped, Rejected };
    struct JointCommandResult { CommandStatus status; float requested, applied; };
    // Per-instance diagnostic state only; no clock, velocity promise, IK or physics.
    class DirectBodyControl
    {
    public:
        explicit DirectBodyControl(const BodyRigBinding& binding);
        const BodyRigBinding& binding() const { return binding_; }
        void setEnabled(bool enabled) { enabled_ = enabled; }
        bool enabled() const { return enabled_; }
        void reset();
        float coordinate(JointId id, std::size_t coordinate) const;
        JointCommandResult setCoordinate(JointId id, std::size_t coordinate, float radians,
            LimitPolicy policy = LimitPolicy::Reject);
        // Enabled: exact bind base replaces the clip; then controlled locals.
        // Disabled: leaves the incoming clip/bind untouched. Eyes run afterwards.
        void apply(std::span<glm::mat4> locals) const;
        std::span<const glm::vec3> coordinates() const { return coordinates_; }
    private:
        friend class BodyInstance;
        // Stateless display reconstruction from validated, interpolated q.
        void applyCoordinates(std::span<glm::mat4> locals, std::span<const glm::vec3> coordinates) const;
        const BodyRigBinding& binding_;
        std::vector<glm::vec3> coordinates_;
        bool enabled_ = false;
    };
}
