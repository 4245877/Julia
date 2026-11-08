#pragma once
#include "julia/embodiment/Animation.hpp"
#include "julia/embodiment/BodyProfile.hpp"
#include "julia/embodiment/ArmReach.hpp"
#include "julia/embodiment/EyePoseLayer.hpp"
#include "julia/runtime/FixedStepScheduler.hpp"
#include <memory>
#include <unordered_set>
#include <variant>

namespace julia::embodiment
{
    using BodyId = std::uint64_t;
    struct RootPose { glm::vec3 position{0}; glm::quat rotation{1,0,0,0}; };
    struct SetCoordinate { JointId joint{}; std::size_t coordinate = 0; float radians = 0;
        LimitPolicy policy = LimitPolicy::Reject; bool relative = false; };
    struct ResetCoordinates {};
    struct SetDirectMode { bool enabled; };
    struct ResetBody {};
    struct SetRoot { RootPose pose; };
    enum class ClipOperation { Select, Play, Pause, Stop, Seek, Loop, TogglePlay, ToggleLoop, Step, Next };
    struct ClipCommand { ClipOperation operation; double value = 0; };
    enum class EyeOperation { Enable, Natural, AutoBlink, AutoGaze, Blink, Gaze, Neutral,
        Target, ClearTarget, ResetSeed, BlinkInterval, ManualBlink };
    // Gaze in degrees; Target in resource coordinates; manual blink in x/y [0,1].
    struct EyeCommand { EyeOperation operation; glm::vec3 value{0}; };
    using BodyOperation = std::variant<SetCoordinate, ResetCoordinates, SetDirectMode, ResetBody,
        SetRoot, ClipCommand, EyeCommand, StartReach, RetargetReach, CancelReach>;
    struct BodyCommand
    {
        std::uint64_t id, target, tick, epoch;
        BodyOperation operation;
    };
    enum class BodyCommandStatus { Queued, Applied, Clamped, Rejected, Cancelled };
    struct BodyCommandResult
    {
        std::uint64_t id, tick;
        BodyCommandStatus status;
        std::string reason;
        std::uint64_t epoch = 0; // ID namespace, including reset acknowledgement/cancellations.
    };
    struct BodySnapshot
    {
        std::uint64_t tick = 0, epoch = 0;
        double time = 0, clipTime = 0;
        std::size_t clipIndex = 0;
        PlaybackState playback = PlaybackState::Stopped;
        bool direct = false, eyesEnabled = false;
        RootPose root;
        std::vector<glm::vec3> coordinates; // Diagnostic q, never inferred from arbitrary clips.
        render::EyeBehaviorOutput eyes;
        std::array<ReachObservation,2> reaches;
        // Base clip components only; exact matrices for all nonanimated nodes.
        std::vector<NodeTRS> components;
        std::vector<bool> animated;
        animation::Pose pose;
        explicit BodySnapshot(const animation::Skeleton& skeleton) : components(skeleton.nodes().size()),
            animated(skeleton.nodes().size()), pose(skeleton) {}
    };
    struct BodyDisplay
    {
        std::uint64_t fromTick = 0, toTick = 0;
        double time = 0;
        RootPose root;
        bool direct = false;
        std::vector<glm::vec3> coordinates;
        animation::Pose pose;
        explicit BodyDisplay(const animation::Skeleton& skeleton) : pose(skeleton) {}
    };
    // Single writer; all exposed state is const. Asset is held alive, binding is owned.
    // Noncopyable: internal component references/pointers always address this instance.
    class BodyInstance
    {
    public:
        BodyInstance(BodyId id, std::shared_ptr<const model::ModelAsset> asset,
            const BodyProfile* profile = nullptr, bool eyesEnabled = true);
        BodyInstance(const BodyInstance&) = delete;
        BodyInstance& operator=(const BodyInstance&) = delete;
        BodyCommandResult submit(BodyCommand command);
        BodyCommandResult enqueue(BodyOperation operation); // Local input: next tick, monotonically assigned ID.
        void step();
        void collapseHistory();
        BodyDisplay display(double alpha) const; // No simulation mutation, including target gaze.
        const BodySnapshot& snapshot() const { return current_; }
        const BodySnapshot& previousSnapshot() const { return previous_; }
        const auto& results() const { return results_; }
        const std::vector<ReachObservation>& reachEvents() const;
        std::size_t pendingCommands() const { return commands_.size(); }
        std::size_t commandHistorySize() const { return seen_.size(); } // Grows until ResetBody.
        BodyId id() const { return id_; }
        const model::ModelAsset& asset() const { return *asset_; }
        const AnimationPlayer& player() const { return player_; }
        const EyePoseLayer& eyes() const { return eyes_; }
        const DirectBodyControl* control() const { return control_.get(); }
        glm::mat4 resourceRoot(const RootPose& root) const;
    private:
        BodyId id_;
        std::shared_ptr<const model::ModelAsset> asset_;
        std::unique_ptr<BodyRigBinding> binding_;
        std::unique_ptr<DirectBodyControl> control_;
        std::unique_ptr<ArmReach> reach_;
        EyePoseLayer eyes_;
        AnimationPlayer player_;
        RootPose root_;
        bool initialEyes_, discontinuity_ = false;
        std::uint64_t tick_ = 0, epoch_ = 0, nextId_ = 1;
        std::vector<BodyCommand> commands_;
        std::unordered_set<std::uint64_t> seen_;
        std::vector<BodyCommandResult> results_;
        BodySnapshot previous_, current_;
        void capture();
        BodyCommandStatus apply(const BodyOperation& operation, std::uint64_t commandId);
    };
}
