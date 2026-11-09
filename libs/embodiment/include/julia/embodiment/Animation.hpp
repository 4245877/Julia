#pragma once

#include "julia/animation/Animation.hpp"
#include "julia/model/ModelAsset.hpp"
#include "julia/animation/Pose.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace julia::embodiment
{
    
    class EyePoseLayer;
    class DirectBodyControl;
    class HandLayer;

    // Transitional source-compatible names; data is owned by the CPU module.
    using animation::NodeTRS;
    using animation::AnimationKey;
    using animation::AnimationChannel;
    using animation::AnimationClip;

    // Same TRS/endpoint-clamp convention as the standalone Raphtalia auditor.
    // Unanimated nodes keep their exact bind matrices; missing components use
    // imported bind TRS. Output is caller-owned, in model node order.
    void sampleAnimation(const model::ModelAsset& model, const AnimationClip& clip,
        double seconds, std::span<glm::mat4> localTransforms);

    enum class PlaybackState { Stopped, Playing, Paused, Finished };

    class AnimationPlayer
    {
    public:
        // Model must remain alive and unchanged for this player's lifetime.
        explicit AnimationPlayer(const model::ModelAsset& model);
        void selectClip(std::size_t index);
        void play();
        void pause();
        void stop(); // Bind base, preserving selection; exact bind when the optional eye layer is disabled.
        void seek(double seconds); // Clamp to endpoints; a stopped player becomes paused.
        // Advances clip and independent procedural eyes. Clip pause/seek does not
        // pause eyes; BodyInstance is the production fixed-tick owner of this API.
        void update(double deltaSeconds);
        void setLooping(bool enabled) { looping_ = enabled; }
        // Optional non-owning, model-specific layer. Null preserves the exact
        // original Animation Player behavior, including stop/bind diagnostics.
        void setEyeLayer(EyePoseLayer* layer);
        // Non-owning diagnostic override: clip/bind -> direct body -> eyes -> FK.
        void setBodyControl(DirectBodyControl* control);
        void setHandLayer(HandLayer* layer) { handLayer_=layer; evaluate(); }

        bool looping() const { return looping_; }
        PlaybackState state() const { return state_; }
        std::size_t clipIndex() const { return clipIndex_; }
        const AnimationClip* clip() const;
        double timeSeconds() const { return timeSeconds_; }
        std::span<const glm::mat4> palette() const { return pose_.palette; }
        std::span<const glm::mat4> localTransforms() const { return pose_.locals; }
        std::span<const glm::mat4> globalTransforms() const { return pose_.globals; }

    private:
        const model::ModelAsset& model_;
        std::size_t clipIndex_ = 0;
        double timeSeconds_ = 0.0;
        bool looping_ = true;
        PlaybackState state_ = PlaybackState::Stopped;
        animation::Pose pose_;
        EyePoseLayer* eyeLayer_ = nullptr;
        DirectBodyControl* bodyControl_ = nullptr;
        HandLayer* handLayer_ = nullptr;
        void evaluate();
    };
}
