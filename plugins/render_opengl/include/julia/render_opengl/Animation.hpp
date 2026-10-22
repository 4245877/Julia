#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace julia::render_opengl
{
    class GLModel;
    class EyePoseLayer;

    struct NodeTRS
    {
        glm::vec3 translation{0.0f};
        glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
        glm::vec3 scale{1.0f};
    };

    template<class T> struct AnimationKey
    {
        double ticks = 0.0;
        T value{};
    };

    struct AnimationChannel
    {
        std::uint32_t nodeIndex = 0;
        std::vector<AnimationKey<glm::vec3>> translations;
        std::vector<AnimationKey<glm::quat>> rotations;
        std::vector<AnimationKey<glm::vec3>> scales;
    };

    struct AnimationClip
    {
        std::string name;
        double durationTicks = 0.0;
        double ticksPerSecond = 25.0;
        bool usedDefaultTicksPerSecond = false;
        std::vector<AnimationChannel> channels;

        double durationSeconds() const { return durationTicks / ticksPerSecond; }
    };

    // Same TRS/endpoint-clamp convention as the standalone Raphtalia auditor.
    // Unanimated nodes keep their exact bind matrices; missing components use
    // imported bind TRS. Output is caller-owned, in model node order.
    void sampleAnimation(const GLModel& model, const AnimationClip& clip,
        double seconds, std::span<glm::mat4> localTransforms);

    enum class PlaybackState { Stopped, Playing, Paused, Finished };

    class AnimationPlayer
    {
    public:
        // Model must remain alive and unchanged for this player's lifetime.
        explicit AnimationPlayer(const GLModel& model);
        void selectClip(std::size_t index);
        void play();
        void pause();
        void stop(); // Bind base, preserving selection; exact bind when the optional eye layer is disabled.
        void seek(double seconds); // Clamp to endpoints; a stopped player becomes paused.
        void update(double deltaSeconds);
        void setLooping(bool enabled) { looping_ = enabled; }
        // Optional non-owning, model-specific layer. Null preserves the exact
        // original Animation Player behavior, including stop/bind diagnostics.
        void setEyeLayer(EyePoseLayer* layer);

        bool looping() const { return looping_; }
        PlaybackState state() const { return state_; }
        std::size_t clipIndex() const { return clipIndex_; }
        const AnimationClip* clip() const;
        double timeSeconds() const { return timeSeconds_; }
        std::span<const glm::mat4> palette() const { return palette_; }
        std::span<const glm::mat4> localTransforms() const { return locals_; }

    private:
        const GLModel& model_;
        std::size_t clipIndex_ = 0;
        double timeSeconds_ = 0.0;
        bool looping_ = true;
        PlaybackState state_ = PlaybackState::Stopped;
        std::vector<glm::mat4> locals_, globals_, palette_;
        EyePoseLayer* eyeLayer_ = nullptr;
        void evaluate();
    };
}
