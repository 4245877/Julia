#include "julia/render_opengl/EyePoseLayer.hpp"
#include "julia/render_opengl/GLModel.hpp"

#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string_view>

namespace julia::render_opengl
{
    namespace
    {
        glm::quat orientation(const glm::mat4& matrix)
        {
            glm::mat3 rotation{matrix};
            for (int i = 0; i < 3; ++i) rotation[i] = glm::normalize(rotation[i]);
            return glm::normalize(glm::quat_cast(rotation));
        }

        glm::vec2 safeGaze(glm::vec2 gaze, const render::EyeBehaviorSettings& settings)
        {
            // Authored and visually checked limits: yaw +/-12, pitch +/-8.
            // An ellipse also keeps simultaneous extremes inside those poses.
            gaze.x = std::clamp(gaze.x, -std::min(12.0f, settings.gazeYawDegrees), std::min(12.0f, settings.gazeYawDegrees));
            gaze.y = std::clamp(gaze.y, -std::min(8.0f, settings.gazePitchDegrees), std::min(8.0f, settings.gazePitchDegrees));
            const float radius = glm::length(gaze / glm::vec2{12,8});
            return radius > 1 ? gaze / radius : gaze;
        }
    }

    bool EyePoseLayer::authoredEyes(const AnimationClip* clip)
    {
        if (!clip) return false;
        constexpr std::string_view names[]{"TEST_blink_natural", "TEST_blink_independent", "TEST_blink_gaze", "TEST_blink_head"};
        for (const auto name : names) if (clip->name.ends_with(name)) return true;
        return false;
    }

    EyePoseLayer::EyePoseLayer(const GLModel& model) : model_(model), globals_(model.nodes().size())
    {
        const auto findNode = [&](std::string_view name) {
            for (std::uint32_t i = 0; i < model.nodes().size(); ++i)
                if (model.nodes()[i].name == name) return i;
            return NoModelNode;
        };
        head_ = findNode("head.x_018");
        eyes_ = {findNode("eye.L"), findNode("eye.R")};
        if (head_ == NoModelNode || eyes_[0] == NoModelNode || eyes_[1] == NoModelNode) return;
        for (const auto eye : eyes_) if (model.nodes()[eye].parent != head_) return;
        for (const auto& clip : model.animations())
        {
            bool ownsGaze = false;
            for (const auto& channel : clip.channels)
                for (const auto eye : eyes_)
                    if (channel.nodeIndex == eye)
                        for (const auto& key : channel.rotations)
                            if (std::abs(glm::dot(key.value, model.nodes()[eye].bindTRS.rotation)) < .99999f)
                                ownsGaze = true;
            if (ownsGaze) gazeClips_.push_back(&clip);
        }
        const AnimationClip* source = nullptr;
        for (const auto& clip : model.animations()) if (clip.name.ends_with("TEST_blink_natural")) source = &clip;
        const auto reference = findNode("lid.upper.L.16");
        if (!source || reference == NoModelNode || std::abs(source->ticksPerSecond - 30) > 1e-6 || source->durationTicks < 11) return;
        for (std::uint32_t i = 0; i < model.nodes().size(); ++i)
        {
            const auto& node = model.nodes()[i];
            if (!node.name.starts_with("lid.")) continue;
            const bool left = node.name.find(".L.") != std::string::npos;
            if ((!left && node.name.find(".R.") == std::string::npos) || node.parent != head_) return;
            lids_.push_back({i, left, {}});
        }
        if (lids_.size() != 188 || std::count_if(lids_.begin(), lids_.end(), [](const auto& l) { return l.left; }) != 94) return;

        // Frames 9..12 (ticks 8..11) are the source's first close. The
        // reference's vertical translation is linear in Blender Blink_L;
        // its depth is nonlinear. Read the *actual bake*, not a guessed curve.
        std::vector<glm::mat4> open(model.nodes().size()), closed(open.size()), sampled(open.size());
        sampleAnimation(model, *source, 8.0 / 30, open);
        sampleAnimation(model, *source, 11.0 / 30, closed);
        const float start = open[reference][3].y;
        const float range = closed[reference][3].y - start;
        if (std::abs(range) < 1e-4f) return;
        for (int key = 0; key <= 12; ++key)
        {
            sampleAnimation(model, *source, (8.0 + key * .25) / 30, sampled);
            const float closure = std::clamp((sampled[reference][3].y - start) / range, 0.0f, 1.0f);
            if (!closures_.empty() && closure <= closures_.back()) return;
            closures_.push_back(closure);
            for (auto& lid : lids_)
            {
                // The audited source has translation-only eyelid deformation.
                // Reject another schema rather than silently lose rotation.
                for (int c = 0; c < 3; ++c)
                    if (glm::length(glm::vec3{sampled[lid.node][c] - open[lid.node][c]}) > 1e-4f) return;
                lid.offsets.push_back(glm::vec3{sampled[lid.node][3] - open[lid.node][3]});
            }
        }
        closures_.front() = 0;
        closures_.back() = 1;
        eyeCenter_ = .5f * glm::vec3{model.nodes()[eyes_[0]].globalBindTransform[3] + model.nodes()[eyes_[1]].globalBindTransform[3]};
        available_ = enabled_ = true;
    }

    void EyePoseLayer::setTargetPoint(const glm::vec3& point)
    {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            throw std::invalid_argument("Non-finite gaze target");
        target_ = point;
        if (!targeted_) targetGaze_ = behavior_.output().gazeDegrees;
        targeted_ = true;
    }

    void EyePoseLayer::clearTargetPoint()
    {
        if (targeted_)
        {
            behavior_.releaseGazeFrom(targetGaze_);
        }
        targeted_ = false;
    }

    void EyePoseLayer::update(double seconds)
    {
        elapsed_ = seconds >= 0 && seconds <= .5 ? seconds : 0;
        if (enabled_) behavior_.update(seconds);
    }

    void EyePoseLayer::apply(std::span<glm::mat4> locals, const AnimationClip* baseClip)
    {
        if (!enabled_ || authoredEyes(baseClip)) return;
        if (locals.size() != model_.nodes().size()) throw std::invalid_argument("Eye layer pose size mismatch");
        auto gaze = behavior_.output().gazeDegrees;
        if (targeted_)
        {
            for (std::size_t i = 0; i < locals.size(); ++i)
            {
                const auto parent = model_.nodes()[i].parent;
                globals_[i] = parent == NoModelNode ? locals[i] : globals_[parent] * locals[i];
            }
            const auto headDelta = globals_[head_] * glm::inverse(model_.nodes()[head_].globalBindTransform);
            const auto center = glm::vec3{headDelta * glm::vec4{eyeCenter_,1}};
            const auto direction = glm::inverse(glm::mat3{headDelta}) * (target_ - center);
            glm::vec2 desired{0};
            // A target behind/at the head safely returns forward, never flips.
            if (direction.z > 1e-5f)
                desired = glm::degrees(glm::vec2{std::atan2(direction.x, direction.z),
                    std::atan2(direction.y, std::hypot(direction.x, direction.z))});
            desired = safeGaze(desired, behavior_.settings());
            targetGaze_ = glm::mix(targetGaze_, desired, static_cast<float>(-std::expm1(-elapsed_ / .045)));
            gaze = targetGaze_;
        }
        elapsed_ = 0; // Re-evaluation/seek must not advance a target twice.
        gaze = safeGaze(gaze, behavior_.settings());
        const auto headBind = orientation(model_.nodes()[head_].globalBindTransform);
        const auto delta = glm::inverse(headBind) *
            glm::angleAxis(glm::radians(gaze.x), glm::vec3{0,1,0}) *
            glm::angleAxis(glm::radians(-gaze.y), glm::vec3{1,0,0}) * headBind;
        const bool baseOwnsGaze = std::find(gazeClips_.begin(), gazeClips_.end(), baseClip) != gazeClips_.end();
        for (const auto eye : eyes_)
        {
            if (baseOwnsGaze) continue;
            const auto& node = model_.nodes()[eye];
            const auto& rest = node.bindTRS;
            locals[eye] = gaze == glm::vec2{0} ? node.localTransform :
                glm::translate(glm::mat4{1}, rest.translation) * glm::mat4_cast(delta * rest.rotation) * glm::scale(glm::mat4{1}, rest.scale);
        }
        const auto close = [&](float value, const Lid& lid) {
            const float amount = std::clamp(value, 0.0f, 1.0f);
            if (amount == 0) return glm::vec3{0};
            if (amount == 1) return lid.offsets.back();
            const auto next = std::upper_bound(closures_.begin(), closures_.end(), amount);
            const auto index = static_cast<std::size_t>(next - closures_.begin());
            const float weight = (amount - closures_[index - 1]) / (closures_[index] - closures_[index - 1]);
            return glm::mix(lid.offsets[index - 1], lid.offsets[index], weight);
        };
        const auto& output = behavior_.output();
        for (const auto& lid : lids_)
        {
            locals[lid.node] = model_.nodes()[lid.node].localTransform;
            locals[lid.node][3] += glm::vec4{close(lid.left ? output.blinkLeft : output.blinkRight, lid),0};
        }
    }
}
