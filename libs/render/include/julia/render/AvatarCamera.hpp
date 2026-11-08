#pragma once

#include "julia/render/OrbitCamera.hpp"

namespace julia::render
{
    // A stable front portrait in scene space. Framing is independent of window
    // pixels, input, skeleton evaluation, and the full-window orbit camera.
    class AvatarCamera
    {
    public:
        explicit AvatarCamera(CameraSubject subject, std::optional<glm::vec3> waist = {});
        float aspect() const { return width_ / height_; }
        float frameHeight() const { return height_; }
        float waistHeight() const { return center_.y - height_ * 0.5f; }
        glm::vec3 target(glm::vec3 rootOffset = {}) const { return center_ + rootOffset; }
        glm::vec3 position(glm::vec3 rootOffset = {}) const;

    private:
        glm::vec3 center_{};
        float width_ = 1, height_ = 1, distance_ = 1;
    };
}
