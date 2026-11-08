#include "julia/render/AvatarCamera.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace julia::render
{
    AvatarCamera::AvatarCamera(CameraSubject subject, std::optional<glm::vec3> waist)
    {
        const auto finite = [](glm::vec3 v) {
            return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
        };
        const auto size = subject.max - subject.min;
        if (!finite(subject.min) || !finite(subject.max) ||
            glm::any(glm::lessThan(size, glm::vec3{0})) || size.y < 1e-6f)
            throw std::invalid_argument("Avatar requires finite, nondegenerate Y-up bounds");
        float bottom = subject.min.y + size.y * 0.51f;
        if (waist && finite(*waist) && waist->y > subject.min.y + size.y * 0.35f &&
            waist->y < subject.min.y + size.y * 0.66f)
            bottom = waist->y;
        // Headroom includes ears/hair from the actual model bounds and a small
        // animation margin. The bottom image boundary is the waist plane at
        // every depth: no scissor, removed geometry, or near-plane body cut.
        const float top = subject.max.y + size.y * 0.045f;
        height_ = top - bottom;
        width_ = std::max(size.x + size.y * 0.08f, height_ * 0.65f);
        center_ = (subject.min + subject.max) * 0.5f;
        center_.y = (top + bottom) * 0.5f;
        distance_ = size.z * 0.5f + size.y * 2.0f;
    }

    glm::vec3 AvatarCamera::position(glm::vec3 rootOffset) const
    {
        return target(rootOffset) + glm::vec3{0, 0, distance_};
    }
}
