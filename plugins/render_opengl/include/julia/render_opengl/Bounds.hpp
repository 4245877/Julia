#pragma once

#include <glm/glm.hpp>

#include <algorithm>
#include <limits>

namespace julia::render_opengl
{
    struct Bounds
    {
        glm::vec3 min{
            std::numeric_limits<float>::max(),
            std::numeric_limits<float>::max(),
            std::numeric_limits<float>::max()
        };

        glm::vec3 max{
            std::numeric_limits<float>::lowest(),
            std::numeric_limits<float>::lowest(),
            std::numeric_limits<float>::lowest()
        };

        bool valid = false;

        void expand(const glm::vec3& point)
        {
            if (!valid)
            {
                min = point;
                max = point;
                valid = true;
                return;
            }

            min = glm::min(min, point);
            max = glm::max(max, point);
        }

        void expand(const Bounds& other)
        {
            if (!other.valid)
            {
                return;
            }

            expand(other.min);
            expand(other.max);
        }

        glm::vec3 size() const
        {
            if (!valid)
            {
                return glm::vec3{0.0f};
            }

            return max - min;
        }

        glm::vec3 center() const
        {
            if (!valid)
            {
                return glm::vec3{0.0f};
            }

            return (min + max) * 0.5f;
        }
    };
}