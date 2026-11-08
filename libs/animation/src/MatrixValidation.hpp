#pragma once

#include <glm/glm.hpp>
#include <cmath>

namespace julia::animation::detail
{
    inline bool finite(const glm::mat4& matrix)
    {
        for (int c = 0; c < 4; ++c)
            for (int r = 0; r < 4; ++r)
                if (!std::isfinite(matrix[c][r])) return false;
        return true;
    }
}
