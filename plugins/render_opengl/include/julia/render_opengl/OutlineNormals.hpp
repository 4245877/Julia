#pragma once

#include <glm/glm.hpp>

#include <cstdint>
#include <span>
#include <vector>

namespace julia::render_opengl
{
    struct GLVertex;

    // Smooth only the inverted hull: UV/hard-normal duplicates share an
    // angle-weighted direction when their positions and skinning agree.
    // Does not modify surface normals, geometry, UVs, or bone influences.
    // Invalid triangle indices/counts and non-finite positions are rejected.
    // Degenerate/unreferenced vertices fall back to a finite surface normal.
    std::vector<glm::vec3> buildOutlineNormals(
        std::span<const GLVertex> vertices,
        std::span<const std::uint32_t> indices);
}
