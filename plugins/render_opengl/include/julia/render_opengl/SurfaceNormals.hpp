#pragma once

#include <cstdint>
#include <vector>

namespace julia::render_opengl
{
    struct GLVertex;

    // Opt-in repair for faceted exports. Angle-weighted normals follow connected
    // rest-surface fans across UV and skin-weight splits. Each copy retains its
    // own deformation; only rest normals are shared. Creases at or above the
    // supplied angle and non-manifold edges remain hard. Vertices are split
    // when required; positions, UVs, weights and triangle order are kept.
    void rebuildSurfaceNormals(std::vector<GLVertex>& vertices,
        std::vector<std::uint32_t>& indices, float creaseAngleDegrees = 45.0f);
}
