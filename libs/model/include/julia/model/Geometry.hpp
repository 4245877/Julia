#pragma once
#include "julia/model/Bounds.hpp"
#include <cstdint>

namespace julia::model
{
    struct Vertex
    {
        glm::vec3 position{0.0f};
        glm::vec3 normal{0.0f, 1.0f, 0.0f};
        glm::vec2 texCoord{0.0f};

        // Four strongest normalized influences, shared with model.vert.
        glm::ivec4 boneIds{-1, -1, -1, -1};
        glm::vec4 boneWeights{0.0f};
        // Geometric outline normal, welded across UV and shading-normal seams.
        // Filled once by GLMesh; authored lighting normals remain untouched.
        glm::vec3 outlineNormal{0.0f};
    };

    enum class AlphaMode { Opaque, Mask, Blend };

    // Shader contract: Generic=0, Skin=1, Hair=2, Eye=3, Metal=4,
    // Fabric=5, Leather=6, Ink=7. Profiles affect lighting, never base color.
    enum class ToonMaterialKind
    {
        Generic = 0, Skin = 1, Hair = 2, Eye = 3,
        Metal = 4, Fabric = 5, Leather = 6, Ink = 7
    };

    struct MaterialProperties
    {
        glm::vec3 diffuseColor{0.8f, 0.8f, 0.8f};
        float lightResponse = 0.80f;
        float specularStrength = 0.004f;
        float rimStrength = 0.032f;
        float opacity = 1.0f;
        float alphaCutoff = 0.5f;
        AlphaMode alphaMode = AlphaMode::Opaque;
        ToonMaterialKind toonKind = ToonMaterialKind::Generic;
        float specularPower = 80.0f;
        // Half-width of the painted terminator in N.L space, before pixel AA.
        // Wider skin/fabric transitions hide tessellation without blurring UVs.
        float shadowSoftness = 0.065f;
        // Groom axis in imported scene space; transformed with the skeleton.
        // Projected onto the surface for the hair highlight, never camera space.
        glm::vec3 strandDirection{0.0f, 1.0f, 0.0f};
        // Inverted hull is unsuitable for thin eye/decal/ink layers.
        float outlineScale = 1.0f;
    };

}
