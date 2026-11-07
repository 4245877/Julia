#pragma once

#include "julia/render_opengl/GLModel.hpp"

#include <filesystem>

namespace julia::render_opengl
{
    struct GLModelLoadOptions
    {
        // Compatibility for legacy assets without texture references. Material
        // names alone cannot establish which atlas matches the mesh UV layout,
        // so guessing body_color/hair_color/shield_color is explicitly opt-in.
        bool useLegacyMaterialTextureFallback = false;

        // Opt-in repair for assets exported with per-face normals. Rebuild at
        // import time with angle-weighted connected rest-surface fans across UV
        // and skin-weight splits, retaining creases >= 45 degrees. Vertex copies
        // keep their own deformation. Authored normals stay untouched by default.
        bool rebuildSmoothNormals = false;
    };

    class GLModelLoader
    {
    public:
        // Requires a current initialized OpenGL context. Geometry is baked to
        // imported scene space; nodes and adjusted inverse binds are retained
        // for GLModel::skinningMatrices(). Skeletal clips/keys are deep-copied
        // before freeing Assimp; sampling/playback belongs to AnimationPlayer.
        static GLModel loadFromFile(const std::filesystem::path& path,
            GLModelLoadOptions options = {});
    };
}
