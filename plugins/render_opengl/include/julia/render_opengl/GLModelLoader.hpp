#pragma once

#include "julia/render_opengl/GLModel.hpp"

#include <filesystem>
#include "julia/model_import/ModelImporter.hpp"

namespace julia::render_opengl
{
    using GLModelLoadOptions = model_import::ModelImportOptions;

    class GLModelLoader
    {
    public:
        // Creation and destruction of all returned GPU owners require a current
        // initialized context. upload() performs no file access or Assimp work.
        static GLModel upload(std::shared_ptr<const model::ModelAsset> asset);
        // Convenience adapter: CPU import followed by the same upload operation.
        static GLModel loadFromFile(const std::filesystem::path& path,
            GLModelLoadOptions options = {});
    };
}
