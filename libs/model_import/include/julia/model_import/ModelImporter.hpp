#pragma once
#include "julia/model/ModelAsset.hpp"
#include <filesystem>
namespace julia::model_import
{
    struct ModelImportOptions
    {
        // Compatibility guess for body/hair/shield atlases without references.
        // Default preserves imported assignments instead of guessing by name.
        bool useLegacyMaterialTextureFallback = false;
        // Connected, angle-weighted fans, retaining creases >= 45 degrees.
        // Runs after mesh transforms and weights; preserves authored UV/skin.
        bool rebuildSmoothNormals = false;
    };
    class ModelImporter
    {
    public:
        // CPU-only. The local Assimp::Importer/aiScene die before return.
        // External/embedded images are decoded and owned with a shared cache.
        static std::shared_ptr<const model::ModelAsset> loadFromFile(
            const std::filesystem::path& path, ModelImportOptions options = {});
    };
}
