#include "julia/model_import/ModelImporter.hpp"
#include "julia/embodiment/EyePoseLayer.hpp"
#include "julia/embodiment/Animation.hpp"
#include "model_import_fixtures.hpp"
#include "julia_test.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <iostream>
#include <limits>

namespace
{
    using namespace julia;
    using model_import::ModelImporter;
    template<class F> bool rejects(F f)
    { try { f(); } catch (const std::exception&) { return true; } return false; }

    bool close(const glm::mat4& a, const glm::mat4& b)
    {
        for (int c=0;c<4;++c) for (int r=0;r<4;++r)
            if (std::abs(a[c][r]-b[c][r]) > 1e-5f) return false;
        return true;
    }

    void fixtures(const std::filesystem::path& directory)
    {
        using namespace model_fixture;
        const auto path = writeInstancedSkin(directory, false, true);
        auto asset = ModelImporter::loadFromFile(path);
        // The exporter scene and importer have both died; remove the source too.
        std::filesystem::remove(path);
        JULIA_CHECK(asset->meshCount() == 2 && asset->boneCount() == 2);
        JULIA_CHECK(asset->bones()[0].nodeIndex == asset->bones()[1].nodeIndex);
        JULIA_CHECK(asset->bones()[0].offsetMatrix != asset->bones()[1].offsetMatrix);
        const auto palette = asset->skinningMatrices();
        JULIA_CHECK(close(palette[0], glm::mat4{1}));
        JULIA_CHECK(close(palette[1], glm::translate(glm::mat4{1}, glm::vec3{-.8f,0,0})));
        for (const auto& mesh : asset->meshes())
        {
            JULIA_CHECK(mesh.indices.size() == 3 && mesh.vertices.size() == 3);
            for (const auto& v : mesh.vertices)
            {
                JULIA_CHECK(v.boneWeights == glm::vec4(1,0,0,0));
                JULIA_CHECK(v.boneIds[0] >= 0 && v.boneIds[0] < 2);
            }
            const auto& image = *mesh.material.diffuseTexture;
            JULIA_CHECK(image.width() == 1 && image.height() == 1);
            JULIA_CHECK(image.hasTransparency() && image.pixels()[0] == 200 && image.pixels()[3] == 128);
            JULIA_CHECK(mesh.material.alphaMode == model::AlphaMode::Mask);
        }
        JULIA_CHECK(asset->meshes()[0].material.diffuseTexture == asset->meshes()[1].material.diffuseTexture);
        const auto& clip = asset->animations()[0];
        JULIA_CHECK(clip.usedDefaultTicksPerSecond && clip.ticksPerSecond == 25 && clip.durationSeconds() == 2);
        embodiment::AnimationPlayer player(*asset);
        player.seek(1);
        JULIA_CHECK(std::abs(player.localTransforms()[clip.channels[0].nodeIndex][3].x - .5f) < 1e-5f);
        embodiment::EyePoseLayer unsupported(*asset);
        JULIA_CHECK(!unsupported.available());
        JULIA_CHECK(rejects([&] { ModelImporter::loadFromFile(writeInstancedSkin(directory,true)); }));

        const auto gltf = ModelImporter::loadFromFile(writeGltf(directory));
        JULIA_CHECK(gltf->meshCount() == 3);
        JULIA_CHECK(gltf->meshes()[0].material.alphaMode == model::AlphaMode::Opaque);
        JULIA_CHECK(gltf->meshes()[1].material.alphaMode == model::AlphaMode::Mask);
        JULIA_CHECK(gltf->meshes()[2].material.alphaMode == model::AlphaMode::Blend);
        const auto& first = gltf->meshes()[0];
        JULIA_CHECK(first.material.wrapS == model::sampler::ClampToEdge);
        JULIA_CHECK(first.material.wrapT == model::sampler::MirroredRepeat);
        JULIA_CHECK(first.material.minFilter == model::sampler::Nearest);
        JULIA_CHECK(gltf->meshes()[2].material.wrapS == model::sampler::Repeat);
        JULIA_CHECK(first.material.diffuseTexture == gltf->meshes()[2].material.diffuseTexture);
        const auto expectedNormal = glm::normalize(glm::vec3{-1,1/1.5f,2});
        for (const auto& v : first.vertices)
        {
            JULIA_CHECK(glm::length(v.normal - expectedNormal) < 1e-5f);
            JULIA_CHECK(glm::length(v.texCoord - glm::vec2{.7f,.8f}) < 1e-5f);
        }
        const auto& a = first.vertices[first.indices[0]].position;
        const auto& b = first.vertices[first.indices[1]].position;
        const auto& c = first.vertices[first.indices[2]].position;
        JULIA_CHECK(glm::cross(b-a,c-a).z > 0); // Reflection repairs winding.
        JULIA_CHECK(first.material.diffuseColor == glm::vec3(.25f,.5f,.75f));
        JULIA_CHECK(std::abs(first.material.opacity-.4f) < 1e-5f);
        JULIA_CHECK(first.material.diffuseTexture->pixels()[0] == 200);

        const auto unicode = directory / std::filesystem::path{u8"імпорт_тест"};
        std::filesystem::create_directories(unicode);
        JULIA_CHECK(ModelImporter::loadFromFile(writeGltf(unicode))->meshCount() == 3);
        const auto legacyPath = writeLegacyFallbackFixture(directory);
        const auto plain = ModelImporter::loadFromFile(legacyPath);
        for (const auto& mesh : plain->meshes()) JULIA_CHECK(!mesh.material.diffuseTexture);
        const auto legacy = ModelImporter::loadFromFile(legacyPath, {.useLegacyMaterialTextureFallback=true});
        for (const auto& mesh : legacy->meshes()) JULIA_CHECK(mesh.material.diffuseTexture);
        std::filesystem::remove(directory / "textures/body_color.png");
        std::filesystem::remove(directory / "textures/hair_color.png");
        for (const auto& mesh : legacy->meshes()) JULIA_CHECK(mesh.material.diffuseTexture->pixels()[0] == 200);

        JULIA_CHECK(rejects([&] { ModelImporter::loadFromFile(directory / "missing.fbx"); }));
        JULIA_CHECK(rejects([&] { model::Image::decode(nullptr,0); }));
        const unsigned char grayAlpha[]{17,128};
        const auto image = model::Image::fromPixels(grayAlpha,1,1,2);
        JULIA_CHECK(image.pixels()[0] == 17 && image.pixels()[1] == 17 && image.pixels()[2] == 17 && image.pixels()[3] == 128);
        JULIA_CHECK(rejects([&] { model::Image::fromPixels(grayAlpha,0,1,2); }));
        model::ModelAsset builder;
        model::Mesh invalid;
        invalid.vertices.resize(3); invalid.indices = {0,1,3};
        JULIA_CHECK(rejects([&] { builder.addMesh(invalid); }));
        invalid.indices = {0,1};
        JULIA_CHECK(rejects([&] { builder.addMesh(invalid); }));
        invalid.indices = {0,1,2}; invalid.vertices[0].boneWeights.x = 1;
        JULIA_CHECK(rejects([&] { builder.addMesh(invalid); }));
        invalid.vertices[0].boneWeights.x = 0;
        invalid.vertices[0].texCoord.x = std::numeric_limits<float>::infinity();
        JULIA_CHECK(rejects([&] { builder.addMesh(invalid); }));
        JULIA_CHECK(builder.empty() && !builder.bounds().valid);
    }
}

int main(int argc, char** argv)
{
    try
    {
        const auto directory = argc > 1 ? std::filesystem::path{argv[1]} :
            std::filesystem::path{JULIA_TEST_SOURCE_DIR} / "../build/diagnostics/stage2/model_asset_import";
        std::filesystem::create_directories(directory);
        fixtures(directory);
        std::cout << "CPU fixtures: owned raw/compressed/external images, UV1, reflection, instanced skin and default ticks verified\n";
    }
    catch (const std::exception& e) { std::cerr << e.what() << '\n'; JULIA_CHECK(false); }
    return JULIA_TEST_RESULT();
}
