#include "julia/model_import/ModelImporter.hpp"
#include "julia/embodiment/Animation.hpp"
#include "julia/embodiment/EyePoseLayer.hpp"
#include "model_asset_snapshot.hpp"
#include "julia_test.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

int main(int argc, char** argv)
{
    using namespace julia;
    try
    {
        std::ostringstream snapshot;
        for (bool repair : {false, true})
        {
            // Importer/scene are already dead. This binary has no GL/GLFW link.
            auto asset = model_import::ModelImporter::loadFromFile(
                std::filesystem::path{JULIA_TEST_ASSET_DIR} / "models/Raphtalia_prepared.fbx",
                {.rebuildSmoothNormals = repair});
            for (const auto& mesh : asset->meshes())
                model_snapshot::mesh(snapshot, mesh.vertices, mesh.indices, mesh.material, [&](auto& h) {
                    const auto& image = mesh.material.diffuseTexture;
                    h.add(bool(image));
                    if (!image) return;
                    h.add(image->width()); h.add(image->height()); h.add(image->hasTransparency());
                    for (const auto byte : image->pixels()) h.add(byte);
                });
            model_snapshot::rig(snapshot, *asset);
            embodiment::AnimationPlayer player(*asset);
            embodiment::EyePoseLayer eyes(*asset);
            JULIA_CHECK(eyes.available() && eyes.lidCount() == 188);
            model_snapshot::eyes(snapshot, player, eyes);

            // Two independent instances share immutable data, never pose/time.
            embodiment::AnimationPlayer other(*asset);
            const auto before = std::vector<glm::mat4>(other.palette().begin(), other.palette().end());
            player.seek(.5); player.play(); player.update(.1);
            JULIA_CHECK(other.timeSeconds() == 0);
            JULIA_CHECK(std::equal(before.begin(), before.end(), other.palette().begin()));
            model::ModelAsset different;
            embodiment::EyePoseLayer foreign(different);
            bool rejected = false;
            try { player.setEyeLayer(&foreign); } catch (const std::invalid_argument&) { rejected = true; }
            JULIA_CHECK(rejected);
        }
        const auto output = argc > 1 ? std::filesystem::path{argv[1]} :
            std::filesystem::path{JULIA_TEST_SOURCE_DIR} / "../build/diagnostics/stage2";
        std::filesystem::create_directories(output);
        std::ofstream(output / "cpu-import-snapshot.txt") << snapshot.str();
        std::ifstream reference(std::filesystem::path{JULIA_TEST_SOURCE_DIR} / "data/raphtalia_stage2_reference.txt");
        JULIA_CHECK(reference.good());
        std::ostringstream expected; expected << reference.rdbuf();
        JULIA_CHECK(snapshot.str() == expected.str());
        std::cout << "Owned CPU FBX, two normal modes: geometry/materials/RGBA/rig/keys/eyes "
            << (snapshot.str() == expected.str() ? "match pre-refactor reference" : "REFERENCE MISMATCH") << '\n';
    }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; JULIA_CHECK(false); }
    return JULIA_TEST_RESULT();
}
