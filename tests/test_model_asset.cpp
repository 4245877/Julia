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
        const auto path=std::filesystem::path{JULIA_TEST_ASSET_DIR}/"models/Raphtalia_prepared.fbx";
        std::ostringstream snapshot;
        for (bool repair : {false, true})
        {
            // Importer/scene are already dead. This binary has no GL/GLFW link.
            auto asset = model_import::ModelImporter::loadFromFile(
                path, {.rebuildSmoothNormals = repair});
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

            if (repair)
            {
                const auto bare=model_import::ModelImporter::loadFromFile(path,
                    {.rebuildSmoothNormals=true,.omitPreparedGloves=true});
                // The normalized source snapshot pins these authored parts.
                constexpr std::size_t gloves=7, skin=9, leftCuff=17, rightCuff=24;
                JULIA_CHECK(asset->meshCount()==28 && bare->meshCount()==27);
                if (asset->meshCount()!=28 || bare->meshCount()!=27) return JULIA_TEST_RESULT();
                JULIA_CHECK(asset->meshes()[gloves].indices.size()==5128*3);
                JULIA_CHECK(asset->meshes()[skin].indices.size()==30494*3);
                JULIA_CHECK(asset->meshes()[leftCuff].indices.size()==568*3 && asset->meshes()[rightCuff].indices.size()==568*3);
                for (std::size_t i=0;i<asset->meshCount();++i)
                {
                    if (i==gloves) continue;
                    const auto& source=asset->meshes()[i];
                    const auto& visible=bare->meshes()[i-(i>gloves)];
                    JULIA_CHECK(source.indices==visible.indices && source.material.toonKind==visible.material.toonKind);
                    JULIA_CHECK(std::ranges::equal(source.vertices,visible.vertices,[&](const auto& a,const auto& b) {
                        if (a.position!=b.position || a.normal!=b.normal || a.texCoord!=b.texCoord || a.boneWeights!=b.boneWeights)
                            return false;
                        for (int k=0;k<4;++k) {
                            if (a.boneIds[k]<0 || b.boneIds[k]<0) {
                                if (a.boneIds[k]!=b.boneIds[k]) return false;
                            } else {
                                const auto& from=asset->bones()[a.boneIds[k]];
                                const auto& to=bare->bones()[b.boneIds[k]];
                                if (from.nodeIndex!=to.nodeIndex || from.offsetMatrix!=to.offsetMatrix) return false;
                            }
                        }
                        return true;
                    }));
                }
                std::cout << "Bare Raphtalia: glove mesh omitted, original skin and both cuffs preserved\n";
            }
        }
        if (!julia_test::hasOutputDirectoryArgument(argc, argv)) return 2;
        const std::filesystem::path output{argv[1]};
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
