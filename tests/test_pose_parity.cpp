#include "julia/model_import/ModelImporter.hpp"
#include "julia/embodiment/Animation.hpp"
#include "pose_test_support.hpp"
#include <filesystem>
#include <iostream>
#include <set>

int main()
{
    using namespace julia::embodiment;
    const auto asset = julia::model_import::ModelImporter::loadFromFile(
        std::filesystem::path{JULIA_TEST_ASSET_DIR} / "models/Raphtalia_prepared.fbx");
    const auto& skeleton = asset->skeleton();
    const auto& clips = asset->animations();
    AnimationPlayer player(*asset);
    JULIA_CHECK(pose_test::delta(player.palette(), asset->skinningMatrices()) == 0);
    for (std::size_t i = 0; i < clips.size(); ++i)
    {
        player.selectClip(i);
        for (double fraction : {0.0, .37, 1.0})
        {
            const double time = fraction * clips[i].durationSeconds();
            julia::animation::Pose pose(skeleton);
            julia::animation::sampleAnimation(skeleton, clips[i], time, pose.locals);
            julia::animation::computeGlobalTransforms(skeleton, pose.locals, pose.globals);
            julia::animation::buildSkinningPalette(skeleton, pose.globals, pose.palette);
            player.seek(time);
            JULIA_CHECK(pose_test::delta(player.localTransforms(), pose.locals) == 0);
            JULIA_CHECK(pose_test::delta(player.globalTransforms(), pose.globals) == 0);
            JULIA_CHECK(pose_test::delta(player.palette(), pose.palette) == 0);
        }
    }
    JULIA_CHECK(skeleton.bindings().size() == 429 && clips.size() == 15);

    float maximum = pose_test::compare(skeleton);
    std::size_t samples = 1;
    for (const auto& clip : clips)
    {
        // Negative/end/outside times plus selected interior fractions of every clip.
        for (double time : {-1.0, 0.0, clip.durationSeconds() * .25,
            clip.durationSeconds() * .5, clip.durationSeconds() * .75,
            clip.durationSeconds(), clip.durationSeconds() + 1})
        {
            maximum = std::max(maximum, pose_test::compare(skeleton, &clip, time));
            ++samples;
        }
        // Every distinct source time across ALL channels, including sparse ones.
        std::set<double> keyTimes;
        for (const auto& channel : clip.channels)
        {
            const auto collectTimes = [&](const auto& keys) {
                for (const auto& key : keys)
                    keyTimes.insert(key.ticks / clip.ticksPerSecond);
            };
            collectTimes(channel.translations);
            collectTimes(channel.rotations);
            collectTimes(channel.scales);
        }
        for (double time : keyTimes)
            for (double offset : {-1e-8, 0.0, 1e-8})
            {
                maximum = std::max(maximum, pose_test::compare(skeleton, &clip, time + offset));
                ++samples;
            }
    }
    std::cout << "Headless CPU parity: nodes=" << skeleton.nodes().size()
        << " bindings=" << skeleton.bindings().size() << " clips=" << clips.size()
        << " samples=" << samples << " max_local_global_palette_delta=" << maximum << '\n';
    return JULIA_TEST_RESULT();
}
