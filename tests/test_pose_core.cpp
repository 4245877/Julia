#include "pose_test_support.hpp"
#include <array>

namespace
{
    template<class F> bool rejects(F&& f)
    {
        try { f(); } catch (const std::invalid_argument&) { return true; }
        return false;
    }
}

int main()
{
    using namespace julia::animation;
    const glm::mat4 identity{1};
    Skeleton skeleton;
    // Root scale/rotation/reflection and helper shear must survive exactly.
    const auto rootLocal = glm::translate(identity, glm::vec3{12,5,-3}) *
        glm::rotate(identity, .3f, glm::vec3{0,1,0}) * glm::scale(identity, glm::vec3{-2,3,.5f});
    const auto root = skeleton.addNode({"root", NoNode, rootLocal});
    auto helperLocal = glm::translate(identity, glm::vec3{0,4,0});
    helperLocal[1][0] = .35f;
    const auto helper = skeleton.addNode({"helper", root, helperLocal});
    const NodeTRS rest{{0,2,0}, glm::angleAxis(.2f, glm::vec3{1,0,0}), {2,3,4}};
    const auto jointLocal = glm::translate(identity, rest.translation) * glm::mat4_cast(rest.rotation) *
        glm::scale(identity, rest.scale);
    const auto joint = skeleton.addNode({"joint", helper, jointLocal, identity, rest});
    const auto inverseBind = glm::inverse(rootLocal * helperLocal * jointLocal);
    const auto meshOffset = glm::translate(identity, glm::vec3{3,0,0});
    skeleton.addBinding({"joint", inverseBind, joint});
    skeleton.addBinding({"joint", inverseBind * meshOffset, joint});
    // A second root is legal (a forest), including a node without a binding.
    skeleton.addNode({"other root", NoNode, identity});
    JULIA_CHECK(pose_test::compare(skeleton) == 0);
    Pose first(skeleton), second(skeleton);
    JULIA_CHECK(first.locals[root] == rootLocal && first.locals[helper] == helperLocal);
    JULIA_CHECK(pose_test::delta(first.palette, std::array{identity, meshOffset}) < 1e-4f);

    AnimationClip clip;
    clip.durationTicks = 10; clip.ticksPerSecond = 10;
    AnimationChannel channel;
    channel.nodeIndex = joint;
    // Sparse translation: rotation/scale come from importer-provided bind TRS.
    channel.translations = {{0,{0,2,0}}, {5,{1,2,0}}, {5,{2,2,0}}, {10,{4,2,0}}};
    clip.channels.push_back(channel);
    clip.channels.push_back({helper}); // Empty channel must preserve exact shear.
    prepareAnimationClip(clip, skeleton);
    for (double t : {-100.0, 0.0, .25, .5, .50000001, .75, 1.0, 100.0})
        JULIA_CHECK(pose_test::compare(skeleton, &clip, t) <= 1e-5f);
    sampleAnimation(skeleton, clip, .5, first.locals);
    JULIA_CHECK(first.locals[helper] == helperLocal && first.locals[root] == rootLocal);
    JULIA_CHECK(first.locals[joint][3] == glm::vec4(2,2,0,1)); // Last duplicate at internal endpoint.
    JULIA_CHECK(second.locals[joint] == jointLocal); // Shared asset, independent characters.

    // Arbitrary procedural pose source after sampling, before FK/palette.
    first.locals[helper] *= glm::rotate(identity, .7f, glm::vec3{0,0,1});
    computeGlobalTransforms(skeleton, first.locals, first.globals);
    buildSkinningPalette(skeleton, first.globals, first.palette);
    const auto expected = rootLocal * first.locals[helper] * first.locals[joint] * inverseBind;
    JULIA_CHECK(pose_test::delta(first.palette, std::array{expected, expected * meshOffset}) < 1e-4f);
    auto inPlace = first.locals;
    computeGlobalTransforms(skeleton, inPlace, inPlace);
    JULIA_CHECK(pose_test::delta(inPlace, first.globals) == 0);
    computeGlobalTransforms(skeleton, {}, first.globals);
    JULIA_CHECK(first.globals[joint] == skeleton.nodes()[joint].globalBindTransform);

    // Rotation-only and scale-only channels retain the other bind components.
    clip.channels[0].translations.clear();
    clip.channels[0].rotations = {{0,rest.rotation},
        {10,glm::angleAxis(.6f, glm::vec3{1,0,0})}};
    prepareAnimationClip(clip, skeleton);
    sampleAnimation(skeleton, clip, .5, first.locals);
    const auto rotatedLocal = glm::translate(identity, rest.translation) *
        glm::mat4_cast(glm::angleAxis(.4f, glm::vec3{1,0,0})) * glm::scale(identity, rest.scale);
    JULIA_CHECK(pose_test::delta(std::array{first.locals[joint]}, std::array{rotatedLocal}) < 1e-5f);
    clip.channels[0].rotations.clear();
    clip.channels[0].scales = {{0,{2,3,4}}, {10,{4,5,6}}};
    prepareAnimationClip(clip, skeleton);
    sampleAnimation(skeleton, clip, .5, first.locals);
    const auto scaledLocal = glm::translate(identity, rest.translation) *
        glm::mat4_cast(rest.rotation) * glm::scale(identity, glm::vec3{3,4,5});
    JULIA_CHECK(pose_test::delta(std::array{first.locals[joint]}, std::array{scaledLocal}) < 1e-5f);

    // Single key, antipodal quaternions, zero-duration clip, endpoint clamping.
    clip.channels[0].scales.clear();
    clip.channels[0].translations = {{5,{3,4,5}}};
    clip.channels[0].rotations = {{0,rest.rotation}, {10,-rest.rotation}};
    prepareAnimationClip(clip, skeleton);
    JULIA_CHECK(pose_test::compare(skeleton, &clip, .5) <= 1e-5f);
    clip.durationTicks = 0;
    JULIA_CHECK(pose_test::compare(skeleton, &clip, 100) <= 1e-5f);

    const auto nan = std::numeric_limits<double>::quiet_NaN();
    JULIA_CHECK(rejects([&] { sampleAnimation(skeleton, clip, nan, first.locals); }));
    JULIA_CHECK(rejects([&] { sampleAnimation(skeleton, clip, 0, {}); }));
    JULIA_CHECK(rejects([&] { computeGlobalTransforms(skeleton, {}, {}); }));
    JULIA_CHECK(rejects([&] { buildSkinningPalette(skeleton, first.globals, {}); }));
    first.locals[helper][0][0] = std::numeric_limits<float>::infinity();
    JULIA_CHECK(rejects([&] { computeGlobalTransforms(skeleton, first.locals, first.globals); }));
    first.globals[joint][0][0] = std::numeric_limits<float>::quiet_NaN();
    JULIA_CHECK(rejects([&] { buildSkinningPalette(skeleton, first.globals, first.palette); }));
    JULIA_CHECK(rejects([&] { skeleton.addNode({"bad", 99}); }));
    JULIA_CHECK(rejects([&] { skeleton.addBinding({"bad", identity, 99}); }));
    auto bad = clip; bad.channels.push_back(bad.channels.front());
    JULIA_CHECK(rejects([&] { prepareAnimationClip(bad, skeleton); }));
    bad = clip; bad.ticksPerSecond = 0;
    JULIA_CHECK(rejects([&] { prepareAnimationClip(bad, skeleton); }));
    bad = clip; bad.channels[0].rotations[0].value = glm::quat{0,0,0,0};
    JULIA_CHECK(rejects([&] { prepareAnimationClip(bad, skeleton); }));
    bad = clip; bad.channels[0].translations.push_back({nan,{0,0,0}});
    JULIA_CHECK(rejects([&] { prepareAnimationClip(bad, skeleton); }));
    bad = clip; bad.channels[0].translations = {{1,{0,0,0}}, {0,{1,1,1}}};
    JULIA_CHECK(rejects([&] { prepareAnimationClip(bad, skeleton); }));
    bad = clip; bad.channels[0].nodeIndex = NoNode;
    JULIA_CHECK(rejects([&] { prepareAnimationClip(bad, skeleton); }));

    Skeleton empty;
    Pose emptyPose(empty);
    AnimationClip emptyClip;
    prepareAnimationClip(emptyClip, empty);
    JULIA_CHECK(pose_test::compare(empty, &emptyClip) == 0);
    return JULIA_TEST_RESULT();
}
