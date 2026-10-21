#include "julia/render_opengl/GLModel.hpp"
#include "julia_test.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
    bool close(const glm::mat4& a, const glm::mat4& b)
    {
        for (int c = 0; c < 4; ++c)
            for (int r = 0; r < 4; ++r)
                if (std::abs(a[c][r] - b[c][r]) > 1e-4f) return false;
        return true;
    }
    template<class F> bool rejects(F&& f)
    {
        try { f(); } catch (const std::exception&) { return true; }
        return false;
    }
}

int main()
{
    using namespace julia::render_opengl;
    GLModel model;
    const glm::mat4 identity{1};
    const auto rootMatrix = glm::translate(identity, glm::vec3{4,5,6}) * glm::scale(identity, glm::vec3{.01f});
    const auto root = model.addNode({"root", NoModelNode, rootMatrix});
    const auto bindLocal = glm::translate(identity, glm::vec3{0,3,0}) * glm::scale(identity, glm::vec3{2,3,4});
    const auto helper = model.addNode({"helper", root, bindLocal});
    const auto bone = model.addNode({"bone", helper, glm::translate(identity, glm::vec3{0,2,0})});
    model.addBone({"bone", glm::inverse(model.nodes()[bone].globalBindTransform), bone});
    AnimationClip sparse;
    sparse.name = "rotation-only helper";
    sparse.durationTicks = 60;
    sparse.ticksPerSecond = 30;
    AnimationChannel channel;
    channel.nodeIndex = helper;
    channel.rotations = {{0, glm::quat{1,0,0,0}},
        {60, glm::angleAxis(glm::radians(120.0f), glm::vec3{0,0,1})}};
    sparse.channels.push_back(channel);
    model.addAnimation(sparse);
    AnimationPlayer player(model);
    const auto bind = model.skinningMatrices();
    JULIA_CHECK(close(player.palette()[0], identity));
    player.seek(1);
    const auto expectedLocal = glm::translate(identity, glm::vec3{0,3,0}) *
        glm::rotate(identity, glm::radians(60.0f), glm::vec3{0,0,1}) * glm::scale(identity, glm::vec3{2,3,4});
    JULIA_CHECK(close(player.localTransforms()[helper], expectedLocal));
    JULIA_CHECK(player.localTransforms()[root] == rootMatrix);
    JULIA_CHECK(close(player.palette()[0], rootMatrix * expectedLocal * model.nodes()[bone].localTransform * model.bones()[0].offsetMatrix));
    player.play(); player.update(.25); player.pause();
    const auto pausedMatrix = player.palette()[0];
    player.update(20);
    JULIA_CHECK(player.timeSeconds() == 1.25 && player.palette()[0] == pausedMatrix);
    player.stop();
    JULIA_CHECK(player.palette()[0] == bind[0] && player.timeSeconds() == 0);
    player.setLooping(false); player.play(); player.update(10);
    JULIA_CHECK(player.state() == PlaybackState::Finished && player.timeSeconds() == 2);
    player.play(); JULIA_CHECK(player.timeSeconds() == 0);
    player.setLooping(true); player.update(6.25);
    JULIA_CHECK(std::abs(player.timeSeconds() - .25) < 1e-10);
    JULIA_CHECK(rejects([&] { player.seek(std::numeric_limits<double>::quiet_NaN()); }));
    JULIA_CHECK(rejects([&] { player.update(-1); }));
    JULIA_CHECK(rejects([&] { player.selectClip(1); }));
    auto bad = sparse; bad.channels.push_back(channel);
    JULIA_CHECK(rejects([&] { model.addAnimation(bad); }));
    bad = sparse; bad.channels[0].rotations[1].value = glm::quat{0,0,0,0};
    JULIA_CHECK(rejects([&] { model.addAnimation(bad); }));
    bad = sparse; bad.ticksPerSecond = 0;
    JULIA_CHECK(rejects([&] { model.addAnimation(bad); }));

    GLModel translationModel;
    translationModel.addNode({"node"});
    AnimationClip translationClip;
    translationClip.durationTicks = 10; translationClip.ticksPerSecond = 10;
    AnimationChannel translations;
    translations.translations = {{0,{0,0,0}}, {10,{10,20,30}}};
    translations.scales = {{0,{1,1,1}}, {10,{3,3,3}}};
    translationClip.channels.push_back(translations);
    translationModel.addAnimation(translationClip);
    AnimationPlayer translationsPlayer(translationModel);
    translationsPlayer.seek(.5);
    JULIA_CHECK(close(translationsPlayer.localTransforms()[0],
        glm::translate(identity, glm::vec3{5,10,15}) * glm::scale(identity, glm::vec3{2})));
    // A single component key clamps; absent rotation keeps the bind rotation.
    translationClip.channels[0].translations = {{5,{2,4,6}}};
    translationModel.addAnimation(translationClip);
    translationsPlayer.selectClip(1); translationsPlayer.seek(1);
    JULIA_CHECK(close(translationsPlayer.localTransforms()[0],
        glm::translate(identity, glm::vec3{2,4,6}) * glm::scale(identity, glm::vec3{3})));

    GLModel noClips;
    AnimationPlayer empty(noClips);
    empty.play(); empty.update(1); empty.pause(); empty.stop();
    JULIA_CHECK(empty.clip() == nullptr && empty.palette().empty());
    return JULIA_TEST_RESULT();
}
