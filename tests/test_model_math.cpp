#include "julia/render_opengl/GLModel.hpp"
#include "julia_test.hpp"

#include <glm/gtc/matrix_transform.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace
{
    bool close(const glm::mat4& a, const glm::mat4& b)
    {
        for (int column = 0; column < 4; ++column)
            for (int row = 0; row < 4; ++row)
                if (std::abs(a[column][row] - b[column][row]) > 0.0001f) return false;
        return true;
    }

    template<class Exception, class Callable>
    bool throws(Callable&& call)
    {
        try { call(); }
        catch (const Exception&) { return true; }
        return false;
    }
}

int main()
{
    using namespace julia::render_opengl;
    const glm::mat4 identity{1.0f};
    GLModel model;
    const auto rootTransform = glm::translate(identity, glm::vec3{12.0f, 5.0f, -3.0f}) *
        glm::rotate(identity, 0.3f, glm::vec3{0.0f, 1.0f, 0.0f}) *
        glm::scale(identity, glm::vec3{2.0f, 3.0f, 0.5f});
    const auto root = model.addNode({"root", NoModelNode, rootTransform});
    const auto helperTransform = glm::translate(identity, glm::vec3{0.0f, 4.0f, 0.0f});
    const auto helper = model.addNode({"helper", root, helperTransform});
    const auto boneTransform = glm::translate(identity, glm::vec3{0.0f, 2.0f, 0.0f});
    const auto bone = model.addNode({"joint", helper, boneTransform});
    const auto globalBind = rootTransform * helperTransform * boneTransform;
    JULIA_CHECK(close(model.nodes()[bone].globalBindTransform, globalBind));
    model.addBone({"joint", glm::inverse(globalBind), bone});
    // One node can drive several mesh-local offsets. They must remain separate
    // palette entries even when their names and skeleton node are identical.
    const auto instanceOffset = glm::translate(identity, glm::vec3{3.0f, 0.0f, 0.0f});
    model.addBone({"joint", glm::inverse(globalBind) * instanceOffset, bone});
    const auto bind = model.skinningMatrices();
    JULIA_CHECK(bind.size() == 2);
    JULIA_CHECK(close(bind[0], identity));
    JULIA_CHECK(close(bind[1], instanceOffset));

    std::vector<glm::mat4> pose{rootTransform, helperTransform, boneTransform};
    const auto movement = glm::translate(identity, glm::vec3{1.0f, 2.0f, 3.0f});
    pose[0] = movement * rootTransform;
    const auto moved = model.skinningMatrices(pose);
    JULIA_CHECK(close(moved[0], movement));
    JULIA_CHECK(close(moved[1], movement * instanceOffset));
    // Helpers that are not bones still affect every descendant bone.
    pose[0] = rootTransform;
    pose[1] *= glm::rotate(identity, 0.7f, glm::vec3{0.0f, 0.0f, 1.0f});
    JULIA_CHECK(close(model.skinningMatrices(pose)[0],
        rootTransform * pose[1] * boneTransform * glm::inverse(globalBind)));

    JULIA_CHECK(throws<std::invalid_argument>([&] { model.skinningMatrices({pose.data(), 1}); }));
    pose[1][0][0] = std::numeric_limits<float>::quiet_NaN();
    JULIA_CHECK(throws<std::invalid_argument>([&] { model.skinningMatrices(pose); }));
    JULIA_CHECK(throws<std::invalid_argument>([&] { model.addNode({"bad parent", 99, identity}); }));
    JULIA_CHECK(throws<std::invalid_argument>([&] { model.fitToHeightTransform(0.0f); }));
    JULIA_CHECK(throws<std::invalid_argument>([&] {
        model.fitToHeightTransform(std::numeric_limits<float>::infinity());
    }));
    JULIA_CHECK(close(model.fitToHeightTransform(), identity));

    Bounds bounds;
    bounds.expand(glm::vec3{std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f});
    JULIA_CHECK(!bounds.valid);
    bounds.expand(glm::vec3{1.0f, -2.0f, 4.0f});
    bounds.expand(glm::vec3{5.0f, 6.0f, 8.0f});
    JULIA_CHECK(bounds.size() == glm::vec3(4.0f, 8.0f, 4.0f));
    JULIA_CHECK(bounds.center() == glm::vec3(3.0f, 2.0f, 6.0f));
    Bounds large;
    large.expand(glm::vec3{std::numeric_limits<float>::max() * 0.75f});
    large.expand(glm::vec3{std::numeric_limits<float>::max()});
    JULIA_CHECK(std::isfinite(large.center().x));
    return JULIA_TEST_RESULT();
}
