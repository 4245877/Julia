#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "julia/render_opengl/GLMesh.hpp"
#include "julia/render_opengl/GLTexture.hpp"
#include "julia/render_opengl/OpenGLRenderer.hpp"
#include "julia_test.hpp"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace
{
    using namespace julia::render_opengl;
    constexpr int ImageSize = 256;

    class RenderTarget
    {
    public:
        RenderTarget()
        {
            glGenFramebuffers(1, &framebuffer_);
            glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);
            glGenTextures(1, &color_);
            glBindTexture(GL_TEXTURE_2D, color_);
            // Linear floating-point pixels make the assertions independent of
            // display gamma and avoid quantizing the material/shadow contrast.
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, ImageSize, ImageSize, 0, GL_RGBA, GL_FLOAT, nullptr);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, color_, 0);
            glGenRenderbuffers(1, &depth_);
            glBindRenderbuffer(GL_RENDERBUFFER, depth_);
            glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, ImageSize, ImageSize);
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_);
            glDrawBuffer(GL_COLOR_ATTACHMENT0);
            glReadBuffer(GL_COLOR_ATTACHMENT0);
            if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
                throw std::runtime_error("Toon regression framebuffer is incomplete");
        }

        ~RenderTarget()
        {
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            glDeleteRenderbuffers(1, &depth_);
            glDeleteTextures(1, &color_);
            glDeleteFramebuffers(1, &framebuffer_);
        }

        RenderTarget(const RenderTarget&) = delete;
        RenderTarget& operator=(const RenderTarget&) = delete;

        glm::vec3 sample(const glm::vec3& world, const CameraConfig& camera) const
        {
            const auto projection = glm::perspective(glm::radians(camera.fovDegrees), 1.0f,
                camera.nearPlane, camera.farPlane);
            const auto view = glm::lookAt(camera.position, camera.target, camera.up);
            const auto clip = projection * view * glm::vec4{world, 1.0f};
            const auto uv = glm::vec2{clip} / clip.w * 0.5f + 0.5f;
            const int x = static_cast<int>(std::floor(uv.x * ImageSize));
            const int y = static_cast<int>(std::floor(uv.y * ImageSize));
            if (clip.w <= 0.0f || x < 1 || y < 1 || x >= ImageSize - 1 || y >= ImageSize - 1)
                throw std::runtime_error("Toon sample falls outside the render target");
            std::array<float, 3 * 3 * 4> pixels{};
            glReadPixels(x - 1, y - 1, 3, 3, GL_RGBA, GL_FLOAT, pixels.data());
            glm::vec3 average{0.0f};
            for (std::size_t pixel = 0; pixel < 9; ++pixel)
            {
                const glm::vec3 color{pixels[pixel * 4], pixels[pixel * 4 + 1], pixels[pixel * 4 + 2]};
                JULIA_CHECK(std::isfinite(color.r) && std::isfinite(color.g) && std::isfinite(color.b));
                average += color;
            }
            JULIA_CHECK(glGetError() == GL_NO_ERROR);
            return average / 9.0f;
        }

    private:
        GLuint framebuffer_ = 0;
        GLuint color_ = 0;
        GLuint depth_ = 0;
    };

    GLMesh horizontalQuad(float halfWidth, float height, GLMeshMaterial material, int boneId = -1)
    {
        std::vector<GLVertex> vertices(4);
        vertices[0].position = {-halfWidth, height, -halfWidth};
        vertices[1].position = {-halfWidth, height, halfWidth};
        vertices[2].position = {halfWidth, height, halfWidth};
        vertices[3].position = {halfWidth, height, -halfWidth};
        vertices[0].texCoord = {0.0f, 0.0f};
        vertices[1].texCoord = {0.0f, 1.0f};
        vertices[2].texCoord = {1.0f, 1.0f};
        vertices[3].texCoord = {1.0f, 0.0f};
        for (auto& vertex : vertices)
        {
            vertex.normal = {0.0f, 1.0f, 0.0f};
            if (boneId >= 0)
            {
                vertex.boneIds = {boneId, -1, -1, -1};
                vertex.boneWeights = {1.0f, 0.0f, 0.0f, 0.0f};
            }
        }
        return GLMesh{std::move(vertices), {0, 1, 2, 0, 2, 3}, std::move(material)};
    }

    GLMeshMaterial matteMaterial()
    {
        GLMeshMaterial material;
        material.diffuseColor = {0.45f, 0.35f, 0.20f};
        material.specularStrength = 0.0f;
        material.rimStrength = 0.0f;
        material.outlineScale = 0.0f;
        return material;
    }

    GLModel shadowScene(AlphaMode casterMode)
    {
        GLModel model;
        model.addMesh(horizontalQuad(1.2f, 0.0f, matteMaterial()));
        auto caster = matteMaterial();
        caster.alphaMode = casterMode;
        caster.minFilter = GL_NEAREST;
        caster.magFilter = GL_NEAREST;
        constexpr std::array<unsigned char, 8> pixels{255, 255, 255, 255, 255, 255, 255, 0};
        caster.diffuseTexture = std::make_shared<GLTexture>(GLTexture::loadFromPixels(pixels.data(), 2, 1, 4));
        model.addMesh(horizontalQuad(0.25f, 0.6f, std::move(caster)));
        return model;
    }

    void draw(OpenGLRenderer& renderer, const GLModel& model, const ToonStyleConfig& style)
    {
        renderer.setStyleConfig(style);
        renderer.beginFrame();
        renderer.renderModel(model, glm::mat4{1.0f});
        JULIA_CHECK(glGetError() == GL_NO_ERROR);
    }

    void testSelfShadowAndAlpha(RenderTarget& target, OpenGLRenderer& renderer)
    {
        CameraConfig camera;
        camera.position = {2.0f, 2.4f, 3.4f};
        camera.target = {0.0f, 0.0f, 0.0f};
        renderer.setCamera(camera);
        ToonStyleConfig style;
        style.keyLightDirection = {0.65f, 1.0f, 0.25f};
        style.outlineWidthPixels = 0.0f;
        style.selfShadows = false;
        auto masked = shadowScene(AlphaMode::Mask);
        // Positions are the projections of caster centers along -light onto Y=0.
        const glm::vec3 opaqueReceiver{-0.125f - 0.39f, 0.0f, -0.15f};
        const glm::vec3 holeReceiver{0.125f - 0.39f, 0.0f, -0.15f};
        draw(renderer, masked, style);
        const auto unshadowed = target.sample(opaqueReceiver, camera);
        const auto unshadowedHole = target.sample(holeReceiver, camera);
        style.selfShadows = true;
        draw(renderer, masked, style);
        const auto shadowed = target.sample(opaqueReceiver, camera);
        const auto hole = target.sample(holeReceiver, camera);
        JULIA_CHECK(shadowed.r < unshadowed.r - 0.05f);
        JULIA_CHECK(glm::length(hole - unshadowedHole) < 0.012f);
        draw(renderer, masked, style);
        JULIA_CHECK(glm::length(target.sample(opaqueReceiver, camera) - shadowed) < 0.001f);

        // Alpha zero in an Opaque texture must still cast a solid shadow.
        auto opaque = shadowScene(AlphaMode::Opaque);
        draw(renderer, opaque, style);
        JULIA_CHECK(target.sample(holeReceiver, camera).r < unshadowedHole.r - 0.05f);
        style.shadowStrength = 0.0f;
        draw(renderer, opaque, style);
        JULIA_CHECK(glm::length(target.sample(holeReceiver, camera) - unshadowedHole) < 0.012f);

        // Moving only the light relocates the cast shadow. The receiving
        // plane's N.L stays identical because the light's Y is unchanged.
        style.shadowStrength = 0.75f;
        style.keyLightDirection.x = -style.keyLightDirection.x;
        draw(renderer, masked, style);
        const glm::vec3 movedReceiver{-0.125f + 0.39f, 0.0f, -0.15f};
        const auto movedShadow = target.sample(movedReceiver, camera);
        JULIA_CHECK(target.sample(opaqueReceiver, camera).r > shadowed.r + 0.05f);
        JULIA_CHECK(movedShadow.r < unshadowed.r - 0.05f);
        camera.position.x = -camera.position.x;
        renderer.setCamera(camera);
        draw(renderer, masked, style);
        JULIA_CHECK(glm::length(target.sample(movedReceiver, camera) - movedShadow) < 0.005f);
    }

    void testLightingAndMaterials(RenderTarget& target, OpenGLRenderer& renderer)
    {
        CameraConfig camera;
        camera.position = {0.0f, 3.0f, 0.1f};
        camera.target = {0.0f, 0.0f, 0.0f};
        renderer.setCamera(camera);
        ToonStyleConfig style;
        style.selfShadows = false;
        style.outlineWidthPixels = 0.0f;
        style.keyLightDirection = {0.0f, 1.0f, 0.0f};
        GLModel generic;
        generic.addMesh(horizontalQuad(1.0f, 0.0f, matteMaterial()));
        draw(renderer, generic, style);
        const auto lit = target.sample(glm::vec3{0.0f}, camera);
        style.keyLightDirection = {0.0f, -1.0f, 0.0f};
        draw(renderer, generic, style);
        const auto shadow = target.sample(glm::vec3{0.0f}, camera);
        JULIA_CHECK(lit.r > shadow.r + 0.1f);

        auto skinMaterial = matteMaterial();
        skinMaterial.toonKind = ToonMaterialKind::Skin;
        GLModel skin;
        skin.addMesh(horizontalQuad(1.0f, 0.0f, skinMaterial));
        draw(renderer, skin, style);
        const auto skinShadow = target.sample(glm::vec3{0.0f}, camera);
        JULIA_CHECK(skinShadow.r > shadow.r + 0.025f);
        JULIA_CHECK(skinShadow.b < skinShadow.r);

        auto inkMaterial = matteMaterial();
        inkMaterial.toonKind = ToonMaterialKind::Ink;
        inkMaterial.diffuseColor = {0.015f, 0.02f, 0.025f};
        GLModel ink;
        ink.addMesh(horizontalQuad(1.0f, 0.0f, inkMaterial));
        draw(renderer, ink, style);
        const auto inkShadow = target.sample(glm::vec3{0.0f}, camera);
        style.keyLightDirection = {0.0f, 1.0f, 0.0f};
        draw(renderer, ink, style);
        JULIA_CHECK(glm::length(target.sample(glm::vec3{0.0f}, camera) - inkShadow) < 0.001f);
        JULIA_CHECK(glm::length(inkShadow - inkMaterial.diffuseColor) < 0.001f);

        // Align the key reflection with the camera on the same geometric plane.
        // Eye glints must be visibly stronger than the old sub-percent output.
        style.keyLightDirection = {0.0f, 3.0f, -0.1f};
        auto eyeMaterial = matteMaterial();
        eyeMaterial.toonKind = ToonMaterialKind::Eye;
        eyeMaterial.specularPower = 160.0f;
        GLModel eyeMatte;
        eyeMatte.addMesh(horizontalQuad(1.0f, 0.0f, eyeMaterial));
        draw(renderer, eyeMatte, style);
        const auto matte = target.sample(glm::vec3{0.0f}, camera);
        eyeMaterial.specularStrength = 0.50f;
        GLModel eyeGlint;
        eyeGlint.addMesh(horizontalQuad(1.0f, 0.0f, eyeMaterial));
        draw(renderer, eyeGlint, style);
        const auto glint = target.sample(glm::vec3{0.0f}, camera);
        JULIA_CHECK(glint.r > matte.r + 0.15f);
        JULIA_CHECK(glint.b > matte.b + 0.2f);
    }

    void testPlanarSelfShadowAcne(RenderTarget& target, OpenGLRenderer& renderer)
    {
        GLModel plane;
        plane.addMesh(horizontalQuad(1.0f, 0.0f, matteMaterial()));
        const auto rotation = glm::rotate(glm::mat4{1.0f}, glm::radians(17.0f), glm::vec3{0.0f, 0.0f, 1.0f}) *
            glm::rotate(glm::mat4{1.0f}, glm::radians(-23.0f), glm::vec3{1.0f, 0.0f, 0.0f});
        const auto basis = glm::mat3{rotation};
        const glm::vec3 u = basis[0], normal = basis[1], v = basis[2];
        CameraConfig camera;
        camera.position = normal * 3.0f + u * 0.6f + v;
        camera.target = glm::vec3{0.0f};
        renderer.setCamera(camera);
        ToonStyleConfig style;
        style.outlineWidthPixels = 0.0f;
        style.shadowStrength = 1.0f;
        // From near-normal incidence to grazing, with both depth-gradient
        // components and both signs. A lone plane cannot shadow its interior.
        const std::array<glm::vec3, 4> directions{
            normal + u * 0.3f + v * 0.5f,
            normal * 0.35f + u * 0.9f + v * 0.4f,
            normal * 0.12f - u * 1.1f + v * 0.7f,
            normal * 0.06f + u * 0.7f + v * 0.9f};
        std::array<glm::vec3, 25> points{};
        std::size_t point = 0;
        for (int z = -2; z <= 2; ++z)
            for (int x = -2; x <= 2; ++x)
                points[point++] = u * (static_cast<float>(x) * 0.3f) +
                                  v * (static_cast<float>(z) * 0.3f);
        for (const auto& direction : directions)
        {
            style.keyLightDirection = direction;
            style.selfShadows = false;
            renderer.setStyleConfig(style);
            renderer.beginFrame();
            renderer.renderModel(plane, rotation);
            std::array<glm::vec3, 25> baseline{};
            for (std::size_t i = 0; i < points.size(); ++i)
            {
                baseline[i] = target.sample(points[i], camera);
                JULIA_CHECK(baseline[i].r > 0.4f);
            }
            style.selfShadows = true;
            renderer.setStyleConfig(style);
            renderer.beginFrame();
            renderer.renderModel(plane, rotation);
            for (std::size_t i = 0; i < points.size(); ++i)
            {
                const auto actual = target.sample(points[i], camera);
                const auto difference = glm::abs(actual - baseline[i]);
                const float worst = std::max({difference.r, difference.g, difference.b});
                if (worst >= 0.005f)
                    std::cerr << "Planar shadow acne: sample " << i << ", error " << worst
                              << ", normal/light " << glm::dot(normal, glm::normalize(direction)) << '\n';
                JULIA_CHECK(worst < 0.005f);
            }
            JULIA_CHECK(glGetError() == GL_NO_ERROR);
        }
    }

    void testSkinnedCasterOutsideBindBounds(RenderTarget& target, OpenGLRenderer& renderer)
    {
        GLModel model;
        model.addMesh(horizontalQuad(1.2f, 0.0f, matteMaterial()));
        model.addMesh(horizontalQuad(0.25f, 0.6f, matteMaterial(), 0));
        GLModelNode node;
        node.name = "moving caster";
        GLBoneInfo bone;
        bone.name = node.name;
        bone.nodeIndex = model.addNode(std::move(node));
        model.addBone(std::move(bone));
        CameraConfig camera;
        camera.position = {2.0f, 2.4f, 3.4f};
        camera.target = glm::vec3{0.0f};
        renderer.setCamera(camera);
        ToonStyleConfig style;
        style.outlineWidthPixels = 0.0f;
        style.keyLightDirection = {0.3f, 1.0f, 0.15f};
        std::array<glm::mat4, 1> palette{glm::translate(glm::mat4{1.0f}, glm::vec3{0.0f, 1.8f, 0.0f})};
        const glm::vec3 receiver{-0.72f, 0.0f, -0.36f};
        const glm::vec3 bindReceiver{-0.18f, 0.0f, -0.09f};
        const auto render = [&] {
            renderer.setStyleConfig(style);
            renderer.beginFrame();
            renderer.renderModel(model, glm::mat4{1.0f}, palette);
            JULIA_CHECK(glGetError() == GL_NO_ERROR);
        };
        // The animated caster is at Y=2.4, well above bind bounds.max.y=0.6.
        // The unweighted receiver remains static while the map fit expands.
        JULIA_CHECK(model.bounds().max.y < 1.0f);
        style.selfShadows = false;
        render();
        const auto baseline = target.sample(receiver, camera);
        style.selfShadows = true;
        render();
        JULIA_CHECK(target.sample(receiver, camera).r < baseline.r - 0.05f);
        JULIA_CHECK(glm::length(target.sample(bindReceiver, camera) - baseline) < 0.012f);
        palette[0] = glm::mat4{1.0f};
        render();
        JULIA_CHECK(glm::length(target.sample(receiver, camera) - baseline) < 0.012f);
        JULIA_CHECK(target.sample(bindReceiver, camera).r < baseline.r - 0.05f);
    }
}

int main()
{
    if (!glfwInit()) return 1;
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(64, 64, "JULIA toon render tests", nullptr, nullptr);
    if (!window) { glfwTerminate(); return 1; }
    glfwMakeContextCurrent(window);
    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)))
    {
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }
    int result = 0;
    try
    {
        RenderTarget target;
        OpenGLRenderer renderer;
        renderer.initialize(ImageSize, ImageSize);
        glDisable(GL_DITHER);
        testSelfShadowAndAlpha(target, renderer);
        testLightingAndMaterials(target, renderer);
        testPlanarSelfShadowAcne(target, renderer);
        testSkinnedCasterOutsideBindBounds(target, renderer);
        result = JULIA_TEST_RESULT();
    }
    catch (const std::exception& error)
    {
        std::cerr << "Toon render regression exception: " << error.what() << '\n';
        result = 1;
    }
    glfwDestroyWindow(window);
    glfwTerminate();
    return result;
}
