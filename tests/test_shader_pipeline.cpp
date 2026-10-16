#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "julia/render_opengl/GLMesh.hpp"
#include "julia/render_opengl/GLShader.hpp"
#include "julia/render_opengl/OpenGLRenderer.hpp"
#include "julia_test.hpp"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
    using julia::render_opengl::GLShader;
    using julia::render_opengl::GLVertex;

    std::string readText(const std::filesystem::path& path)
    {
        std::ifstream file(path);
        if (!file) throw std::runtime_error("Cannot read shader: " + path.string());
        std::ostringstream contents;
        contents << file.rdbuf();
        return contents.str();
    }

    void checkVector(const glm::vec3& actual, const glm::vec3& expected)
    {
        constexpr float tolerance = 0.0002f;
        const bool matches = std::isfinite(actual.x) && std::isfinite(actual.y) &&
            std::isfinite(actual.z) && glm::length(actual - expected) < tolerance;
        if (!matches)
            std::cerr << "Vector mismatch: (" << actual.x << ", " << actual.y
                      << ", " << actual.z << "), expected (" << expected.x
                      << ", " << expected.y << ", " << expected.z << ")\n";
        JULIA_CHECK(matches);
    }

    void testShaderLifetime()
    {
        constexpr const char* vertex = R"glsl(#version 330 core
            uniform float uCached;
            void main() { gl_Position = vec4(uCached, 0.0, 0.0, 1.0); }
        )glsl";
        constexpr const char* fragment = R"glsl(#version 330 core
            out vec4 color;
            void main() { color = vec4(1.0); }
        )glsl";
        GLShader shader;
        shader.createFromSource(vertex, fragment);
        shader.bind();
        shader.setFloat("uCached", 0.25f);
        const GLuint original = shader.id();

        const auto failedReplacement = [&](const char* replacementVertex,
                                           const char* replacementFragment) {
            bool threw = false;
            try { shader.createFromSource(replacementVertex, replacementFragment); }
            catch (const std::exception& error)
            {
                threw = true;
                JULIA_CHECK(std::string{error.what()}.size() > 20);
            }
            JULIA_CHECK(threw);
            JULIA_CHECK(shader.valid());
            JULIA_CHECK(shader.id() == original);
            JULIA_CHECK(glIsProgram(original) == GL_TRUE);
            shader.bind();
            shader.setFloat("uCached", 0.5f);
            GLfloat value = 0.0f;
            glGetUniformfv(shader.id(), glGetUniformLocation(shader.id(), "uCached"), &value);
            JULIA_CHECK(std::abs(value - 0.5f) < 0.0001f);
        };

        failedReplacement("#version 330 core\n this is not valid GLSL", fragment);
        failedReplacement(vertex, "#version 330 core\n this is not valid GLSL");
        failedReplacement(R"glsl(#version 330 core
            uniform float uLinkType;
            void main() { gl_Position = vec4(uLinkType, 0.0, 0.0, 1.0); }
        )glsl", R"glsl(#version 330 core
            uniform vec4 uLinkType;
            out vec4 color;
            void main() { color = uLinkType; }
        )glsl");

        GLShader moved{std::move(shader)};
        JULIA_CHECK(!shader.valid());
        JULIA_CHECK(moved.id() == original);
        moved.bind();
        moved.setFloat("uCached", 0.75f);

        // Reusing the moved-from object must not reuse its old uniform cache.
        constexpr const char* replacement = R"glsl(#version 330 core
            uniform vec4 uLeading[8];
            uniform float uCached;
            void main() {
                vec4 sum = vec4(0.0);
                for (int i = 0; i < 8; ++i) sum += uLeading[i];
                gl_Position = sum + vec4(uCached, 0.0, 0.0, 1.0);
            }
        )glsl";
        shader.createFromSource(replacement, fragment);
        shader.bind();
        shader.setFloat("uCached", 0.375f);
        GLfloat value = 0.0f;
        glGetUniformfv(shader.id(), glGetUniformLocation(shader.id(), "uCached"), &value);
        JULIA_CHECK(std::abs(value - 0.375f) < 0.0001f);

        const GLuint overwritten = shader.id();
        shader = std::move(moved);
        JULIA_CHECK(!moved.valid());
        JULIA_CHECK(shader.id() == original);
        shader.bind();
        JULIA_CHECK(glIsProgram(overwritten) == GL_FALSE);
        shader.setFloat("uCached", 0.625f);
        glGetUniformfv(shader.id(), glGetUniformLocation(shader.id(), "uCached"), &value);
        JULIA_CHECK(std::abs(value - 0.625f) < 0.0001f);
        const GLuint beforeReload = shader.id();
        shader.createFromSource(replacement, fragment);
        GLint current = 0;
        glGetIntegerv(GL_CURRENT_PROGRAM, &current);
        JULIA_CHECK(static_cast<GLuint>(current) == shader.id());
        JULIA_CHECK(glIsProgram(beforeReload) == GL_FALSE);
        shader.setFloat("uCached", 0.875f);
        glGetUniformfv(shader.id(), glGetUniformLocation(shader.id(), "uCached"), &value);
        JULIA_CHECK(std::abs(value - 0.875f) < 0.0001f);
        JULIA_CHECK(glGetError() == GL_NO_ERROR);
        glUseProgram(0);

        GLShader arrays;
        arrays.createFromSource(R"glsl(#version 330 core
            uniform mat4 uMatrices[2];
            uniform vec4 uWrongType[2];
            void main() {
                gl_Position = uMatrices[0] * uMatrices[1] * uWrongType[0] + uWrongType[1];
            }
        )glsl", fragment);
        arrays.bind();
        const std::array<glm::mat4, 2> validMatrices{
            glm::mat4{1.0f}, glm::translate(glm::mat4{1.0f}, {0.2f, -0.3f, 0.4f})};
        arrays.setMat4Array("uMatrices", validMatrices);
        const std::array<glm::mat4, 3> oversizedMatrices{
            glm::mat4{2.0f}, glm::mat4{2.0f}, glm::mat4{2.0f}};
        bool rejectedOversize = false;
        try { arrays.setMat4Array("uMatrices", oversizedMatrices); }
        catch (const std::invalid_argument&) { rejectedOversize = true; }
        JULIA_CHECK(rejectedOversize);
        bool rejectedType = false;
        try { arrays.setMat4Array("uWrongType", validMatrices); }
        catch (const std::invalid_argument&) { rejectedType = true; }
        JULIA_CHECK(rejectedType);
        std::array<float, 16> matrixValue{};
        glGetUniformfv(arrays.id(), glGetUniformLocation(arrays.id(), "uMatrices[1]"), matrixValue.data());
        JULIA_CHECK(std::abs(matrixValue[12] - 0.2f) < 0.0001f);
        JULIA_CHECK(std::abs(matrixValue[13] + 0.3f) < 0.0001f);
        JULIA_CHECK(std::abs(matrixValue[14] - 0.4f) < 0.0001f);
        JULIA_CHECK(glGetError() == GL_NO_ERROR);
        glUseProgram(0);
    }

    GLuint compileStage(GLenum type, const std::string& source)
    {
        const GLuint stage = glCreateShader(type);
        const char* pointer = source.c_str();
        glShaderSource(stage, 1, &pointer, nullptr);
        glCompileShader(stage);
        GLint compiled = GL_FALSE;
        glGetShaderiv(stage, GL_COMPILE_STATUS, &compiled);
        if (compiled == GL_FALSE)
        {
            GLint length = 0;
            glGetShaderiv(stage, GL_INFO_LOG_LENGTH, &length);
            std::string log(static_cast<std::size_t>(length), '\0');
            glGetShaderInfoLog(stage, length, nullptr, log.data());
            glDeleteShader(stage);
            throw std::runtime_error("Transform feedback shader compile failed: " + log);
        }
        return stage;
    }

    struct FeedbackResult
    {
        glm::vec3 position;
        glm::vec3 normal;
        glm::vec4 clip;
    };

    // Capture the actual model vertex shader, so tests exercise its transforms
    // independently of lighting, rasterization, and the imported FBX fixture.
    class VertexFeedback
    {
    public:
        VertexFeedback(const std::string& vertex, const std::string& fragment)
        {
            const GLuint vertexStage = compileStage(GL_VERTEX_SHADER, vertex);
            GLuint fragmentStage = 0;
            try { fragmentStage = compileStage(GL_FRAGMENT_SHADER, fragment); }
            catch (...) { glDeleteShader(vertexStage); throw; }
            program_ = glCreateProgram();
            glAttachShader(program_, vertexStage);
            glAttachShader(program_, fragmentStage);
            constexpr const char* varyings[] = {"vWorldPosition", "vNormal", "gl_Position"};
            glTransformFeedbackVaryings(program_, 3, varyings, GL_INTERLEAVED_ATTRIBS);
            glLinkProgram(program_);
            glDeleteShader(vertexStage);
            glDeleteShader(fragmentStage);
            GLint linked = GL_FALSE;
            glGetProgramiv(program_, GL_LINK_STATUS, &linked);
            if (linked == GL_FALSE)
            {
                GLint length = 0;
                glGetProgramiv(program_, GL_INFO_LOG_LENGTH, &length);
                std::string log(static_cast<std::size_t>(length), '\0');
                glGetProgramInfoLog(program_, length, nullptr, log.data());
                glDeleteProgram(program_);
                program_ = 0;
                throw std::runtime_error("Transform feedback shader link failed: " + log);
            }

            glGenVertexArrays(1, &vao_);
            glGenBuffers(1, &vertexBuffer_);
            glGenBuffers(1, &feedbackBuffer_);
            glGenBuffers(1, &paletteBuffer_);
            glGenTextures(1, &paletteTexture_);
            glBindVertexArray(vao_);
            glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer_);
            constexpr auto stride = static_cast<GLsizei>(sizeof(GLVertex));
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride,
                reinterpret_cast<void*>(offsetof(GLVertex, position)));
            glEnableVertexAttribArray(1);
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride,
                reinterpret_cast<void*>(offsetof(GLVertex, normal)));
            glEnableVertexAttribArray(2);
            glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, stride,
                reinterpret_cast<void*>(offsetof(GLVertex, texCoord)));
            glEnableVertexAttribArray(3);
            glVertexAttribIPointer(3, 4, GL_INT, stride,
                reinterpret_cast<void*>(offsetof(GLVertex, boneIds)));
            glEnableVertexAttribArray(4);
            glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, stride,
                reinterpret_cast<void*>(offsetof(GLVertex, boneWeights)));
            glBindVertexArray(0);
        }

        ~VertexFeedback()
        {
            glDeleteTextures(1, &paletteTexture_);
            glDeleteBuffers(1, &paletteBuffer_);
            glDeleteBuffers(1, &feedbackBuffer_);
            glDeleteBuffers(1, &vertexBuffer_);
            glDeleteVertexArrays(1, &vao_);
            glDeleteProgram(program_);
        }

        VertexFeedback(const VertexFeedback&) = delete;
        VertexFeedback& operator=(const VertexFeedback&) = delete;

        FeedbackResult capture(const GLVertex& vertex, const glm::mat4& model,
                               std::span<const glm::mat4> palette, bool useSkinning,
                               const glm::mat4& view = glm::mat4{1.0f},
                               const glm::mat4& projection = glm::mat4{1.0f},
                               bool outline = false, glm::vec2 viewport = {640.0f, 480.0f})
        {
            glUseProgram(program_);
            const glm::mat4 identity{1.0f};
            const glm::mat3 normalMatrix = glm::transpose(glm::inverse(glm::mat3{model}));
            const auto location = [&](const char* name) { return glGetUniformLocation(program_, name); };
            glUniformMatrix4fv(location("uModel"), 1, GL_FALSE, glm::value_ptr(model));
            glUniformMatrix4fv(location("uView"), 1, GL_FALSE, glm::value_ptr(view));
            glUniformMatrix4fv(location("uProjection"), 1, GL_FALSE, glm::value_ptr(projection));
            glUniformMatrix3fv(location("uNormalMatrix"), 1, GL_FALSE, glm::value_ptr(normalMatrix));
            glUniform1i(location("uUseSkinning"), useSkinning ? 1 : 0);
            glUniform1i(location("uBoneCount"), static_cast<GLint>(palette.size()));
            glUniform1i(location("uOutlinePass"), outline ? 1 : 0);
            glUniform1i(location("uBonePalette"), 0);
            // Active samplers of different types must use distinct units even
            // when rasterizer discard prevents fragment shader execution.
            glUniform1i(location("uDiffuseTexture"), 1);
            glUniform1i(location("uShadowMap"), 2);
            glUniform3f(location("uViewport"), viewport.x, viewport.y, 0.0f);
            glUniform1f(location("uOutlineWidthPixels"), 2.4f);

            glBindBuffer(GL_TEXTURE_BUFFER, paletteBuffer_);
            glBufferData(GL_TEXTURE_BUFFER,
                static_cast<GLsizeiptr>(std::max(palette.size(), std::size_t{1}) * sizeof(glm::mat4)),
                palette.empty() ? glm::value_ptr(identity) : glm::value_ptr(palette.front()), GL_STREAM_DRAW);
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_BUFFER, paletteTexture_);
            glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, paletteBuffer_);

            glBindVertexArray(vao_);
            glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer_);
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLVertex), &vertex, GL_STREAM_DRAW);
            glBindBuffer(GL_TRANSFORM_FEEDBACK_BUFFER, feedbackBuffer_);
            glBufferData(GL_TRANSFORM_FEEDBACK_BUFFER, 10 * sizeof(float), nullptr, GL_STREAM_READ);
            glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, feedbackBuffer_);
            glEnable(GL_RASTERIZER_DISCARD);
            glBeginTransformFeedback(GL_POINTS);
            glDrawArrays(GL_POINTS, 0, 1);
            glEndTransformFeedback();
            glDisable(GL_RASTERIZER_DISCARD);
            std::array<float, 10> output{};
            glGetBufferSubData(GL_TRANSFORM_FEEDBACK_BUFFER, 0, sizeof(output), output.data());
            glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, 0);
            glBindVertexArray(0);
            glBindTexture(GL_TEXTURE_BUFFER, 0);
            glUseProgram(0);
            JULIA_CHECK(glGetError() == GL_NO_ERROR);
            return {{output[0], output[1], output[2]}, {output[3], output[4], output[5]},
                {output[6], output[7], output[8], output[9]}};
        }

    private:
        GLuint program_ = 0;
        GLuint vao_ = 0;
        GLuint vertexBuffer_ = 0;
        GLuint feedbackBuffer_ = 0;
        GLuint paletteBuffer_ = 0;
        GLuint paletteTexture_ = 0;
    };

    void testVertexTransforms(const std::string& vertexSource, const std::string& fragmentSource)
    {
        VertexFeedback feedback{vertexSource, fragmentSource};
        GLVertex vertex;
        vertex.position = {0.2f, -0.3f, 0.4f};
        vertex.normal = glm::normalize(glm::vec3{1.0f, 2.0f, 3.0f});
        const glm::mat4 identity{1.0f};
        const glm::mat4 model = glm::translate(identity, {0.1f, 0.2f, -0.1f}) *
            glm::rotate(identity, 0.4f, glm::normalize(glm::vec3{1.0f, 2.0f, 1.0f})) *
            glm::scale(identity, {1.2f, 0.7f, 1.8f});

        const auto check = [&](const glm::mat4& expectedSkin, std::span<const glm::mat4> palette,
                               bool skinning = true) {
            const auto result = feedback.capture(vertex, model, palette, skinning);
            checkVector(result.position, glm::vec3{model * expectedSkin * glm::vec4{vertex.position, 1.0f}});
            checkVector(result.normal, glm::normalize(
                glm::transpose(glm::inverse(glm::mat3{model * expectedSkin})) * vertex.normal));
        };

        check(identity, {}, false);
        std::array<glm::mat4, 2> bones{
            glm::translate(identity, {0.3f, 0.0f, -0.2f}) * glm::scale(identity, {2.0f, 1.0f, 0.5f}),
            glm::translate(identity, {-0.2f, 0.1f, 0.3f}) * glm::scale(identity, {1.0f, 2.0f, 1.0f})
        };
        vertex.boneIds = {0, -1, -1, -1};
        vertex.boneWeights = {1.0f, 0.0f, 0.0f, 0.0f};
        check(bones[0], bones);
        vertex.boneIds = {0, 1, -1, -1};
        // Non-unit sums must be renormalized after selecting valid influences.
        vertex.boneWeights = {0.5f, 1.5f, 0.0f, 0.0f};
        check(bones[0] * 0.25f + bones[1] * 0.75f, bones);
        vertex.boneWeights = glm::vec4{0.0f};
        check(identity, bones);
        vertex.boneIds = {-1, 999999, 1, 0};
        vertex.boneWeights = {0.1f, 0.2f, 0.5f, 0.2f};
        check(bones[0], std::span<const glm::mat4>{bones}.first(1));
        // Empty palettes and entirely invalid influences leave static geometry.
        check(identity, {});
        vertex.boneIds = {-1, 999999, -5, 2};
        check(identity, bones);
        // Buffer palettes must work beyond the former 128-matrix uniform cap.
        std::vector<glm::mat4> largePalette(257, identity);
        largePalette.back() = glm::translate(identity, {0.1f, 0.2f, 0.3f});
        vertex.boneIds = {256, -1, -1, -1};
        vertex.boneWeights = {1.0f, 0.0f, 0.0f, 0.0f};
        check(largePalette.back(), largePalette);
        bones[0] = glm::scale(identity, {-1.0f, 2.0f, 0.5f});
        vertex.boneIds = {0, -1, -1, -1};
        check(bones[0], bones);
        // Normals can collapse under a singular blend (opposing rotations).
        // They must still be finite so later normalize/dot cannot poison pixels.
        vertex.boneIds = {0, 1, -1, -1};
        vertex.boneWeights = {0.5f, 0.5f, 0.0f, 0.0f};
        bones = {identity, glm::scale(identity, {-1.0f, -1.0f, 1.0f})};
        const auto singular = feedback.capture(vertex, model, bones, true);
        JULIA_CHECK(std::isfinite(singular.normal.x));
        JULIA_CHECK(std::isfinite(singular.normal.y));
        JULIA_CHECK(std::isfinite(singular.normal.z));
        JULIA_CHECK(std::abs(glm::length(singular.normal) - 1.0f) < 0.0002f);

        const glm::mat4 view = glm::lookAt(glm::vec3{2.0f, 1.0f, 4.0f}, glm::vec3{0.0f},
            glm::vec3{0.0f, 1.0f, 0.0f});
        for (const glm::vec2 viewport : {glm::vec2{640.0f, 480.0f}, glm::vec2{1280.0f, 720.0f},
                                       glm::vec2{480.0f, 640.0f}})
        {
            const glm::mat4 projection = glm::perspective(glm::radians(45.0f),
                viewport.x / viewport.y, 0.1f, 50.0f);
            const auto plain = feedback.capture(vertex, model, {}, false, view, projection, false, viewport);
            const auto outlined = feedback.capture(vertex, model, {}, false, view, projection, true, viewport);
            const glm::vec4 expected = projection * view * model * glm::vec4{vertex.position, 1.0f};
            JULIA_CHECK(glm::length(plain.clip - expected) < 0.0002f);
            const glm::vec2 plainPixel = glm::vec2{plain.clip} / plain.clip.w * viewport * 0.5f;
            const glm::vec2 outlinePixel = glm::vec2{outlined.clip} / outlined.clip.w * viewport * 0.5f;
            const float taper = glm::mix(1.0f, 0.8f, std::abs((glm::mat3{view} * plain.normal).z));
            JULIA_CHECK(std::abs(glm::length(outlinePixel - plainPixel) - 2.4f * taper) < 0.002f);
        }
    }

    class PixelTarget
    {
    public:
        explicit PixelTarget(bool srgb = false)
        {
            glGenFramebuffers(1, &framebuffer_);
            glGenTextures(1, &color_);
            glGenRenderbuffers(1, &depth_);
            glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);
            glBindTexture(GL_TEXTURE_2D, color_);
            glTexImage2D(GL_TEXTURE_2D, 0, srgb ? GL_SRGB8_ALPHA8 : GL_RGBA8,
                64, 64, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, color_, 0);
            glBindRenderbuffer(GL_RENDERBUFFER, depth_);
            glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, 64, 64);
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_);
            glDrawBuffer(GL_COLOR_ATTACHMENT0);
            glReadBuffer(GL_COLOR_ATTACHMENT0);
            JULIA_CHECK(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);
            glViewport(0, 0, 64, 64);
            glDisable(GL_FRAMEBUFFER_SRGB);
            glDisable(GL_DITHER);
        }

        ~PixelTarget()
        {
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            glDeleteRenderbuffers(1, &depth_);
            glDeleteTextures(1, &color_);
            glDeleteFramebuffers(1, &framebuffer_);
        }

        PixelTarget(const PixelTarget&) = delete;
        PixelTarget& operator=(const PixelTarget&) = delete;

        void clear(const glm::vec4& value) const
        {
            glDisable(GL_SCISSOR_TEST);
            glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
            glDepthMask(GL_TRUE);
            glClearColor(value.r, value.g, value.b, value.a);
            glClearDepth(1.0);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        }

        glm::vec4 read(int x = 32, int y = 32) const
        {
            glm::vec4 pixel{0.0f};
            glReadPixels(x, y, 1, 1, GL_RGBA, GL_FLOAT, glm::value_ptr(pixel));
            JULIA_CHECK(glGetError() == GL_NO_ERROR);
            return pixel;
        }

        std::array<unsigned char, 64 * 64 * 4> pixels() const
        {
            std::array<unsigned char, 64 * 64 * 4> result{};
            glReadPixels(0, 0, 64, 64, GL_RGBA, GL_UNSIGNED_BYTE, result.data());
            JULIA_CHECK(glGetError() == GL_NO_ERROR);
            return result;
        }

    private:
        GLuint framebuffer_ = 0;
        GLuint color_ = 0;
        GLuint depth_ = 0;
    };

    void checkPixel(const glm::vec4& actual, const glm::vec4& expected)
    {
        constexpr float tolerance = 0.015f;
        const bool matches = glm::all(glm::lessThan(glm::abs(actual - expected), glm::vec4{tolerance}));
        if (!matches)
            std::cerr << "Pixel mismatch: (" << actual.r << ", " << actual.g << ", "
                      << actual.b << ", " << actual.a << "), expected (" << expected.r
                      << ", " << expected.g << ", " << expected.b << ", " << expected.a << ")\n";
        JULIA_CHECK(matches);
    }

    void testToonTransition(GLShader& shader)
    {
        PixelTarget target;
        const glm::mat4 identity{1.0f};
        shader.bind();
        shader.setMat4("uModel", identity);
        shader.setMat4("uView", identity);
        shader.setMat4("uProjection", identity);
        shader.setMat3("uNormalMatrix", glm::mat3{1.0f});
        shader.setBool("uUseSkinning", false);
        shader.setInt("uBoneCount", 0);
        shader.setInt("uBonePalette", 1);
        shader.setBool("uOutlinePass", false);
        shader.setVec3("uViewPos", {0.0f, 0.0f, 3.0f});
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_CULL_FACE);
        glDisable(GL_BLEND);
        const auto light = glm::normalize(glm::vec3{0.34f, 0.78f, 0.52f});
        const auto tangent = glm::normalize(glm::cross(light, glm::vec3{0.0f, 1.0f, 0.0f}));
        julia::render_opengl::GLMeshMaterial material;
        material.diffuseColor = glm::vec3{0.4f};
        material.lightResponse = 0.8f;
        material.specularStrength = 0.0f;
        material.rimStrength = 0.0f;
        const auto sample = [&](float ndl) {
            const auto n = light * ndl + tangent * std::sqrt(1.0f - ndl * ndl);
            std::vector<GLVertex> vertices(3);
            vertices[0].position = {-1.0f, -1.0f, 0.0f};
            vertices[1].position = {3.0f, -1.0f, 0.0f};
            vertices[2].position = {-1.0f, 3.0f, 0.0f};
            for (auto& vertex : vertices) vertex.normal = n;
            julia::render_opengl::GLMesh mesh{std::move(vertices), {0, 1, 2}, material};
            target.clear(glm::vec4{0.0f});
            mesh.draw(shader);
            return target.read().r;
        };
        constexpr float edge = 0.10f - 0.8f * 0.18f;
        const float shadow = sample(edge - 0.08f);
        const float lit = sample(edge + 0.08f);
        const float low = sample(edge - 0.0125f);
        const float mid = sample(edge);
        const float high = sample(edge + 0.0125f);
        const float contrast = lit - shadow;
        JULIA_CHECK(contrast > 0.10f);
        // Constant normals give fwidth=0: the narrow lighting transition must
        // still exist at close range instead of snapping between flat colors.
        JULIA_CHECK(low > shadow + contrast * 0.05f);
        JULIA_CHECK(low < mid && mid < high);
        JULIA_CHECK(high < lit - contrast * 0.05f);
        JULIA_CHECK(std::abs(mid - (lit + shadow) * 0.5f) < 0.01f);
        // The broad toon fields remain flat outside the small transition.
        JULIA_CHECK(std::abs(sample(edge + 0.12f) - lit) < 0.005f);
        JULIA_CHECK(glGetError() == GL_NO_ERROR);
    }

    void testFragmentAlpha(GLShader& shader)
    {
        PixelTarget target;
        GLuint vao = 0;
        GLuint vbo = 0;
        GLuint texture = 0;
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glGenTextures(1, &texture);
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        constexpr float triangle[] = {-1.0f, -1.0f, 0.0f, 3.0f, -1.0f, 0.0f, -1.0f, 3.0f, 0.0f};
        glBufferData(GL_ARRAY_BUFFER, sizeof(triangle), triangle, GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
        glVertexAttrib3f(1, 0.0f, 0.0f, 1.0f);
        glVertexAttrib2f(2, 0.5f, 0.5f);
        glVertexAttribI4i(3, -1, -1, -1, -1);
        glVertexAttrib4f(4, 0.0f, 0.0f, 0.0f, 0.0f);
        shader.bind();
        const glm::mat4 identity{1.0f};
        shader.setMat4("uModel", identity);
        shader.setMat4("uView", identity);
        shader.setMat4("uProjection", identity);
        shader.setMat3("uNormalMatrix", glm::mat3{1.0f});
        shader.setBool("uUseSkinning", false);
        shader.setInt("uBoneCount", 0);
        shader.setInt("uBonePalette", 1);
        shader.setVec3("uViewport", {64.0f, 64.0f, 0.0f});
        shader.setFloat("uOutlineWidthPixels", 0.0f);
        shader.setVec3("uViewPos", {0.0f, 0.0f, 3.0f});
        shader.setVec3("uDiffuseColor", {0.4f, 0.3f, 0.2f});
        shader.setBool("uHasDiffuseTexture", true);
        shader.setInt("uDiffuseTexture", 0);
        shader.setFloat("uLightResponse", 0.8f);
        shader.setFloat("uSpecularStrength", 0.0f);
        shader.setFloat("uRimStrength", 0.0f);
        shader.setFloat("uAlphaCutoff", 0.5f);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_CULL_FACE);
        glDisable(GL_BLEND);
        const glm::vec4 background{0.13f, 0.19f, 0.29f, 0.7f};
        const auto draw = [&](int alphaMode, float opacity, unsigned char textureAlpha,
                              bool outline = false, bool blend = false) {
            target.clear(background);
            shader.setInt("uAlphaMode", alphaMode);
            shader.setFloat("uOpacity", opacity);
            shader.setBool("uOutlinePass", outline);
            const std::array<unsigned char, 4> texel{255, 255, 255, textureAlpha};
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, texel.data());
            if (blend)
            {
                glEnable(GL_BLEND);
                glBlendEquationSeparate(GL_FUNC_ADD, GL_FUNC_ADD);
                glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
            }
            else glDisable(GL_BLEND);
            glDrawArrays(GL_TRIANGLES, 0, 3);
            return target.read();
        };
        const auto opaque = draw(0, 0.0f, 0);
        JULIA_CHECK(opaque.a > 0.99f);
        JULIA_CHECK(glm::length(glm::vec3{opaque - background}) > 0.05f);
        checkPixel(draw(1, 1.0f, 64), background);
        checkPixel(draw(1, 1.0f, 64, true), background);
        JULIA_CHECK(draw(1, 1.0f, 192).a > 0.99f);
        checkPixel(draw(2, 1.0f, 0), background);
        checkPixel(draw(2, 1.0f, 0, true), background);
        const auto source = draw(2, 0.5f, 128);
        const float alpha = (128.0f / 255.0f) * 0.5f;
        JULIA_CHECK(std::abs(source.a - alpha) < 0.005f);
        checkPixel(draw(2, 0.5f, 128, false, true), glm::vec4{
            glm::vec3{source} * alpha + glm::vec3{background} * (1.0f - alpha),
            alpha + background.a * (1.0f - alpha)});
        glDisable(GL_BLEND);
        glBindVertexArray(0);
        glUseProgram(0);
        glDeleteTextures(1, &texture);
        glDeleteBuffers(1, &vbo);
        glDeleteVertexArrays(1, &vao);
    }

    template<typename Function>
    void expectException(Function&& function)
    {
        bool threw = false;
        try { std::forward<Function>(function)(); }
        catch (const std::exception&) { threw = true; }
        JULIA_CHECK(threw);
    }

    julia::render_opengl::GLMesh triangleMesh(
        float depth = 0.0f,
        julia::render_opengl::GLMeshMaterial material = {})
    {
        using namespace julia::render_opengl;
        std::vector<GLVertex> vertices(3);
        vertices[0].position = {-0.6f, -0.6f, depth};
        vertices[1].position = {0.6f, -0.6f, depth};
        vertices[2].position = {0.0f, 0.6f, depth};
        for (auto& vertex : vertices) vertex.normal = {0.0f, 0.0f, 1.0f};
        return GLMesh{std::move(vertices), {0, 1, 2}, std::move(material)};
    }

    julia::render_opengl::GLModel triangleModel(int boneCount = 0)
    {
        using namespace julia::render_opengl;
        GLModel model;
        model.addMesh(triangleMesh());
        for (int i = 0; i < boneCount; ++i)
        {
            GLModelNode node;
            node.name = "test-node-" + std::to_string(i);
            GLBoneInfo bone;
            bone.name = node.name;
            bone.nodeIndex = model.addNode(std::move(node));
            model.addBone(std::move(bone));
        }
        return model;
    }

    void testTransparentOrdering()
    {
        using namespace julia::render_opengl;
        PixelTarget target{true};
        OpenGLRenderer renderer;
        renderer.initialize(64, 64);
        CameraConfig camera;
        camera.position = {0.0f, 0.0f, 3.0f};
        camera.target = {0.0f, 0.0f, 0.0f};
        renderer.setCamera(camera);
        ToonStyleConfig style;
        const auto invalidStyle = [&](const ToonStyleConfig& value) {
            expectException([&] { renderer.setStyleConfig(value); });
            renderer.setStyleConfig(style);
        };
        auto badStyle = style;
        badStyle.keyLightDirection = glm::vec3{0.0f};
        invalidStyle(badStyle);
        badStyle = style;
        badStyle.keyLightColor.x = -1.0f;
        invalidStyle(badStyle);
        badStyle = style;
        badStyle.shadowTint.y = 1.1f;
        invalidStyle(badStyle);
        badStyle = style;
        badStyle.shadowStrength = 1.1f;
        invalidStyle(badStyle);
        badStyle = style;
        badStyle.outlineWidthPixels = std::numeric_limits<float>::quiet_NaN();
        invalidStyle(badStyle);
        GLMeshMaterial nearMaterial;
        nearMaterial.diffuseColor = {0.8f, 0.02f, 0.02f};
        nearMaterial.specularStrength = 0.0f;
        nearMaterial.rimStrength = 0.0f;
        nearMaterial.opacity = 0.5f;
        nearMaterial.alphaMode = AlphaMode::Blend;
        auto farMaterial = nearMaterial;
        farMaterial.diffuseColor = {0.02f, 0.02f, 0.8f};
        GLModel frontFirst;
        frontFirst.addMesh(triangleMesh(0.0f, nearMaterial));
        frontFirst.addMesh(triangleMesh(-0.5f, farMaterial));
        GLModel backFirst;
        backFirst.addMesh(triangleMesh(-0.5f, farMaterial));
        backFirst.addMesh(triangleMesh(0.0f, nearMaterial));
        const glm::mat4 identity{1.0f};
        const auto render = [&](const GLModel& model) {
            renderer.beginFrame();
            renderer.renderModel(model, identity);
            return target.read();
        };
        const auto a = render(frontFirst);
        const auto b = render(backFirst);
        checkPixel(a, b);
        JULIA_CHECK(a.r > a.b + 0.1f);
        // A transparent layer must not occlude a later opaque model by writing
        // its depth. The second opaque draw is farther away but must remain visible.
        GLModel transparentNear;
        transparentNear.addMesh(triangleMesh(0.0f, nearMaterial));
        auto opaqueMaterial = farMaterial;
        opaqueMaterial.alphaMode = AlphaMode::Opaque;
        opaqueMaterial.opacity = 1.0f;
        GLModel opaqueFar;
        opaqueFar.addMesh(triangleMesh(-0.5f, opaqueMaterial));
        renderer.beginFrame();
        renderer.renderModel(transparentNear, identity);
        renderer.renderModel(opaqueFar, identity);
        const auto opaqueAfterTransparent = target.read();
        const auto opaqueOnly = render(opaqueFar);
        checkPixel(opaqueAfterTransparent, opaqueOnly);
    }

    void testReflectedSkinOrientation()
    {
        using namespace julia::render_opengl;
        PixelTarget target{true};
        OpenGLRenderer renderer;
        renderer.initialize(64, 64);
        CameraConfig camera;
        camera.position = {1.7f, 0.9f, 3.0f};
        camera.target = {0.0f, 0.0f, 0.0f};
        renderer.setCamera(camera);
        constexpr glm::vec3 corners[] = {
            {-0.5f, -0.5f, -0.5f}, {0.5f, -0.5f, -0.5f},
            {0.5f, 0.5f, -0.5f}, {-0.5f, 0.5f, -0.5f},
            {-0.5f, -0.5f, 0.5f}, {0.5f, -0.5f, 0.5f},
            {0.5f, 0.5f, 0.5f}, {-0.5f, 0.5f, 0.5f}
        };
        std::vector<GLVertex> vertices;
        for (const auto& corner : corners)
        {
            GLVertex vertex;
            vertex.position = corner;
            vertex.normal = glm::normalize(corner);
            vertex.boneIds = {0, -1, -1, -1};
            vertex.boneWeights = {1.0f, 0.0f, 0.0f, 0.0f};
            vertices.push_back(vertex);
        }
        GLMeshMaterial material;
        material.diffuseColor = {0.7f, 0.25f, 0.1f};
        material.specularStrength = 0.0f;
        material.rimStrength = 0.0f;
        GLModel cube;
        cube.addMesh(GLMesh{std::move(vertices), {
            4, 5, 6, 4, 6, 7, 1, 0, 3, 1, 3, 2,
            0, 4, 7, 0, 7, 3, 5, 1, 2, 5, 2, 6,
            3, 7, 6, 3, 6, 2, 0, 1, 5, 0, 5, 4}, material});
        GLModelNode node;
        node.name = "reflection";
        GLBoneInfo bone;
        bone.name = node.name;
        bone.nodeIndex = cube.addNode(std::move(node));
        cube.addBone(std::move(bone));
        const glm::mat4 identity{1.0f};
        const glm::mat4 reflection = glm::scale(identity, {-1.0f, 1.0f, 1.0f});
        renderer.beginFrame();
        renderer.renderModel(cube, reflection);
        const auto modelReflection = target.pixels();
        renderer.beginFrame();
        const std::array<glm::mat4, 1> palette{reflection};
        renderer.renderModel(cube, identity, palette);
        const auto skinReflection = target.pixels();
        std::size_t differences = 0;
        std::size_t colored = 0;
        for (std::size_t i = 0; i < modelReflection.size(); ++i)
            if (std::abs(static_cast<int>(modelReflection[i]) - static_cast<int>(skinReflection[i])) > 2)
                ++differences;
        for (std::size_t i = 0; i < modelReflection.size(); i += 4)
            if (modelReflection[i] != modelReflection[i + 1] || modelReflection[i + 1] != modelReflection[i + 2])
                ++colored;
        // Identical world geometry/normals must have identical two-sided light
        // and inverted-hull outlines, regardless of which transform reflected it.
        if (differences != 0) std::cerr << "Reflected-skin differing channels: " << differences << '\n';
        JULIA_CHECK(colored > 100);
        JULIA_CHECK(differences == 0);
    }

    void testRendererValidationAndState(GLShader& previousShader)
    {
        using namespace julia::render_opengl;
        PixelTarget target{true};
        OpenGLRenderer renderer;
        renderer.initialize(64, 64);
        CameraConfig camera;
        camera.position = {0.0f, 0.0f, 3.0f};
        camera.target = {0.0f, 0.0f, 0.0f};
        renderer.setCamera(camera);
        const auto invalidCamera = [&](const CameraConfig& invalid) {
            expectException([&] { renderer.setCamera(invalid); });
            renderer.setCamera(camera);
        };
        auto invalid = camera;
        invalid.position = invalid.target;
        invalidCamera(invalid);
        invalid = camera;
        invalid.up = {0.0f, 0.0f, 1.0f};
        invalidCamera(invalid);
        invalid = camera;
        invalid.fovDegrees = 180.0f;
        invalidCamera(invalid);
        invalid = camera;
        invalid.fovDegrees = std::numeric_limits<float>::denorm_min();
        invalidCamera(invalid);
        invalid = camera;
        invalid.nearPlane = 0.0f;
        invalidCamera(invalid);
        invalid = camera;
        invalid.farPlane = invalid.nearPlane;
        invalidCamera(invalid);
        invalid = camera;
        invalid.position.x = std::numeric_limits<float>::quiet_NaN();
        invalidCamera(invalid);
        // These finite planes overflow the intermediate float far+near and
        // far*near expressions, while their final projection coefficients fit.
        auto largeCamera = camera;
        largeCamera.nearPlane = 1.0e38f;
        largeCamera.farPlane = 2.8e38f;
        renderer.setCamera(largeCamera);
        renderer.resize(64, 64);
        renderer.setCamera(camera);

        auto model = triangleModel();
        const glm::mat4 identity{1.0f};
        expectException([&] { renderer.renderModel(model, glm::scale(identity, {1.0f, 0.0f, 1.0f})); });
        glm::mat4 invalidMatrix{1.0f};
        invalidMatrix[0][0] = std::numeric_limits<float>::infinity();
        expectException([&] { renderer.renderModel(model, invalidMatrix); });
        auto rigged = triangleModel(2);
        const std::array<glm::mat4, 1> incompletePalette{identity};
        expectException([&] { renderer.renderModel(rigged, identity, incompletePalette); });
        const std::array<glm::mat4, 2> invalidPalette{identity, invalidMatrix};
        expectException([&] { renderer.renderModel(rigged, identity, invalidPalette); });

        target.clear({0.9f, 0.1f, 0.6f, 0.4f});
        glEnable(GL_SCISSOR_TEST);
        glScissor(0, 0, 1, 1);
        glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
        glDepthMask(GL_FALSE);
        renderer.beginFrame();
        checkPixel(target.read(48, 48), glm::vec4{128.0f / 255.0f, 128.0f / 255.0f, 128.0f / 255.0f, 1.0f});
        glDisable(GL_SCISSOR_TEST);
        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

        GLuint previousVao = 0;
        glGenVertexArrays(1, &previousVao);
        glBindVertexArray(previousVao);
        previousShader.bind();
        glActiveTexture(GL_TEXTURE3);
        glDisable(GL_DEPTH_TEST);
        glDepthFunc(GL_GEQUAL);
        glDepthMask(GL_FALSE);
        glEnable(GL_CULL_FACE);
        glCullFace(GL_FRONT_AND_BACK);
        glFrontFace(GL_CW);
        glEnable(GL_BLEND);
        glBlendEquationSeparate(GL_FUNC_REVERSE_SUBTRACT, GL_FUNC_SUBTRACT);
        glBlendFuncSeparate(GL_DST_COLOR, GL_SRC_COLOR, GL_DST_ALPHA, GL_SRC_ALPHA);
        GLint expectedDrawFramebuffer = 0;
        glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &expectedDrawFramebuffer);
        GLuint readFramebuffer = 0, texture2 = 0, sampler2 = 0;
        glGenFramebuffers(1, &readFramebuffer);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, readFramebuffer);
        glReadBuffer(GL_NONE);
        glGenTextures(1, &texture2);
        glGenSamplers(1, &sampler2);
        glActiveTexture(GL_TEXTURE2);
        glBindTexture(GL_TEXTURE_2D, texture2);
        glBindSampler(2, sampler2);
        glActiveTexture(GL_TEXTURE3);
        glViewport(3, 4, 55, 52);
        glEnable(GL_SCISSOR_TEST);
        glScissor(0, 0, 64, 64);
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(100.0f, 100.0f);
        renderer.renderModel(model, glm::scale(identity, {-1.0f, 1.0f, 1.0f}));
        const auto integer = [](GLenum name) { GLint value = 0; glGetIntegerv(name, &value); return value; };
        JULIA_CHECK(glIsEnabled(GL_DEPTH_TEST) == GL_FALSE);
        JULIA_CHECK(integer(GL_DEPTH_FUNC) == GL_GEQUAL);
        JULIA_CHECK(integer(GL_DEPTH_WRITEMASK) == GL_FALSE);
        JULIA_CHECK(glIsEnabled(GL_CULL_FACE) == GL_TRUE);
        JULIA_CHECK(integer(GL_CULL_FACE_MODE) == GL_FRONT_AND_BACK);
        JULIA_CHECK(integer(GL_FRONT_FACE) == GL_CW);
        JULIA_CHECK(glIsEnabled(GL_BLEND) == GL_TRUE);
        JULIA_CHECK(integer(GL_BLEND_EQUATION_RGB) == GL_FUNC_REVERSE_SUBTRACT);
        JULIA_CHECK(integer(GL_BLEND_EQUATION_ALPHA) == GL_FUNC_SUBTRACT);
        JULIA_CHECK(integer(GL_BLEND_SRC_RGB) == GL_DST_COLOR);
        JULIA_CHECK(integer(GL_BLEND_DST_RGB) == GL_SRC_COLOR);
        JULIA_CHECK(integer(GL_BLEND_SRC_ALPHA) == GL_DST_ALPHA);
        JULIA_CHECK(integer(GL_BLEND_DST_ALPHA) == GL_SRC_ALPHA);
        JULIA_CHECK(integer(GL_CURRENT_PROGRAM) == static_cast<GLint>(previousShader.id()));
        JULIA_CHECK(integer(GL_VERTEX_ARRAY_BINDING) == static_cast<GLint>(previousVao));
        JULIA_CHECK(integer(GL_ACTIVE_TEXTURE) == GL_TEXTURE3);
        JULIA_CHECK(integer(GL_DRAW_FRAMEBUFFER_BINDING) == expectedDrawFramebuffer);
        JULIA_CHECK(integer(GL_READ_FRAMEBUFFER_BINDING) == static_cast<GLint>(readFramebuffer));
        std::array<GLint, 4> viewport{};
        glGetIntegerv(GL_VIEWPORT, viewport.data());
        JULIA_CHECK((viewport == std::array<GLint, 4>{3, 4, 55, 52}));
        JULIA_CHECK(glIsEnabled(GL_SCISSOR_TEST) == GL_TRUE);
        JULIA_CHECK(glIsEnabled(GL_POLYGON_OFFSET_FILL) == GL_TRUE);
        glActiveTexture(GL_TEXTURE2);
        JULIA_CHECK(integer(GL_TEXTURE_BINDING_2D) == static_cast<GLint>(texture2));
        JULIA_CHECK(integer(GL_SAMPLER_BINDING) == static_cast<GLint>(sampler2));
        glBindSampler(2, 0);
        glDeleteSamplers(1, &sampler2);
        glDeleteTextures(1, &texture2);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, static_cast<GLuint>(expectedDrawFramebuffer));
        glDeleteFramebuffers(1, &readFramebuffer);
        glDisable(GL_SCISSOR_TEST);
        glDisable(GL_POLYGON_OFFSET_FILL);
        glViewport(0, 0, 64, 64);
        JULIA_CHECK(glm::length(glm::vec3{target.read()} - glm::vec3{128.0f / 255.0f}) > 0.05f);
        glBindVertexArray(0);
        glDeleteVertexArrays(1, &previousVao);
        glUseProgram(0);
        glDisable(GL_BLEND);
        glDisable(GL_CULL_FACE);
        glFrontFace(GL_CCW);
        glDepthMask(GL_TRUE);
        glDepthFunc(GL_LESS);
        glActiveTexture(GL_TEXTURE0);
    }
}

int main()
{
    glfwSetErrorCallback([](int error, const char* message) {
        std::cerr << "GLFW " << error << ": " << (message ? message : "") << '\n';
    });
    if (!glfwInit()) return 1;
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
    GLFWwindow* window = glfwCreateWindow(64, 64, "JULIA shader pipeline tests", nullptr, nullptr);
    if (!window) { glfwTerminate(); return 1; }
    glfwMakeContextCurrent(window);
    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)))
    {
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }
    try
    {
        const std::filesystem::path shaders = std::filesystem::path{JULIA_TEST_ASSET_DIR} / "shaders";
        const std::string vertex = readText(shaders / "model.vert");
        const std::string fragment = readText(shaders / "model.frag");
        GLShader actual;
        actual.createFromSource(vertex.c_str(), fragment.c_str());
        actual.bind();
        actual.setInt("uShadowMap", 2);
        JULIA_CHECK(actual.valid());
        testShaderLifetime();
        testVertexTransforms(vertex, fragment);
        testToonTransition(actual);
        testFragmentAlpha(actual);
        testRendererValidationAndState(actual);
        testTransparentOrdering();
        testReflectedSkinOrientation();
        JULIA_CHECK(glGetError() == GL_NO_ERROR);
        std::cout << "Shader pipeline verified on " << glGetString(GL_VERSION) << '\n';
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        JULIA_CHECK(false);
    }
    glfwDestroyWindow(window);
    glfwTerminate();
    return JULIA_TEST_RESULT();
}
