#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "julia/render_opengl/GLMesh.hpp"
#include "julia/render_opengl/GLShader.hpp"
#include "julia_test.hpp"

#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace
{
    using namespace julia::render_opengl;

    template<class Exception, class Function>
    void expectThrow(Function&& function)
    {
        bool thrown = false;
        try { function(); }
        catch (const Exception&) { thrown = true; }
        JULIA_CHECK(thrown);
    }

    GLint integer(GLenum name)
    {
        GLint value = 0;
        glGetIntegerv(name, &value);
        return value;
    }

    void checkTexture(const GLTexture& texture, const std::vector<unsigned char>& expected)
    {
        texture.bind(0);
        GLint format = 0;
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_INTERNAL_FORMAT, &format);
        JULIA_CHECK(format == GL_SRGB8_ALPHA8);
        std::vector<unsigned char> pixels(expected.size());
        glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
        JULIA_CHECK(pixels == expected);
        JULIA_CHECK(glGetError() == GL_NO_ERROR);
    }

    void testTextures(const std::filesystem::path& directory)
    {
        const std::array<unsigned char, 4> rgba{100, 50, 25, 255};
        auto existing = GLTexture::loadFromPixels(rgba.data(), 1, 1, 4);
        JULIA_CHECK(!existing.hasTransparency());

        GLuint pbo = 0;
        glGenBuffers(1, &pbo);
        glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo);
        glBufferData(GL_PIXEL_UNPACK_BUFFER, 128, nullptr, GL_STATIC_DRAW);
        existing.bind(3);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 8);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, 13);
        glPixelStorei(GL_UNPACK_SKIP_ROWS, 2);
        glPixelStorei(GL_UNPACK_SKIP_PIXELS, 3);

        const std::array<unsigned char, 4> grayAlpha{128, 255, 64, 0};
        auto texture = GLTexture::loadFromPixels(grayAlpha.data(), 2, 1, 2);
        JULIA_CHECK(texture.hasTransparency());
        JULIA_CHECK(integer(GL_ACTIVE_TEXTURE) == GL_TEXTURE3);
        JULIA_CHECK(integer(GL_TEXTURE_BINDING_2D) == static_cast<GLint>(existing.id()));
        JULIA_CHECK(integer(GL_PIXEL_UNPACK_BUFFER_BINDING) == static_cast<GLint>(pbo));
        JULIA_CHECK(integer(GL_UNPACK_ALIGNMENT) == 8);
        JULIA_CHECK(integer(GL_UNPACK_ROW_LENGTH) == 13);
        JULIA_CHECK(integer(GL_UNPACK_SKIP_ROWS) == 2);
        JULIA_CHECK(integer(GL_UNPACK_SKIP_PIXELS) == 3);
        glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
        glDeleteBuffers(1, &pbo);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
        glPixelStorei(GL_UNPACK_SKIP_ROWS, 0);
        glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);
        const std::vector<unsigned char> expected{128, 128, 128, 255, 64, 64, 64, 0};
        checkTexture(texture, expected);

        const unsigned char encoded[] = {
            0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a, 0x00, 0x00, 0x00, 0x0d,
            0x49, 0x48, 0x44, 0x52, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01,
            0x08, 0x04, 0x00, 0x00, 0x00, 0x5e, 0x2b, 0xb7, 0x01, 0x00, 0x00, 0x00,
            0x0d, 0x49, 0x44, 0x41, 0x54, 0x78, 0x9c, 0x63, 0x68, 0xf8, 0xef, 0xc0,
            0x00, 0x00, 0x05, 0x82, 0x01, 0xc0, 0x2c, 0x00, 0x9f, 0xda, 0x00, 0x00,
            0x00, 0x00, 0x49, 0x45, 0x4e, 0x44, 0xae, 0x42, 0x60, 0x82};
        auto decoded = GLTexture::loadFromMemory(encoded, sizeof(encoded));
        checkTexture(decoded, expected);
        JULIA_CHECK(decoded.hasTransparency());
        std::filesystem::create_directories(directory);
        const auto unicodePath = directory / L"texture-\u0441\u0456\u0440\u0438\u0439-\u30c6\u30b9\u30c8.png";
        {
            std::ofstream file(unicodePath, std::ios::binary);
            file.write(reinterpret_cast<const char*>(encoded), sizeof(encoded));
        }
        checkTexture(GLTexture::loadFromFile(unicodePath), expected);
        std::filesystem::remove(unicodePath);

        const GLuint id = decoded.id();
        GLTexture moved(std::move(decoded));
        JULIA_CHECK(!decoded.valid() && !decoded.hasTransparency());
        JULIA_CHECK(moved.id() == id && moved.hasTransparency());
        decoded = std::move(moved);
        JULIA_CHECK(decoded.id() == id && !moved.valid());
        expectThrow<std::invalid_argument>([] { (void)GLTexture::loadFromMemory(nullptr, 0); });
        expectThrow<std::invalid_argument>([&] { (void)GLTexture::loadFromPixels(rgba.data(), 0, 1, 4); });
        expectThrow<std::invalid_argument>([&] { (void)GLTexture::loadFromPixels(rgba.data(), 1, 1, 5); });
        expectThrow<std::runtime_error>([&] { (void)GLTexture::loadFromMemory(rgba.data(), rgba.size()); });
        expectThrow<std::out_of_range>([&] {
            decoded.bind(static_cast<unsigned int>(integer(GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS)));
        });
        const unsigned char gray = 128;
        checkTexture(GLTexture::loadFromPixels(&gray, 1, 1, 1), {128, 128, 128, 255});
        const std::array<unsigned char, 3> rgb{25, 50, 100};
        checkTexture(GLTexture::loadFromPixels(rgb.data(), 1, 1, 3), {25, 50, 100, 255});
        JULIA_CHECK(glGetError() == GL_NO_ERROR);
    }

    void testMeshes()
    {
        GLuint vao = 0;
        GLuint buffers[2]{};
        glGenVertexArrays(1, &vao);
        glGenBuffers(2, buffers);
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, buffers[0]);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffers[1]);
        std::vector<GLVertex> vertices(3);
        vertices[0].position = {-1.0f, -1.0f, 0.0f};
        vertices[1].position = {1.0f, -1.0f, 0.0f};
        vertices[2].position = {0.0f, 1.0f, 0.0f};
        {
            GLMesh mesh(vertices, {0, 1, 2}, {});
            JULIA_CHECK(mesh.indexCount() == 3);
            JULIA_CHECK(integer(GL_VERTEX_ARRAY_BINDING) == static_cast<GLint>(vao));
            JULIA_CHECK(integer(GL_ARRAY_BUFFER_BINDING) == static_cast<GLint>(buffers[0]));
            JULIA_CHECK(integer(GL_ELEMENT_ARRAY_BUFFER_BINDING) == static_cast<GLint>(buffers[1]));
            GLMesh moved(std::move(mesh));
            JULIA_CHECK(mesh.indexCount() == 0 && moved.indexCount() == 3);
            GLShader unused;
            mesh.draw(unused); // Moved-from and empty meshes must not issue VAO-zero draws.
            GLMesh{}.draw(unused);
            expectThrow<std::out_of_range>([&] { GLMesh invalid(vertices, {0, 1, 3}, {}); });
            expectThrow<std::invalid_argument>([&] { GLMesh invalid(vertices, {0, 1}, {}); });
            auto malformed = vertices;
            malformed[0].position.x = std::numeric_limits<float>::quiet_NaN();
            expectThrow<std::invalid_argument>([&] { GLMesh invalid(malformed, {0, 1, 2}, {}); });
            malformed = vertices;
            malformed[1].boneWeights.x = -0.5f;
            expectThrow<std::invalid_argument>([&] { GLMesh invalid(malformed, {0, 1, 2}, {}); });
            GLMeshMaterial invalidMaterial;
            invalidMaterial.opacity = std::numeric_limits<float>::infinity();
            expectThrow<std::invalid_argument>([&] { GLMesh invalid(vertices, {0, 1, 2}, invalidMaterial); });
            JULIA_CHECK(integer(GL_VERTEX_ARRAY_BINDING) == static_cast<GLint>(vao));
        }
        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glDeleteBuffers(2, buffers);
        glDeleteVertexArrays(1, &vao);
        JULIA_CHECK(glGetError() == GL_NO_ERROR);
    }

    void testMaterialSamplers()
    {
        // Two materials share one image but must use their own sampling state.
        const std::array<unsigned char, 8> pixels{255, 0, 0, 255, 0, 0, 255, 255};
        auto texture = std::make_shared<GLTexture>(GLTexture::loadFromPixels(pixels.data(), 2, 1, 4));
        std::vector<GLVertex> vertices(3);
        vertices[0].position = {-1.0f, -1.0f, 0.0f};
        vertices[1].position = {1.0f, -1.0f, 0.0f};
        vertices[2].position = {0.0f, 1.0f, 0.0f};
        GLMeshMaterial repeat;
        repeat.diffuseTexture = texture;
        repeat.minFilter = GL_NEAREST;
        repeat.magFilter = GL_NEAREST;
        auto clamp = repeat;
        clamp.wrapS = GL_CLAMP_TO_EDGE;
        clamp.wrapT = GL_MIRRORED_REPEAT;

        GLuint previousSampler = 0;
        glGenSamplers(1, &previousSampler);
        glBindSampler(0, previousSampler);
        glActiveTexture(GL_TEXTURE0);
        GLuint repeatSampler = 0;
        GLuint clampSampler = 0;
        GLShader shader;
        shader.createFromSource(R"glsl(#version 330 core
            layout(location=0) in vec3 aPosition;
            void main() { gl_Position = vec4(aPosition, 1.0); }
        )glsl", R"glsl(#version 330 core
            uniform sampler2D uDiffuseTexture;
            out vec4 color;
            void main() { color = texture(uDiffuseTexture, vec2(1.25, 0.5)); }
        )glsl");
        shader.bind();
        glViewport(0, 0, 64, 64);
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_CULL_FACE);
        glDisable(GL_BLEND);
        glDisable(GL_FRAMEBUFFER_SRGB);
        glReadBuffer(GL_BACK);
        const auto draw = [&](const GLMesh& mesh, const std::array<unsigned char, 4>& expected) {
            mesh.draw(shader);
            std::array<unsigned char, 4> actual{};
            glReadPixels(32, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, actual.data());
            JULIA_CHECK(actual == expected);
            return static_cast<GLuint>(integer(GL_SAMPLER_BINDING));
        };
        {
            GLMesh a(vertices, {0, 1, 2}, repeat);
            JULIA_CHECK(integer(GL_SAMPLER_BINDING) == static_cast<GLint>(previousSampler));
            GLMesh b(vertices, {0, 1, 2}, clamp);
            JULIA_CHECK(integer(GL_SAMPLER_BINDING) == static_cast<GLint>(previousSampler));
            repeatSampler = draw(a, {255, 0, 0, 255});
            clampSampler = draw(b, {0, 0, 255, 255});
            JULIA_CHECK(repeatSampler != 0 && clampSampler != 0 && repeatSampler != clampSampler);
            JULIA_CHECK(glIsSampler(repeatSampler) == GL_TRUE);
            GLint value = 0;
            glGetSamplerParameteriv(clampSampler, GL_TEXTURE_WRAP_S, &value);
            JULIA_CHECK(value == GL_CLAMP_TO_EDGE);
            glGetSamplerParameteriv(clampSampler, GL_TEXTURE_WRAP_T, &value);
            JULIA_CHECK(value == GL_MIRRORED_REPEAT);
            glGetSamplerParameteriv(clampSampler, GL_TEXTURE_MIN_FILTER, &value);
            JULIA_CHECK(value == GL_NEAREST);
            glGetSamplerParameteriv(clampSampler, GL_TEXTURE_MAG_FILTER, &value);
            JULIA_CHECK(value == GL_NEAREST);
            std::array<GLfloat, 4> border{};
            glGetSamplerParameterfv(clampSampler, GL_TEXTURE_BORDER_COLOR, border.data());
            JULIA_CHECK((border == std::array<GLfloat, 4>{1, 1, 1, 1}));
            GLMesh moved(std::move(a));
            JULIA_CHECK(draw(moved, {255, 0, 0, 255}) == repeatSampler);
            b = std::move(moved);
            JULIA_CHECK(glIsSampler(clampSampler) == GL_FALSE);
            JULIA_CHECK(draw(b, {255, 0, 0, 255}) == repeatSampler);
            GLShader unused;
            moved.draw(unused);
            a.draw(unused);
            auto invalid = repeat;
            invalid.magFilter = GL_LINEAR_MIPMAP_LINEAR;
            expectThrow<std::invalid_argument>([&] { GLMesh rejected(vertices, {0, 1, 2}, invalid); });
            JULIA_CHECK(integer(GL_SAMPLER_BINDING) == static_cast<GLint>(repeatSampler));
        }
        JULIA_CHECK(glIsSampler(repeatSampler) == GL_FALSE);
        JULIA_CHECK(glIsSampler(clampSampler) == GL_FALSE);
        glBindSampler(0, 0);
        glDeleteSamplers(1, &previousSampler);
        glUseProgram(0);
        JULIA_CHECK(glGetError() == GL_NO_ERROR);
    }
}

int main(int argc, char** argv)
{
    if (!glfwInit()) return 1;
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(64, 64, "GPU resource regression", nullptr, nullptr);
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
        testTextures(argc > 1 ? std::filesystem::path(argv[1]) : "verification");
        testMeshes();
        testMaterialSamplers();
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
