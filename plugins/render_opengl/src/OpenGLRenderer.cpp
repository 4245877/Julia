#include "julia/render_opengl/OpenGLRenderer.hpp"

#include <glad/glad.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace julia::render_opengl
{
    namespace
    {
        bool finite(const glm::vec3& value)
        {
            return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
        }

        bool finite(const glm::mat4& matrix)
        {
            for (int column = 0; column < 4; ++column)
                for (int row = 0; row < 4; ++row)
                    if (!std::isfinite(matrix[column][row])) return false;
            return true;
        }

        std::pair<glm::mat4, glm::mat4> cameraMatrices(const CameraConfig& camera, int width, int height)
        {
            const glm::mat4 projection{glm::perspective(glm::radians(static_cast<double>(camera.fovDegrees)),
                static_cast<double>(width) / std::max(height, 1),
                static_cast<double>(camera.nearPlane), static_cast<double>(camera.farPlane))};
            const glm::mat4 view{glm::lookAt(glm::dvec3{camera.position},
                glm::dvec3{camera.target}, glm::dvec3{camera.up})};
            if (!finite(projection) || !finite(view))
                throw std::invalid_argument("Camera produces non-finite view/projection matrices");
            return {projection, view};
        }

        void verifyDefaultSrgb()
        {
            // Some WGL drivers report GL_LINEAR for an sRGB-capable default
            // framebuffer. Verify conversion rather than rejecting that query.
            GLint readFramebuffer = 0, readBuffer = 0, packBuffer = 0;
            GLint alignment = 0, rowLength = 0, skipRows = 0, skipPixels = 0;
            GLint scissor[4]{};
            GLboolean mask[4]{};
            GLfloat clear[4]{};
            const auto scissorEnabled = glIsEnabled(GL_SCISSOR_TEST);
            const auto srgbEnabled = glIsEnabled(GL_FRAMEBUFFER_SRGB);
            glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &readFramebuffer);
            glGetIntegerv(GL_PIXEL_PACK_BUFFER_BINDING, &packBuffer);
            glGetIntegerv(GL_PACK_ALIGNMENT, &alignment);
            glGetIntegerv(GL_PACK_ROW_LENGTH, &rowLength);
            glGetIntegerv(GL_PACK_SKIP_ROWS, &skipRows);
            glGetIntegerv(GL_PACK_SKIP_PIXELS, &skipPixels);
            glGetIntegerv(GL_SCISSOR_BOX, scissor);
            glGetBooleanv(GL_COLOR_WRITEMASK, mask);
            glGetFloatv(GL_COLOR_CLEAR_VALUE, clear);
            glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
            glGetIntegerv(GL_READ_BUFFER, &readBuffer);
            GLint drawBuffer = 0;
            glGetIntegerv(GL_DRAW_BUFFER0, &drawBuffer);
            glReadBuffer(drawBuffer == GL_FRONT_AND_BACK ? GL_FRONT_LEFT : static_cast<GLenum>(drawBuffer));
            glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glPixelStorei(GL_PACK_ROW_LENGTH, 0);
            glPixelStorei(GL_PACK_SKIP_ROWS, 0);
            glPixelStorei(GL_PACK_SKIP_PIXELS, 0);
            glEnable(GL_SCISSOR_TEST);
            glScissor(0, 0, 1, 1);
            glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
            glEnable(GL_FRAMEBUFFER_SRGB);
            glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);
            unsigned char pixel[4]{};
            glReadPixels(0, 0, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel);
            glReadBuffer(static_cast<GLenum>(readBuffer));
            glBindFramebuffer(GL_READ_FRAMEBUFFER, static_cast<GLuint>(readFramebuffer));
            glBindBuffer(GL_PIXEL_PACK_BUFFER, static_cast<GLuint>(packBuffer));
            glPixelStorei(GL_PACK_ALIGNMENT, alignment);
            glPixelStorei(GL_PACK_ROW_LENGTH, rowLength);
            glPixelStorei(GL_PACK_SKIP_ROWS, skipRows);
            glPixelStorei(GL_PACK_SKIP_PIXELS, skipPixels);
            glScissor(scissor[0], scissor[1], scissor[2], scissor[3]);
            if (!scissorEnabled) glDisable(GL_SCISSOR_TEST);
            if (!srgbEnabled) glDisable(GL_FRAMEBUFFER_SRGB);
            glColorMask(mask[0], mask[1], mask[2], mask[3]);
            glClearColor(clear[0], clear[1], clear[2], clear[3]);
            for (int channel = 0; channel < 3; ++channel)
                if (std::abs(static_cast<int>(pixel[channel]) - 188) > 2)
                    throw std::runtime_error("Default framebuffer does not perform sRGB conversion; request GLFW_SRGB_CAPABLE");
        }

        void validateAffine(const glm::mat4& value)
        {
            for (int column = 0; column < 4; ++column)
                for (int row = 0; row < 4; ++row)
                    if (!std::isfinite(value[column][row]))
                        throw std::invalid_argument("Transform contains non-finite values");
            if (std::abs(value[0][3]) > 1e-6f || std::abs(value[1][3]) > 1e-6f ||
                std::abs(value[2][3]) > 1e-6f || std::abs(value[3][3] - 1.0f) > 1e-6f)
                throw std::invalid_argument("Model and bone transforms must be affine");
        }

        struct ShadowFit
        {
            glm::mat4 matrix{1.0f};
            glm::vec3 parameters{0.0f};
            bool valid = false;
        };

        ShadowFit fitShadow(const Bounds& bounds, const glm::mat4& model,
                            std::span<const glm::mat4> palette, const glm::vec3& light,
                            int resolution)
        {
            if (!bounds.valid) return {};
            const glm::dvec3 direction{light};
            const glm::dvec3 up = std::abs(direction.y) > 0.95
                ? glm::dvec3{1.0, 0.0, 0.0} : glm::dvec3{0.0, 1.0, 0.0};
            // Rotation only: projection fitting never depends on the camera.
            const glm::dmat4 orientation = glm::lookAt(glm::dvec3{0.0}, -direction, up);
            glm::dvec3 low{std::numeric_limits<double>::max()};
            glm::dvec3 high{std::numeric_limits<double>::lowest()};
            const auto include = [&](const glm::dmat4& skin) {
                const auto transform = orientation * glm::dmat4{model} * skin;
                for (int corner = 0; corner < 8; ++corner)
                {
                    const glm::dvec4 point{(corner & 1) ? bounds.max.x : bounds.min.x,
                        (corner & 2) ? bounds.max.y : bounds.min.y,
                        (corner & 4) ? bounds.max.z : bounds.min.z, 1.0};
                    const glm::dvec3 position{transform * point};
                    low = glm::min(low, position);
                    high = glm::max(high, position);
                }
            };
            // Convex skin weights keep every deformed point inside this union.
            // Identity also covers vertices with no valid bone influences.
            include(glm::dmat4{1.0});
            for (const auto& bone : palette) include(glm::dmat4{bone});
            const auto span = high - low;
            const double extent = std::max({span.x, span.y, span.z * 0.1, 1e-12}) * 1.03;
            // Quantized scale avoids a continuously breathing texel grid while
            // a pose changes. Center is snapped to texels in world light space.
            const double size = std::exp2(std::ceil(std::log2(extent) * 8.0) / 8.0);
            const double texel = size / resolution;
            auto center = low * 0.5 + high * 0.5;
            center.x = std::floor(center.x / texel + 0.5) * texel;
            center.y = std::floor(center.y / texel + 0.5) * texel;
            const double margin = std::max(size * 0.05, texel * 4.0);
            const double nearPlane = -high.z - margin;
            const double farPlane = -low.z + margin;
            ShadowFit result;
            result.matrix = glm::mat4{glm::ortho(center.x - size * 0.5, center.x + size * 0.5,
                center.y - size * 0.5, center.y + size * 0.5, nearPlane, farPlane) * orientation};
            result.parameters = {1.0f / resolution, static_cast<float>(texel),
                static_cast<float>(1.0 / (farPlane - nearPlane))};
            result.valid = finite(result.matrix) && finite(result.parameters);
            return result;
        }

        // Snapshot once at the renderer boundary, rather than per mesh.
        struct DrawState
        {
            GLint program{}, vao{}, activeTexture{}, texture0{}, palette1{}, shadow2{},
                sampler0{}, sampler1{}, sampler2{}, buffer{}, depthFunc{}, cullMode{}, frontFace{},
                srcRGB{}, dstRGB{}, srcAlpha{}, dstAlpha{}, equationRGB{}, equationAlpha{};
            GLint framebuffer{}, viewport[4]{};
            GLboolean depthMask{}, depthTest{}, cull{}, blend{}, scissor{}, polygonOffset{};

            DrawState()
            {
                glGetIntegerv(GL_CURRENT_PROGRAM, &program);
                glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &vao);
                glGetIntegerv(GL_ACTIVE_TEXTURE, &activeTexture);
                glActiveTexture(GL_TEXTURE0);
                glGetIntegerv(GL_TEXTURE_BINDING_2D, &texture0);
                glGetIntegerv(GL_SAMPLER_BINDING, &sampler0);
                glActiveTexture(GL_TEXTURE1);
                glGetIntegerv(GL_TEXTURE_BINDING_BUFFER, &palette1);
                glGetIntegerv(GL_SAMPLER_BINDING, &sampler1);
                glActiveTexture(GL_TEXTURE2);
                glGetIntegerv(GL_TEXTURE_BINDING_2D, &shadow2);
                glGetIntegerv(GL_SAMPLER_BINDING, &sampler2);
                glGetIntegerv(GL_TEXTURE_BUFFER_BINDING, &buffer);
                glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &framebuffer);
                glGetIntegerv(GL_VIEWPORT, viewport);
                glGetBooleanv(GL_DEPTH_WRITEMASK, &depthMask);
                glGetIntegerv(GL_DEPTH_FUNC, &depthFunc);
                glGetIntegerv(GL_CULL_FACE_MODE, &cullMode);
                glGetIntegerv(GL_FRONT_FACE, &frontFace);
                glGetIntegerv(GL_BLEND_SRC_RGB, &srcRGB);
                glGetIntegerv(GL_BLEND_DST_RGB, &dstRGB);
                glGetIntegerv(GL_BLEND_SRC_ALPHA, &srcAlpha);
                glGetIntegerv(GL_BLEND_DST_ALPHA, &dstAlpha);
                glGetIntegerv(GL_BLEND_EQUATION_RGB, &equationRGB);
                glGetIntegerv(GL_BLEND_EQUATION_ALPHA, &equationAlpha);
                depthTest = glIsEnabled(GL_DEPTH_TEST);
                cull = glIsEnabled(GL_CULL_FACE);
                blend = glIsEnabled(GL_BLEND);
                scissor = glIsEnabled(GL_SCISSOR_TEST);
                polygonOffset = glIsEnabled(GL_POLYGON_OFFSET_FILL);
            }

            ~DrawState()
            {
                glUseProgram(static_cast<GLuint>(program));
                glBindVertexArray(static_cast<GLuint>(vao));
                glBindBuffer(GL_TEXTURE_BUFFER, static_cast<GLuint>(buffer));
                glActiveTexture(GL_TEXTURE0);
                glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(texture0));
                glBindSampler(0, static_cast<GLuint>(sampler0));
                glActiveTexture(GL_TEXTURE1);
                glBindTexture(GL_TEXTURE_BUFFER, static_cast<GLuint>(palette1));
                glBindSampler(1, static_cast<GLuint>(sampler1));
                glActiveTexture(GL_TEXTURE2);
                glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(shadow2));
                glBindSampler(2, static_cast<GLuint>(sampler2));
                glActiveTexture(static_cast<GLenum>(activeTexture));
                glBindFramebuffer(GL_DRAW_FRAMEBUFFER, static_cast<GLuint>(framebuffer));
                glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
                glDepthMask(depthMask);
                glDepthFunc(static_cast<GLenum>(depthFunc));
                glCullFace(static_cast<GLenum>(cullMode));
                glFrontFace(static_cast<GLenum>(frontFace));
                glBlendFuncSeparate(static_cast<GLenum>(srcRGB), static_cast<GLenum>(dstRGB),
                    static_cast<GLenum>(srcAlpha), static_cast<GLenum>(dstAlpha));
                glBlendEquationSeparate(static_cast<GLenum>(equationRGB), static_cast<GLenum>(equationAlpha));
                setEnabled(GL_DEPTH_TEST, depthTest);
                setEnabled(GL_CULL_FACE, cull);
                setEnabled(GL_BLEND, blend);
                setEnabled(GL_SCISSOR_TEST, scissor);
                setEnabled(GL_POLYGON_OFFSET_FILL, polygonOffset);
            }

            static void setEnabled(GLenum cap, GLboolean enabled)
            {
                if (enabled) glEnable(cap); else glDisable(cap);
            }
        };

        std::filesystem::path getAssetDir(const std::filesystem::path& requested)
        {
            if (!requested.empty())
                return requested;
#ifdef JULIA_ASSET_DIR
            const std::filesystem::path assetDir = JULIA_ASSET_DIR;

            if (std::filesystem::exists(assetDir / "shaders"))
            {
                return assetDir;
            }

            throw std::runtime_error(
                "Invalid JULIA_ASSET_DIR: " + assetDir.string() +
                ". Expected directory: shaders"
            );
#else
            throw std::runtime_error(
                "JULIA_ASSET_DIR is not defined. Cannot locate assets directory."
            );
#endif
        }

        std::string readTextFile(const std::filesystem::path& path)
        {
            std::ifstream file(path, std::ios::in);

            if (!file.is_open())
            {
                throw std::runtime_error("Failed to open file: " + path.string());
            }

            std::stringstream buffer;
            buffer << file.rdbuf();

            if (file.bad()) throw std::runtime_error("Failed to read file: " + path.string());
            return buffer.str();
        }

        std::string readShaderFile(const std::filesystem::path& assetDir,
                                  const std::filesystem::path& shaderFileName)
        {
            const std::filesystem::path fullPath =
                assetDir / "shaders" / shaderFileName;

            return readTextFile(fullPath);
        }
    }

    OpenGLRenderer::~OpenGLRenderer()
    {
        if (shadowFramebuffer_) glDeleteFramebuffers(1, &shadowFramebuffer_);
        if (shadowTexture_) glDeleteTextures(1, &shadowTexture_);
        if (boneTexture_) glDeleteTextures(1, &boneTexture_);
        if (boneBuffer_) glDeleteBuffers(1, &boneBuffer_);
    }

    void OpenGLRenderer::initialize(int width, int height,
                                   const std::filesystem::path& assetDirectory)
    {
        if (!glGetString || !glGetString(GL_VERSION) || !GLAD_GL_VERSION_3_3)
            throw std::runtime_error("Renderer requires a current OpenGL 3.3 context and initialized glad");
        GLint framebuffer = 0;
        glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &framebuffer);
        if (framebuffer == 0)
        {
            GLint encoding = 0;
            GLboolean doubleBuffered = GL_FALSE;
            glGetBooleanv(GL_DOUBLEBUFFER, &doubleBuffered);
            glGetFramebufferAttachmentParameteriv(GL_DRAW_FRAMEBUFFER,
                doubleBuffered ? GL_BACK_LEFT : GL_FRONT_LEFT,
                GL_FRAMEBUFFER_ATTACHMENT_COLOR_ENCODING, &encoding);
            if (encoding != GL_SRGB) verifyDefaultSrgb();
        }
        width_ = width;
        height_ = height;

        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);

        glEnable(GL_MULTISAMPLE);
        glEnable(GL_DITHER);
        glEnable(GL_FRAMEBUFFER_SRGB);

        // Пока culling лучше не включать:
        // у некоторых FBX бывают неожиданные winding/axis.
        glDisable(GL_CULL_FACE);

        const auto assetDir = getAssetDir(assetDirectory);
        const std::string modelVertexShaderSource =
            readShaderFile(assetDir, "model.vert");

        const std::string modelFragmentShaderSource =
            readShaderFile(assetDir, "model.frag");

        shader_.createFromSource(
            modelVertexShaderSource.c_str(),
            modelFragmentShaderSource.c_str()
        );

        glGetIntegerv(GL_MAX_TEXTURE_BUFFER_SIZE, &maxBoneTexels_);
        GLint maxTextureSize = 0;
        glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maxTextureSize);
        shadowMapSize_ = std::min(ShadowMapSize, maxTextureSize);
        // Treat required inputs as a contract; a misspelled/optimized-away
        // input must fail initialization rather than silently take GL defaults.
        for (const auto& [name, expectedType] : {
             std::pair<const char*, GLenum>{"uModel", GL_FLOAT_MAT4}, {"uView", GL_FLOAT_MAT4},
             {"uProjection", GL_FLOAT_MAT4}, {"uNormalMatrix", GL_FLOAT_MAT3},
             {"uBonePalette", GL_SAMPLER_BUFFER}, {"uBoneCount", GL_INT},
             {"uUseSkinning", GL_BOOL}, {"uOutlinePass", GL_BOOL}, {"uViewport", GL_FLOAT_VEC3},
             {"uOutlineWidthPixels", GL_FLOAT}, {"uDiffuseTexture", GL_SAMPLER_2D},
             {"uDiffuseColor", GL_FLOAT_VEC3}, {"uHasDiffuseTexture", GL_BOOL},
             {"uViewPos", GL_FLOAT_VEC3}, {"uLightResponse", GL_FLOAT},
             {"uSpecularStrength", GL_FLOAT}, {"uRimStrength", GL_FLOAT},
             {"uSpecularPower", GL_FLOAT}, {"uMaterialKind", GL_INT}, {"uOutlineScale", GL_FLOAT},
             {"uKeyLightDirection", GL_FLOAT_VEC3}, {"uKeyLightColor", GL_FLOAT_VEC3},
             {"uShadowTint", GL_FLOAT_VEC3}, {"uShadowPass", GL_BOOL}, {"uReceiveShadows", GL_BOOL},
             {"uShadowMap", GL_SAMPLER_2D_SHADOW}, {"uLightViewProjection", GL_FLOAT_MAT4},
             {"uShadowParams", GL_FLOAT_VEC3}, {"uShadowStrength", GL_FLOAT},
             {"uOpacity", GL_FLOAT}, {"uAlphaMode", GL_INT}, {"uAlphaCutoff", GL_FLOAT}})
        {
            GLuint index = GL_INVALID_INDEX;
            glGetUniformIndices(shader_.id(), 1, &name, &index);
            if (index == GL_INVALID_INDEX || glGetUniformLocation(shader_.id(), name) < 0)
                throw std::runtime_error(std::string("Missing required model shader uniform: ") + name);
            GLint type = 0, size = 0;
            glGetActiveUniformsiv(shader_.id(), 1, &index, GL_UNIFORM_TYPE, &type);
            glGetActiveUniformsiv(shader_.id(), 1, &index, GL_UNIFORM_SIZE, &size);
            if (static_cast<GLenum>(type) != expectedType || size != 1)
                throw std::runtime_error(std::string("Invalid model shader uniform type/size: ") + name);
        }
        for (const auto& [name, location] : {std::pair{"aPosition", 0}, {"aNormal", 1},
             {"aTexCoord", 2}, {"aBoneIds", 3}, {"aBoneWeights", 4}, {"aOutlineNormal", 5}})
            if (glGetAttribLocation(shader_.id(), name) != location)
                throw std::runtime_error(std::string("Invalid vertex attribute: ") + name);

        resize(width_, height_);
    }

    void OpenGLRenderer::initializeShadowMap()
    {
        if (shadowFramebuffer_) return;
        // Caller owns a DrawState guard. Use only the draw FBO binding so a
        // separate caller read framebuffer remains completely untouched.
        GLuint texture = 0, framebuffer = 0;
        glGenTextures(1, &texture);
        glGenFramebuffers(1, &framebuffer);
        try
        {
            if (!texture || !framebuffer) throw std::runtime_error("Cannot allocate self-shadow resources");
            glActiveTexture(GL_TEXTURE2);
            glBindTexture(GL_TEXTURE_2D, texture);
            // A caller pixel-unpack buffer must not turn nullptr into offset 0.
            GLint unpackBuffer = 0;
            glGetIntegerv(GL_PIXEL_UNPACK_BUFFER_BINDING, &unpackBuffer);
            glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, shadowMapSize_, shadowMapSize_,
                0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, nullptr);
            glBindBuffer(GL_PIXEL_UNPACK_BUFFER, static_cast<GLuint>(unpackBuffer));
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
            constexpr GLfloat border[]{1.0f, 1.0f, 1.0f, 1.0f};
            glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border);
            glBindFramebuffer(GL_DRAW_FRAMEBUFFER, framebuffer);
            glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, texture, 0);
            glDrawBuffer(GL_NONE);
            GLint readFramebuffer = 0;
            glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &readFramebuffer);
            glBindFramebuffer(GL_READ_FRAMEBUFFER, framebuffer);
            glReadBuffer(GL_NONE);
            glBindFramebuffer(GL_READ_FRAMEBUFFER, static_cast<GLuint>(readFramebuffer));
            if (glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
                throw std::runtime_error("Self-shadow framebuffer is incomplete");
        }
        catch (...)
        {
            if (framebuffer) glDeleteFramebuffers(1, &framebuffer);
            if (texture) glDeleteTextures(1, &texture);
            throw;
        }
        shadowTexture_ = texture;
        shadowFramebuffer_ = framebuffer;
    }

    void OpenGLRenderer::setStyleConfig(const ToonStyleConfig& style)
    {
        const glm::dvec3 direction{style.keyLightDirection};
        const double length = glm::length(direction);
        if (!finite(style.keyLightDirection) || !finite(style.keyLightColor) || !finite(style.shadowTint) ||
            !std::isfinite(length) || length < 1e-12 ||
            glm::any(glm::lessThan(style.keyLightColor, glm::vec3{0.0f})) ||
            glm::any(glm::greaterThan(style.keyLightColor, glm::vec3{4.0f})) ||
            glm::any(glm::lessThan(style.shadowTint, glm::vec3{0.0f})) ||
            glm::any(glm::greaterThan(style.shadowTint, glm::vec3{1.0f})) ||
            !std::isfinite(style.outlineWidthPixels) || style.outlineWidthPixels < 0.0f || style.outlineWidthPixels > 8.0f ||
            !std::isfinite(style.shadowStrength) || style.shadowStrength < 0.0f || style.shadowStrength > 1.0f)
            throw std::invalid_argument("Invalid toon lighting, shadow or outline parameters");
        style_ = style;
        style_.keyLightDirection = glm::vec3{direction / length};
    }

    void OpenGLRenderer::setCamera(const CameraConfig& camera)
    {
        const glm::vec3 forward = camera.target - camera.position;
        if (!finite(camera.position) || !finite(camera.target) || !finite(camera.up) ||
            !finite(forward) || glm::dot(forward, forward) < 1e-12f ||
            glm::dot(camera.up, camera.up) < 1e-12f ||
            glm::length(glm::cross(glm::normalize(forward), glm::normalize(camera.up))) < 1e-6f ||
            !std::isfinite(camera.fovDegrees) || camera.fovDegrees <= 0.0f || camera.fovDegrees >= 180.0f ||
            !std::isfinite(camera.nearPlane) || !std::isfinite(camera.farPlane) ||
            camera.nearPlane <= 0.0f || camera.farPlane <= camera.nearPlane)
            throw std::invalid_argument("Invalid camera: require finite nondegenerate view and 0 < near < far, 0 < FOV < 180");
        const auto [projection, view] = cameraMatrices(camera, width_, height_);
        camera_ = camera;
        projection_ = projection;
        view_ = view;
    }

    void OpenGLRenderer::resize(int width, int height)
    {
        width_ = std::max(width, 1);
        height_ = std::max(height, 1);

        glViewport(0, 0, width_, height_);

        updateCamera();
    }

    void OpenGLRenderer::beginFrame()
    {
        glViewport(0, 0, width_, height_);
        glDepthMask(GL_TRUE);
        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
        glDisable(GL_SCISSOR_TEST);
        glEnable(GL_FRAMEBUFFER_SRGB);
        glClearDepth(1.0);
        // Linear 0.21586 maps to neutral sRGB #808080. Clearing a buffer needs
        // no geometry, texture, shader, camera transform, or visual effects.
        glClearColor(0.21586f, 0.21586f, 0.21586f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

    void OpenGLRenderer::renderModel(
        const GLModel& model,
        const glm::mat4& modelMatrix,
        std::span<const glm::mat4> boneMatrices
    )
    {
        if (!shader_.valid()) throw std::logic_error("Renderer is not initialized");
        validateAffine(modelMatrix);
        const glm::dmat3 doubleModel{modelMatrix};
        const double determinant = glm::determinant(doubleModel);
        const double scaleProduct = glm::length(doubleModel[0]) * glm::length(doubleModel[1]) * glm::length(doubleModel[2]);
        if (!std::isfinite(determinant) || !std::isfinite(scaleProduct) || scaleProduct == 0.0f ||
            std::abs(determinant) <= scaleProduct * 1e-7f)
            throw std::invalid_argument("Model transform is singular");
        const glm::mat3 normalMatrix{glm::transpose(glm::inverse(doubleModel))};
        if (!finite(glm::mat4{normalMatrix}))
            throw std::invalid_argument("Model normal transform exceeds float range");
        if (!boneMatrices.empty() && boneMatrices.size() < model.boneCount())
            throw std::invalid_argument("Bone palette does not cover all model bones");
        if (boneMatrices.size() > static_cast<std::size_t>(maxBoneTexels_ / 4))
            throw std::length_error("Bone palette exceeds GL_MAX_TEXTURE_BUFFER_SIZE");
        for (const auto& matrix : boneMatrices) validateAffine(matrix);

        const DrawState previousState;
        shader_.bind();
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
        glDisable(GL_CULL_FACE);
        glDisable(GL_POLYGON_OFFSET_FILL);
        glFrontFace(determinant < 0.0f ? GL_CW : GL_CCW);
        glBindSampler(0, 0);
        glBindSampler(1, 0);

        shader_.setMat4("uModel", modelMatrix);
        shader_.setMat4("uView", view_);
        shader_.setMat4("uProjection", projection_);
        shader_.setMat3("uNormalMatrix", normalMatrix);
        shader_.setVec3("uViewport", glm::vec3{static_cast<float>(width_), static_cast<float>(height_), 0.0f});
        shader_.setFloat("uOutlineWidthPixels", style_.outlineWidthPixels);
        shader_.setInt("uDiffuseTexture", 0);
        shader_.setInt("uBonePalette", 1);
        shader_.setInt("uShadowMap", 2);
        shader_.setInt("uBoneCount", static_cast<int>(boneMatrices.size()));
        shader_.setVec3("uViewPos", camera_.position);
        shader_.setVec3("uKeyLightDirection", style_.keyLightDirection);
        shader_.setVec3("uKeyLightColor", style_.keyLightColor);
        shader_.setVec3("uShadowTint", style_.shadowTint);
        shader_.setFloat("uShadowStrength", style_.shadowStrength);
        shader_.setBool("uShadowPass", false);
        shader_.setBool("uReceiveShadows", false);

        const bool useSkinning = !boneMatrices.empty();

        shader_.setBool("uUseSkinning", useSkinning);

        glActiveTexture(GL_TEXTURE1);
        if (!boneBuffer_) glGenBuffers(1, &boneBuffer_);
        if (!boneTexture_) glGenTextures(1, &boneTexture_);
        if (!boneBuffer_ || !boneTexture_) throw std::runtime_error("Cannot allocate bone palette objects");
        glBindBuffer(GL_TEXTURE_BUFFER, boneBuffer_);
        glBindTexture(GL_TEXTURE_BUFFER, boneTexture_);
        if (useSkinning || !boneStorageInitialized_)
        {
            const glm::mat4 identity{1.0f};
            const auto bytes = useSkinning ? static_cast<GLsizeiptr>(boneMatrices.size_bytes())
                : static_cast<GLsizeiptr>(sizeof(identity));
            glBufferData(GL_TEXTURE_BUFFER, bytes,
                useSkinning ? boneMatrices.data() : &identity, GL_STREAM_DRAW);
            const GLenum uploadError = glGetError();
            if (uploadError != GL_NO_ERROR)
                throw std::runtime_error("OpenGL bone palette upload failed, error " + std::to_string(uploadError));
            GLint64 allocated = 0;
            glGetBufferParameteri64v(GL_TEXTURE_BUFFER, GL_BUFFER_SIZE, &allocated);
            if (allocated != bytes) throw std::runtime_error("Bone palette storage allocation failed");
            glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, boneBuffer_);
            const GLenum attachmentError = glGetError();
            if (attachmentError != GL_NO_ERROR)
                throw std::runtime_error("OpenGL bone palette attachment failed, error " + std::to_string(attachmentError));
            boneStorageInitialized_ = true;
        }

        const bool hasCasters = std::any_of(model.meshes().begin(), model.meshes().end(),
            [](const GLMesh& mesh) { return !mesh.transparent() && mesh.indexCount() > 0; });
        const auto shadow = style_.selfShadows && style_.shadowStrength > 0.0f && hasCasters
            ? fitShadow(model.bounds(), modelMatrix, boneMatrices,
                        glm::normalize(style_.keyLightDirection), shadowMapSize_) : ShadowFit{};
        if (shadow.valid)
        {
            initializeShadowMap();
            glBindFramebuffer(GL_DRAW_FRAMEBUFFER, shadowFramebuffer_);
            glViewport(0, 0, shadowMapSize_, shadowMapSize_);
            glDisable(GL_SCISSOR_TEST);
            glActiveTexture(GL_TEXTURE2);
            glBindSampler(2, 0);
            // Avoid sampling a texture while rendering into its attachment.
            glBindTexture(GL_TEXTURE_2D, 0);
            constexpr GLfloat depth = 1.0f;
            glClearBufferfv(GL_DEPTH, 0, &depth);
            shader_.setMat4("uLightViewProjection", shadow.matrix);
            shader_.setVec3("uShadowParams", shadow.parameters);
            shader_.setBool("uOutlinePass", false);
            shader_.setBool("uShadowPass", true);
            model.drawOpaque(shader_);
            shader_.setBool("uShadowPass", false);
            glBindFramebuffer(GL_DRAW_FRAMEBUFFER, static_cast<GLuint>(previousState.framebuffer));
            glViewport(previousState.viewport[0], previousState.viewport[1],
                       previousState.viewport[2], previousState.viewport[3]);
            DrawState::setEnabled(GL_SCISSOR_TEST, previousState.scissor);
            glActiveTexture(GL_TEXTURE2);
            glBindTexture(GL_TEXTURE_2D, shadowTexture_);
            shader_.setBool("uReceiveShadows", true);
        }

        // Write opaque/masked depth first, so hidden shells cannot overwrite
        // visible surfaces or one another according to mesh submission order.
        shader_.setBool("uOutlinePass", false);
        model.drawOpaque(shader_);

        shader_.setBool("uOutlinePass", true);
        glEnable(GL_CULL_FACE);
        glCullFace(GL_FRONT);
        // A reflected bone can reverse only some triangles. In that case the
        // fragment shader culls using the skinned primitive's handedness.
        if (useSkinning) glDisable(GL_CULL_FACE);

        glDepthMask(GL_FALSE);

        if (style_.outlineWidthPixels > 0.0f) model.drawOutline(shader_);

        glDepthMask(GL_TRUE);
        glCullFace(GL_BACK);
        glDisable(GL_CULL_FACE);

        shader_.setBool("uOutlinePass", false);
        glEnable(GL_BLEND);
        glBlendEquationSeparate(GL_FUNC_ADD, GL_FUNC_ADD);
        glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
        model.drawTransparent(shader_, view_ * modelMatrix);
    }

    void OpenGLRenderer::updateCamera()
    {
        const auto [projection, view] = cameraMatrices(camera_, width_, height_);
        projection_ = projection;
        view_ = view;
    }

}
