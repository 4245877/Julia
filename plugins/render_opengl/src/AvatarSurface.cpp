#include "julia/render_opengl/AvatarSurface.hpp"

#include <algorithm>
#include <stdexcept>

namespace julia::render_opengl
{
    AvatarSurface::~AvatarSurface() { release(); }

    void AvatarSurface::release()
    {
        glDeleteFramebuffers(1, &scene_);
        glDeleteFramebuffers(1, &resolved_);
        glDeleteRenderbuffers(1, &color_);
        glDeleteRenderbuffers(1, &depth_);
        glDeleteTextures(1, &texture_);
        glDeleteVertexArrays(1, &vao_);
        scene_ = resolved_ = color_ = depth_ = texture_ = vao_ = 0;
        width_ = height_ = 0;
        coverage_.clear();
    }

    void AvatarSurface::begin(int width, int height)
    {
        if (width <= 0 || height <= 0) throw std::invalid_argument("Invalid avatar surface dimensions");
        if (width != width_ || height != height_)
        {
            release();
            width_ = width;
            height_ = height;
            GLint maximum = 0;
            glGetIntegerv(GL_MAX_SAMPLES, &maximum);
            const int samples = std::min(8, maximum);
            glGenFramebuffers(1, &scene_);
            glBindFramebuffer(GL_FRAMEBUFFER, scene_);
            glGenRenderbuffers(1, &color_);
            glBindRenderbuffer(GL_RENDERBUFFER, color_);
            glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples, GL_RGBA16F, width, height);
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, color_);
            glGenRenderbuffers(1, &depth_);
            glBindRenderbuffer(GL_RENDERBUFFER, depth_);
            glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples, GL_DEPTH_COMPONENT24, width, height);
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_);
            if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
                throw std::runtime_error("Cannot create multisampled avatar surface");
            glGenFramebuffers(1, &resolved_);
            glBindFramebuffer(GL_FRAMEBUFFER, resolved_);
            glGenTextures(1, &texture_);
            glBindTexture(GL_TEXTURE_2D, texture_);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, width, height, 0, GL_RGBA, GL_FLOAT, nullptr);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture_, 0);
            if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
                throw std::runtime_error("Cannot resolve avatar surface");
            glGenVertexArrays(1, &vao_);
            coverage_.resize(static_cast<std::size_t>(width) * height * 4);
        }
        if (!presentation_.valid())
            presentation_.createFromSource(R"(#version 330 core
                void main() {
                    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
                    gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
                })", R"(#version 330 core
                uniform sampler2D scene;
                out vec4 color;
                void main() {
                    vec4 p = texelFetch(scene, ivec2(gl_FragCoord.xy), 0);
                    float a = clamp(p.a, 0.0, 1.0);
                    vec3 linear = a > 0.00001 ? clamp(p.rgb / a, 0.0, 1.0) : vec3(0.0);
                    vec3 encoded = mix(1.055 * pow(linear, vec3(1.0 / 2.4)) - 0.055,
                        linear * 12.92, lessThanEqual(linear, vec3(0.0031308)));
                    color = vec4(encoded * a, a);
                })");
        glBindFramebuffer(GL_FRAMEBUFFER, scene_);
    }

    std::span<const unsigned char> AvatarSurface::resolveCoverage()
    {
        glBindFramebuffer(GL_READ_FRAMEBUFFER, scene_);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, resolved_);
        glDisable(GL_SCISSOR_TEST);
        glBlitFramebuffer(0, 0, width_, height_, 0, 0, width_, height_, GL_COLOR_BUFFER_BIT, GL_NEAREST);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, resolved_);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        // Preserve caller pixel-pack state, including a possible asynchronous
        // readback buffer; a CPU pointer must never become a PBO offset.
        GLint buffer = 0, alignment = 0, row = 0, skipRows = 0, skipPixels = 0;
        glGetIntegerv(GL_PIXEL_PACK_BUFFER_BINDING, &buffer);
        glGetIntegerv(GL_PACK_ALIGNMENT, &alignment);
        glGetIntegerv(GL_PACK_ROW_LENGTH, &row);
        glGetIntegerv(GL_PACK_SKIP_ROWS, &skipRows);
        glGetIntegerv(GL_PACK_SKIP_PIXELS, &skipPixels);
        glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glPixelStorei(GL_PACK_ROW_LENGTH, 0);
        glPixelStorei(GL_PACK_SKIP_ROWS, 0);
        glPixelStorei(GL_PACK_SKIP_PIXELS, 0);
        glReadPixels(0, 0, width_, height_, GL_RGBA, GL_UNSIGNED_BYTE, coverage_.data());
        glBindBuffer(GL_PIXEL_PACK_BUFFER, static_cast<GLuint>(buffer));
        glPixelStorei(GL_PACK_ALIGNMENT, alignment);
        glPixelStorei(GL_PACK_ROW_LENGTH, row);
        glPixelStorei(GL_PACK_SKIP_ROWS, skipRows);
        glPixelStorei(GL_PACK_SKIP_PIXELS, skipPixels);
        return coverage_;
    }

    void AvatarSurface::present()
    {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glViewport(0, 0, width_, height_);
        glDisable(GL_FRAMEBUFFER_SRGB);
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_BLEND);
        glDisable(GL_CULL_FACE);
        glDisable(GL_SCISSOR_TEST);
        glDisable(GL_DITHER);
        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
        presentation_.bind();
        glActiveTexture(GL_TEXTURE0);
        glBindSampler(0, 0);
        glBindTexture(GL_TEXTURE_2D, texture_);
        presentation_.setInt("scene", 0);
        glBindVertexArray(vao_);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glBindVertexArray(0);
        glUseProgram(0);
        glEnable(GL_DITHER);
    }
}
