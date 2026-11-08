#pragma once

#include "julia/render_opengl/GLShader.hpp"
#include <span>
#include <vector>

namespace julia::render_opengl
{
    // Context-owned transparent presentation surface. The scene renderer stays
    // shared; only its destination and final compositor encoding change.
    class AvatarSurface
    {
    public:
        AvatarSurface() = default;
        ~AvatarSurface();
        AvatarSurface(const AvatarSurface&) = delete;
        AvatarSurface& operator=(const AvatarSurface&) = delete;
        void begin(int width, int height);
        // Resolves MSAA and reads coverage before any native window clipping.
        // Bottom-up RGBA8; RGB is linear here and used only by diagnostics.
        std::span<const unsigned char> resolveCoverage();
        // Encode straight linear color to sRGB, then premultiply in display
        // space as required by the Windows desktop compositor.
        void present();
        void release();

    private:
        int width_ = 0, height_ = 0;
        GLuint scene_ = 0, color_ = 0, depth_ = 0;
        GLuint resolved_ = 0, texture_ = 0, vao_ = 0;
        GLShader presentation_;
        std::vector<unsigned char> coverage_;
    };
}
