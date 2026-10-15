#pragma once

#include <glad/glad.h>

#include <cstddef>
#include <filesystem>

namespace julia::render_opengl
{
    // Owns a GL texture object. Like the other GL wrappers, its destructor calls
    // glDeleteTextures and therefore requires a current GL context. Because
    // instances are handed out as shared_ptr via the loader's texture cache, keep
    // every owner (models, caches) destroyed before the window/context is torn
    // down.
    class GLTexture
    {
    public:
        GLTexture() = default;
        ~GLTexture();

        GLTexture(const GLTexture&) = delete;
        GLTexture& operator=(const GLTexture&) = delete;

        GLTexture(GLTexture&& other) noexcept;
        GLTexture& operator=(GLTexture&& other) noexcept;

        static GLTexture loadFromFile(const std::filesystem::path& path);
        static GLTexture loadFromMemory(const unsigned char* data, std::size_t size);
        // Tightly packed unsigned bytes: gray, gray/alpha, RGB, or RGBA.
        // Diffuse color is decoded from sRGB; alpha always remains linear.
        static GLTexture loadFromPixels(
            const unsigned char* pixels, int width, int height, int channels);

        void bind(unsigned int unit = 0) const;

        bool hasTransparency() const
        {
            return hasTransparency_;
        }

        GLuint id() const
        {
            return texture_;
        }

        bool valid() const
        {
            return texture_ != 0;
        }

    private:
        GLuint texture_ = 0;
        int width_ = 0;
        int height_ = 0;
        int channels_ = 0;
        bool hasTransparency_ = false;

        static GLTexture uploadRgba(const unsigned char* pixels, int width, int height);
        void release();
    };
}
