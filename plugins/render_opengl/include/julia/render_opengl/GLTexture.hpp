#pragma once

#include <glad/glad.h>

#include <filesystem>

namespace julia::render_opengl
{
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

        void bind(unsigned int unit = 0) const;

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

        void release();
    };
}