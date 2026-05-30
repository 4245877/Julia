#include "julia/render_opengl/GLTexture.hpp"

#include <glad/glad.h>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include <stdexcept>
#include <string>
#include <utility>

namespace julia::render_opengl
{
    namespace
    {
        GLenum sourceFormatForChannels(int channels)
        {
            switch (channels)
            {
                case 1:
                    return GL_RED;
                case 3:
                    return GL_RGB;
                case 4:
                    return GL_RGBA;
                default:
                    throw std::runtime_error(
                        "Unsupported texture channel count: " + std::to_string(channels)
                    );
            }
        }

        GLint internalFormatForChannels(int channels)
        {
            switch (channels)
            {
                case 1:
                    return GL_R8;
                case 3:
                    return GL_RGB8;
                case 4:
                    return GL_RGBA8;
                default:
                    throw std::runtime_error(
                        "Unsupported texture channel count: " + std::to_string(channels)
                    );
            }
        }
    }

    GLTexture::~GLTexture()
    {
        release();
    }

    GLTexture::GLTexture(GLTexture&& other) noexcept
        : texture_(std::exchange(other.texture_, 0)),
          width_(std::exchange(other.width_, 0)),
          height_(std::exchange(other.height_, 0)),
          channels_(std::exchange(other.channels_, 0))
    {
    }

    GLTexture& GLTexture::operator=(GLTexture&& other) noexcept
    {
        if (this != &other)
        {
            release();

            texture_ = std::exchange(other.texture_, 0);
            width_ = std::exchange(other.width_, 0);
            height_ = std::exchange(other.height_, 0);
            channels_ = std::exchange(other.channels_, 0);
        }

        return *this;
    }

    GLTexture GLTexture::loadFromFile(const std::filesystem::path& path)
    {
        if (!std::filesystem::exists(path))
        {
            throw std::runtime_error("Texture file does not exist: " + path.string());
        }

        int width = 0;
        int height = 0;
        int channels = 0;

        // Важно: в GLModelLoader уже используется aiProcess_FlipUVs.
        // Поэтому картинку здесь НЕ переворачиваем, иначе будет двойной flip.
        stbi_set_flip_vertically_on_load(false);

        unsigned char* pixels = stbi_load(
            path.string().c_str(),
            &width,
            &height,
            &channels,
            0
        );

        if (!pixels)
        {
            throw std::runtime_error(
                "Failed to load texture: " + path.string() +
                "\n" +
                stbi_failure_reason()
            );
        }

        GLuint texture = 0;
        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

        glTexImage2D(
            GL_TEXTURE_2D,
            0,
            internalFormatForChannels(channels),
            width,
            height,
            0,
            sourceFormatForChannels(channels),
            GL_UNSIGNED_BYTE,
            pixels
        );

        glGenerateMipmap(GL_TEXTURE_2D);

        glBindTexture(GL_TEXTURE_2D, 0);

        stbi_image_free(pixels);

        GLTexture result;
        result.texture_ = texture;
        result.width_ = width;
        result.height_ = height;
        result.channels_ = channels;

        return result;
    }

    void GLTexture::bind(unsigned int unit) const
    {
        glActiveTexture(GL_TEXTURE0 + unit);
        glBindTexture(GL_TEXTURE_2D, texture_);
    }

    void GLTexture::release()
    {
        if (texture_ != 0)
        {
            glDeleteTextures(1, &texture_);
            texture_ = 0;
        }

        width_ = 0;
        height_ = 0;
        channels_ = 0;
    }
}