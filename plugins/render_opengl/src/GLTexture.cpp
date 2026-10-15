#include "julia/render_opengl/GLTexture.hpp"

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace julia::render_opengl
{
    namespace
    {
        // Client memory must not inherit another subsystem's PBO or row/skip
        // settings. Restore every changed state even if allocation fails.
        class TextureUploadState
        {
        public:
            TextureUploadState()
            {
                glGetIntegerv(GL_TEXTURE_BINDING_2D, &texture_);
                glGetIntegerv(GL_PIXEL_UNPACK_BUFFER_BINDING, &unpackBuffer_);
                glGetIntegerv(GL_UNPACK_ALIGNMENT, &alignment_);
                glGetIntegerv(GL_UNPACK_ROW_LENGTH, &rowLength_);
                glGetIntegerv(GL_UNPACK_SKIP_ROWS, &skipRows_);
                glGetIntegerv(GL_UNPACK_SKIP_PIXELS, &skipPixels_);
                glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
                glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
                glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
                glPixelStorei(GL_UNPACK_SKIP_ROWS, 0);
                glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);
            }

            ~TextureUploadState()
            {
                glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(texture_));
                glBindBuffer(GL_PIXEL_UNPACK_BUFFER, static_cast<GLuint>(unpackBuffer_));
                glPixelStorei(GL_UNPACK_ALIGNMENT, alignment_);
                glPixelStorei(GL_UNPACK_ROW_LENGTH, rowLength_);
                glPixelStorei(GL_UNPACK_SKIP_ROWS, skipRows_);
                glPixelStorei(GL_UNPACK_SKIP_PIXELS, skipPixels_);
            }

        private:
            GLint texture_ = 0;
            GLint unpackBuffer_ = 0;
            GLint alignment_ = 4;
            GLint rowLength_ = 0;
            GLint skipRows_ = 0;
            GLint skipPixels_ = 0;
        };

        std::size_t checkedPixelCount(int width, int height)
        {
            if (width <= 0 || height <= 0)
                throw std::invalid_argument("Texture dimensions must be positive");
            const auto w = static_cast<std::size_t>(width);
            const auto h = static_cast<std::size_t>(height);
            if (w > std::numeric_limits<std::size_t>::max() / h / 4)
                throw std::length_error("Texture dimensions overflow the pixel buffer size");
            return w * h;
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
          channels_(std::exchange(other.channels_, 0)),
          hasTransparency_(std::exchange(other.hasTransparency_, false))
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
            hasTransparency_ = std::exchange(other.hasTransparency_, false);
        }
        return *this;
    }

    GLTexture GLTexture::loadFromFile(const std::filesystem::path& path)
    {
        // filesystem::path preserves native Unicode filenames on Windows.
        std::ifstream file(path, std::ios::binary | std::ios::ate);
        if (!file)
            throw std::runtime_error("Failed to open texture: " + path.string());
        const auto size = file.tellg();
        if (size <= 0 || size > std::numeric_limits<int>::max())
            throw std::runtime_error("Texture file is empty or too large: " + path.string());
        std::vector<unsigned char> data(static_cast<std::size_t>(size));
        file.seekg(0);
        if (!file.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(size)))
            throw std::runtime_error("Failed to read texture: " + path.string());
        try
        {
            return loadFromMemory(data.data(), data.size());
        }
        catch (const std::exception& error)
        {
            throw std::runtime_error("Failed to load texture '" + path.string() + "': " + error.what());
        }
    }

    GLTexture GLTexture::loadFromMemory(const unsigned char* data, std::size_t size)
    {
        if (!data || size == 0 || size > static_cast<std::size_t>(std::numeric_limits<int>::max()))
            throw std::invalid_argument("Invalid encoded texture buffer size");

        // Assimp already flips UVs. Thread-local stb state avoids changing image
        // loading behavior in other threads and subsystems.
        stbi_set_flip_vertically_on_load_thread(false);
        int width = 0;
        int height = 0;
        int channels = 0;
        const std::unique_ptr<unsigned char, void (*)(void*)> pixels(
            stbi_load_from_memory(data, static_cast<int>(size), &width, &height,
                                  &channels, STBI_rgb_alpha),
            &stbi_image_free);
        if (!pixels)
        {
            const char* reason = stbi_failure_reason();
            throw std::runtime_error(reason ? reason : "Image decoding failed");
        }
        auto texture = uploadRgba(pixels.get(), width, height);
        texture.channels_ = channels;
        return texture;
    }

    GLTexture GLTexture::loadFromPixels(
        const unsigned char* pixels, int width, int height, int channels)
    {
        if (!pixels || channels < 1 || channels > 4)
            throw std::invalid_argument("Texture pixels require 1, 2, 3 or 4 channels");
        const std::size_t count = checkedPixelCount(width, height);
        if (channels == 4)
            return uploadRgba(pixels, width, height);

        // GL_R8/GL_RG8 cannot apply sRGB decoding to grayscale diffuse maps.
        // Sampling them directly would also produce red/red-green instead of gray.
        std::vector<unsigned char> rgba(count * 4);
        const auto stride = static_cast<std::size_t>(channels);
        for (std::size_t i = 0; i < count; ++i)
        {
            const unsigned char* source = pixels + i * stride;
            rgba[i * 4] = source[0];
            rgba[i * 4 + 1] = channels <= 2 ? source[0] : source[1];
            rgba[i * 4 + 2] = channels <= 2 ? source[0] : source[2];
            rgba[i * 4 + 3] = channels == 2 ? source[1] : 255;
        }
        auto texture = uploadRgba(rgba.data(), width, height);
        texture.channels_ = channels;
        return texture;
    }

    GLTexture GLTexture::uploadRgba(const unsigned char* pixels, int width, int height)
    {
        const std::size_t count = checkedPixelCount(width, height);
        GLint maxSize = 0;
        glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maxSize);
        if (width > maxSize || height > maxSize)
            throw std::runtime_error("Texture dimensions exceed GL_MAX_TEXTURE_SIZE");

        TextureUploadState state;
        GLTexture result;
        glGenTextures(1, &result.texture_);
        if (result.texture_ == 0)
            throw std::runtime_error("Failed to allocate OpenGL texture object");
        glBindTexture(GL_TEXTURE_2D, result.texture_);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_SRGB8_ALPHA8, width, height,
                     0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);

        GLint uploadedWidth = 0;
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &uploadedWidth);
        if (uploadedWidth != width)
            throw std::runtime_error("Failed to allocate OpenGL texture storage");
        glGenerateMipmap(GL_TEXTURE_2D);
        GLint mipWidth = 0;
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 1, GL_TEXTURE_WIDTH, &mipWidth);
        if ((width > 1 || height > 1) && mipWidth == 0)
            throw std::runtime_error("Failed to generate OpenGL texture mipmaps");

        result.width_ = width;
        result.height_ = height;
        result.channels_ = 4;
        for (std::size_t i = 0; i < count; ++i)
        {
            if (pixels[i * 4 + 3] < 255)
            {
                result.hasTransparency_ = true;
                break;
            }
        }
        return result;
    }

    void GLTexture::bind(unsigned int unit) const
    {
        GLint maxUnits = 0;
        glGetIntegerv(GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS, &maxUnits);
        if (unit >= static_cast<unsigned int>(maxUnits))
            throw std::out_of_range("Texture unit exceeds GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS");
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
        hasTransparency_ = false;
    }
}
