#include "julia/model/Image.hpp"
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>

namespace julia::model
{
    Image Image::loadFromFile(const std::filesystem::path& path)
    {
        std::ifstream file(path, std::ios::binary | std::ios::ate);
        if (!file) throw std::runtime_error("Failed to open texture: " + path.string());
        const auto size = file.tellg();
        if (size <= 0 || size > std::numeric_limits<int>::max())
            throw std::runtime_error("Texture file is empty or too large: " + path.string());
        std::vector<unsigned char> data(static_cast<std::size_t>(size));
        file.seekg(0);
        if (!file.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(size)))
            throw std::runtime_error("Failed to read texture: " + path.string());
        try { return decode(data.data(), data.size()); }
        catch (const std::exception& error)
        { throw std::runtime_error("Failed to load texture '" + path.string() + "': " + error.what()); }
    }
    Image Image::decode(const unsigned char* bytes, std::size_t size)
    {
        if (!bytes || size == 0 || size > static_cast<std::size_t>(std::numeric_limits<int>::max()))
            throw std::invalid_argument("Invalid encoded texture buffer size");
        stbi_set_flip_vertically_on_load_thread(false);
        int width = 0, height = 0, channels = 0;
        const std::unique_ptr<unsigned char, void (*)(void*)> pixels(
            stbi_load_from_memory(bytes, static_cast<int>(size), &width, &height, &channels, STBI_rgb_alpha),
            &stbi_image_free);
        if (!pixels)
        {
            const char* reason = stbi_failure_reason();
            throw std::runtime_error(reason ? reason : "Image decoding failed");
        }
        auto result = fromPixels(pixels.get(), width, height, 4);
        result.channels_ = channels;
        return result;
    }
    Image Image::fromPixels(const unsigned char* pixels, int width, int height, int channels)
    {
        if (!pixels || width <= 0 || height <= 0 || channels < 1 || channels > 4)
            throw std::invalid_argument("Image requires positive dimensions and 1..4 channels");
        const auto w = static_cast<std::size_t>(width), h = static_cast<std::size_t>(height);
        if (w > std::numeric_limits<std::size_t>::max() / h / 4)
            throw std::length_error("Image dimensions overflow pixel storage");
        Image result;
        result.width_ = width; result.height_ = height; result.channels_ = channels;
        result.rgba_.resize(w * h * 4);
        for (std::size_t i = 0; i < w * h; ++i)
        {
            const auto* source = pixels + i * static_cast<std::size_t>(channels);
            auto* target = result.rgba_.data() + i * 4;
            target[0] = source[0];
            target[1] = channels <= 2 ? source[0] : source[1];
            target[2] = channels <= 2 ? source[0] : source[2];
            target[3] = channels == 4 ? source[3] : channels == 2 ? source[1] : 255;
            result.transparent_ |= target[3] < 255;
        }
        return result;
    }
}
