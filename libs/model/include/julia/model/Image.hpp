#pragma once
#include <cstddef>
#include <filesystem>
#include <span>
#include <vector>

namespace julia::model
{
    // Owned, top-down RGBA8, sRGB color with linear alpha. No importer memory.
    class Image
    {
    public:
        static Image loadFromFile(const std::filesystem::path& path);
        static Image decode(const unsigned char* bytes, std::size_t size);
        static Image fromPixels(const unsigned char* pixels, int width, int height, int channels);
        int width() const { return width_; }
        int height() const { return height_; }
        int channels() const { return channels_; }
        bool hasTransparency() const { return transparent_; }
        std::span<const unsigned char> pixels() const { return rgba_; }
    private:
        int width_ = 0, height_ = 0, channels_ = 0;
        bool transparent_ = false;
        std::vector<unsigned char> rgba_;
    };
}
