#include "DisplaySettings.hpp"
#include "WindowPlatform.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <fstream>
#include <string>

namespace julia::desktop
{
    WindowRect avatarLayout(WorkArea area, float heightDip, float aspect)
    {
        const float scale = std::isfinite(area.scale) ? std::clamp(area.scale, 0.5f, 8.0f) : 1.0f;
        aspect = std::isfinite(aspect) ? std::clamp(aspect, 0.25f, 4.0f) : 1.0f;
        heightDip = std::isfinite(heightDip) ? std::clamp(heightDip, 180.0f, 720.0f) : 380.0f;
        const int margin = std::min(static_cast<int>(std::lround(12.0f * scale)),
            std::max(0, std::min(area.width, area.height) / 8));
        const float availableWidth = static_cast<float>(std::max(1, area.width - margin * 2));
        const float availableHeight = static_cast<float>(std::max(1, area.height - margin * 2));
        const float height = std::max(1.0f, std::min({heightDip * scale,
            availableHeight * 0.75f, availableWidth * 0.60f / aspect}));
        const int pixelHeight = std::max(1, static_cast<int>(std::lround(height)));
        const int pixelWidth = std::max(1, static_cast<int>(std::lround(height * aspect)));
        return {area.x + area.width - margin - pixelWidth,
            area.y + area.height - margin - pixelHeight, pixelWidth, pixelHeight};
    }

    DisplaySettings loadDisplaySettings(const std::filesystem::path& path)
    {
        DisplaySettings settings;
        std::ifstream file(path);
        std::string line;
        while (std::getline(file, line))
        {
            if (!line.empty() && line.back() == '\r') line.pop_back();
            const auto separator = line.find('=');
            if (separator == std::string::npos) continue;
            const auto key = line.substr(0, separator);
            const auto value = line.substr(separator + 1);
            if (key == "mode")
            {
                if (value == "window") settings.mode = DisplayMode::FullWindow;
                else if (value == "desktop") settings.mode = DisplayMode::DesktopAvatar;
                continue;
            }
            int number = 0;
            const auto parsed = std::from_chars(value.data(), value.data() + value.size(), number);
            if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size()) continue;
            if (key == "topmost" && (number == 0 || number == 1)) settings.topmost = number != 0;
            else if (key == "avatar_height_dip" && number >= 180 && number <= 720)
                settings.avatarHeightDip = static_cast<float>(number);
            else if (key == "window_x" && number >= -100000 && number <= 100000) settings.windowX = number;
            else if (key == "window_y" && number >= -100000 && number <= 100000) settings.windowY = number;
            else if (key == "window_width" && number >= 320 && number <= 16384) settings.windowWidth = number;
            else if (key == "window_height" && number >= 240 && number <= 16384) settings.windowHeight = number;
            else if (key == "maximized" && (number == 0 || number == 1)) settings.maximized = number != 0;
        }
        return settings;
    }

    bool saveDisplaySettings(const std::filesystem::path& path, const DisplaySettings& settings)
    {
        auto temporary = path;
        temporary += ".tmp";
        {
            std::ofstream file(temporary, std::ios::trunc);
            file << "version=1\nmode=" << (settings.mode == DisplayMode::DesktopAvatar ? "desktop" : "window")
                << "\ntopmost=" << settings.topmost
                << "\navatar_height_dip=" << std::lround(settings.avatarHeightDip)
                << "\nwindow_x=" << settings.windowX << "\nwindow_y=" << settings.windowY
                << "\nwindow_width=" << settings.windowWidth << "\nwindow_height=" << settings.windowHeight
                << "\nmaximized=" << settings.maximized << '\n';
            file.flush();
            if (!file) return false;
        }
        return replaceSettingsFile(temporary, path);
    }
}
