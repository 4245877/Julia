#pragma once

#include <filesystem>
#include <string>

namespace julia::desktop
{
    enum class DisplayMode { DesktopAvatar, FullWindow };

    struct DisplaySettings
    {
        DisplayMode mode = DisplayMode::DesktopAvatar;
        bool topmost = true;
        float avatarHeightDip = 380;
        float avatarMarginRightDip = 12;
        // Empty follows the primary monitor; F10 from Full Window selects its monitor.
        std::string avatarMonitor;
        int windowX = 80, windowY = 80, windowWidth = 1280, windowHeight = 900;
        bool maximized = false;
    };

    struct WorkArea
    {
        // Windows/GLFW screen coordinates are physical pixels in this DPI-aware app.
        int x = 0, y = 0, width = 1, height = 1;
        float scale = 1;
        bool operator==(const WorkArea&) const = default;
    };
    struct WindowRect { int x, y, width, height; };

    WindowRect avatarLayout(WorkArea area, float heightDip, float aspect, float marginRightDip = 12);
    DisplaySettings loadDisplaySettings(const std::filesystem::path& path);
    bool saveDisplaySettings(const std::filesystem::path& path, const DisplaySettings& settings);
}
