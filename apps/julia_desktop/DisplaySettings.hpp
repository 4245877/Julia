#pragma once

#include <filesystem>

namespace julia::desktop
{
    enum class DisplayMode { DesktopAvatar, FullWindow };

    struct DisplaySettings
    {
        DisplayMode mode = DisplayMode::DesktopAvatar;
        bool topmost = true;
        float avatarHeightDip = 380;
        int windowX = 80, windowY = 80, windowWidth = 1280, windowHeight = 900;
        bool maximized = false;
    };

    struct WorkArea
    {
        int x = 0, y = 0, width = 1, height = 1;
        float scale = 1;
        bool operator==(const WorkArea&) const = default;
    };
    struct WindowRect { int x, y, width, height; };

    WindowRect avatarLayout(WorkArea area, float heightDip, float aspect);
    DisplaySettings loadDisplaySettings(const std::filesystem::path& path);
    bool saveDisplaySettings(const std::filesystem::path& path, const DisplaySettings& settings);
}
