#pragma once

#include "WindowPlatform.hpp"

namespace julia::desktop
{
    class DisplayController
    {
    public:
        DisplayController(GLFWwindow* window, DisplaySettings settings, float avatarAspect,
            std::filesystem::path settingsPath, bool persist);
        bool avatar() const { return settings_.mode == DisplayMode::DesktopAvatar; }
        const DisplaySettings& settings() const { return settings_; }
        void initialize();
        void toggleMode();
        void execute(DisplayCommand command);
        void refreshLayout();
        void updateShape(std::span<const unsigned char> pixels, int width, int height);
        void save();

    private:
        GLFWwindow* window_;
        WindowPlatform platform_;
        DisplaySettings settings_;
        float aspect_;
        std::filesystem::path settingsPath_;
        bool persist_;
        void applyMode();
        void captureWindow();
    };
}
