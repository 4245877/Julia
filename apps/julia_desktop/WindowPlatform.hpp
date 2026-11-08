#pragma once

#include "DisplaySettings.hpp"
#include <span>
#include <string_view>
#include <vector>

struct GLFWwindow;

namespace julia::desktop
{
    enum class DisplayCommand { None, ToggleMode, ToggleTopmost, Larger, Smaller, Quit };

    // Windows-specific operations live behind this boundary. GLFW owns the
    // native window, its context, DPI awareness, and all input callbacks.
    class WindowPlatform
    {
    public:
        explicit WindowPlatform(GLFWwindow* window) : window_(window) {}
        bool supportsAvatar() const;
        WorkArea primaryWorkArea() const;
        WorkArea workArea(std::string_view monitorId = {}) const;
        std::string windowMonitorId() const;
        void applyAvatarRect(WindowRect rect);
        void resetShape();
        void updateShape(std::span<const unsigned char> bottomUpRgba, int width, int height);
        DisplayCommand contextMenu(bool topmost) const;

    private:
        GLFWwindow* window_;
        std::vector<int> previousRuns_;
        int shapeWidth_ = 0, shapeHeight_ = 0;
    };

    bool replaceSettingsFile(const std::filesystem::path& temporary, const std::filesystem::path& target);
    // A console created solely for this EXE should not accompany the avatar.
    // A user's existing PowerShell/terminal console is never hidden.
    void setAvatarConsoleMode(bool compact);
}
