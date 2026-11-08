#include "DisplayController.hpp"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <algorithm>
#include <iostream>

namespace julia::desktop
{
    DisplayController::DisplayController(GLFWwindow* window, DisplaySettings settings, float avatarAspect,
        std::filesystem::path settingsPath, bool persist)
        : window_(window), platform_(window), settings_(settings), aspect_(avatarAspect),
          settingsPath_(std::move(settingsPath)), persist_(persist) {}

    void DisplayController::initialize()
    {
        if (avatar() && !platform_.supportsAvatar())
        {
            std::cerr << "Desktop transparency/input regions unavailable; using Full Window.\n";
            settings_.mode = DisplayMode::FullWindow;
        }
        applyMode();
    }

    void DisplayController::captureWindow()
    {
        if (avatar() || glfwGetWindowAttrib(window_, GLFW_ICONIFIED)) return;
        settings_.maximized = glfwGetWindowAttrib(window_, GLFW_MAXIMIZED) != 0;
        if (!settings_.maximized)
        {
            glfwGetWindowPos(window_, &settings_.windowX, &settings_.windowY);
            glfwGetWindowSize(window_, &settings_.windowWidth, &settings_.windowHeight);
        }
    }

    void DisplayController::applyMode()
    {
        setAvatarConsoleMode(avatar());
        // Clear the old region before resizing: the next FBO render computes
        // the new silhouette before presentation, so it never clips itself.
        platform_.resetShape();
        if (glfwGetWindowAttrib(window_, GLFW_MAXIMIZED)) glfwRestoreWindow(window_);
        glfwSetWindowAttrib(window_, GLFW_DECORATED, avatar() ? GLFW_FALSE : GLFW_TRUE);
        glfwSetWindowAttrib(window_, GLFW_RESIZABLE, avatar() ? GLFW_FALSE : GLFW_TRUE);
        glfwSetWindowAttrib(window_, GLFW_FLOATING, avatar() && settings_.topmost ? GLFW_TRUE : GLFW_FALSE);
        if (avatar())
        {
            refreshLayout();
        }
        else
        {
            glfwSetWindowSize(window_, settings_.windowWidth, settings_.windowHeight);
            // Retain a reachable title bar after a saved monitor disconnect.
            int count = 0;
            auto** monitors = glfwGetMonitors(&count);
            bool visible = false;
            for (int i = 0; i < count; ++i)
            {
                int x = 0, y = 0, w = 0, h = 0;
                glfwGetMonitorWorkarea(monitors[i], &x, &y, &w, &h);
                if (settings_.windowX >= x && settings_.windowY >= y + 32 &&
                    settings_.windowX + 100 < x + w && settings_.windowY < y + h - 40) visible = true;
            }
            if (!visible)
            {
                const auto area = platform_.primaryWorkArea();
                settings_.windowX = area.x + 32;
                settings_.windowY = area.y + 64;
            }
            glfwSetWindowPos(window_, settings_.windowX, settings_.windowY);
            if (settings_.maximized) glfwMaximizeWindow(window_);
        }
    }

    void DisplayController::toggleMode()
    {
        if (!avatar() && !platform_.supportsAvatar())
        {
            std::cerr << "Desktop Avatar requires Windows desktop compositing and a transparent framebuffer.\n";
            return;
        }
        captureWindow();
        if (!avatar()) settings_.avatarMonitor = platform_.windowMonitorId();
        settings_.mode = avatar() ? DisplayMode::FullWindow : DisplayMode::DesktopAvatar;
        applyMode();
        save();
    }

    void DisplayController::execute(DisplayCommand command)
    {
        if (command == DisplayCommand::ToggleMode) toggleMode();
        else if (command == DisplayCommand::ToggleTopmost)
        {
            settings_.topmost = !settings_.topmost;
            if (avatar()) glfwSetWindowAttrib(window_, GLFW_FLOATING, settings_.topmost ? GLFW_TRUE : GLFW_FALSE);
            save();
        }
        else if (command == DisplayCommand::Larger || command == DisplayCommand::Smaller)
        {
            settings_.avatarHeightDip = std::clamp(settings_.avatarHeightDip +
                (command == DisplayCommand::Larger ? 40.0f : -40.0f), 180.0f, 720.0f);
            refreshLayout();
            save();
        }
        else if (command == DisplayCommand::Quit) glfwSetWindowShouldClose(window_, GLFW_TRUE);
    }

    void DisplayController::refreshLayout()
    {
        if (!avatar()) return;
        const auto area = platform_.workArea(settings_.avatarMonitor);
        const auto rect = avatarLayout(area, settings_.avatarHeightDip, aspect_, settings_.avatarMarginRightDip);
        int x = 0, y = 0, width = 0, height = 0;
        glfwGetWindowPos(window_, &x, &y);
        glfwGetWindowSize(window_, &width, &height);
        // Also repair an OS-adjusted rectangle even when work area/DPI is unchanged.
        if (x == rect.x && y == rect.y && width == rect.width && height == rect.height) return;
        platform_.resetShape();
        platform_.applyAvatarRect(rect);
    }

    void DisplayController::updateShape(std::span<const unsigned char> pixels, int width, int height)
    {
        if (avatar()) platform_.updateShape(pixels, width, height);
    }

    void DisplayController::save()
    {
        if (!persist_) return;
        captureWindow();
        if (!saveDisplaySettings(settingsPath_, settings_))
            std::cerr << "Cannot save display preferences: " << settingsPath_.string() << '\n';
    }
}
