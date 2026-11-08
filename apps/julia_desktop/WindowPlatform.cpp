#include "WindowPlatform.hpp"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#ifdef _WIN32
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3native.h>
#endif

#include <algorithm>
#include <cstring>
#include <stdexcept>

namespace julia::desktop
{
    void setAvatarConsoleMode(bool compact)
    {
#ifdef _WIN32
        DWORD processes[2]{};
        if (GetConsoleProcessList(processes, 2) == 1 && processes[0] == GetCurrentProcessId())
            if (const auto console = GetConsoleWindow())
                ShowWindow(console, compact ? SW_HIDE : SW_SHOWNOACTIVATE);
#else
        (void)compact;
#endif
    }

    bool WindowPlatform::supportsAvatar() const
    {
#ifdef _WIN32
        return glfwGetWindowAttrib(window_, GLFW_TRANSPARENT_FRAMEBUFFER) != 0;
#else
        // Other compositors need their own per-pixel input region adapter.
        return false;
#endif
    }

    WorkArea WindowPlatform::primaryWorkArea() const
    {
        auto* monitor = glfwGetPrimaryMonitor();
        if (!monitor) throw std::runtime_error("No primary display available");
        WorkArea area;
        float scaleY = 1;
        glfwGetMonitorWorkarea(monitor, &area.x, &area.y, &area.width, &area.height);
        glfwGetMonitorContentScale(monitor, &area.scale, &scaleY);
        return area;
    }

    void WindowPlatform::resetShape()
    {
#ifdef _WIN32
        if (!SetWindowRgn(glfwGetWin32Window(window_), nullptr, FALSE))
            throw std::runtime_error("Cannot reset desktop avatar input region");
#endif
        previousRuns_.clear();
        shapeWidth_ = shapeHeight_ = 0;
    }

    void WindowPlatform::updateShape(std::span<const unsigned char> pixels, int width, int height)
    {
        if (width <= 0 || height <= 0 || pixels.size() != static_cast<std::size_t>(width) * height * 4)
            throw std::invalid_argument("Invalid avatar coverage dimensions");
#ifdef _WIN32
        // A native region is also an input region across process boundaries.
        // HTTRANSPARENT alone only forwards within the same UI thread. Keep
        // every nonzero alpha pixel, including MSAA and translucent hair edges;
        // DWM continues to composite its original fractional alpha.
        std::vector<int> runs;
        for (int y = 0; y < height; ++y)
        {
            const auto row = static_cast<std::size_t>(height - 1 - y) * width * 4;
            int x = 0;
            while (x < width)
            {
                while (x < width && pixels[row + static_cast<std::size_t>(x) * 4 + 3] == 0) ++x;
                const int start = x;
                while (x < width && pixels[row + static_cast<std::size_t>(x) * 4 + 3] != 0) ++x;
                if (start < x) runs.insert(runs.end(), {start, y, x, y + 1});
            }
        }
        if (shapeWidth_ == width && shapeHeight_ == height && runs == previousRuns_) return;
        const auto count = runs.size() / 4;
        const auto byteSize = sizeof(RGNDATAHEADER) + count * sizeof(RECT);
        // vector<DWORD> guarantees sufficient alignment for RGNDATA/RECT.
        std::vector<DWORD> storage((byteSize + sizeof(DWORD) - 1) / sizeof(DWORD));
        auto* data = reinterpret_cast<RGNDATA*>(storage.data());
        data->rdh.dwSize = sizeof(RGNDATAHEADER);
        data->rdh.iType = RDH_RECTANGLES;
        data->rdh.nCount = static_cast<DWORD>(count);
        data->rdh.nRgnSize = static_cast<DWORD>(count * sizeof(RECT));
        data->rdh.rcBound = {0, 0, width, height};
        auto* rectangles = reinterpret_cast<RECT*>(data->Buffer);
        for (std::size_t i = 0; i < count; ++i)
            rectangles[i] = {runs[i * 4], runs[i * 4 + 1], runs[i * 4 + 2], runs[i * 4 + 3]};
        HRGN region = count ? ExtCreateRegion(nullptr, static_cast<DWORD>(byteSize), data)
                            : CreateRectRgn(0, 0, 0, 0);
        if (!region) throw std::runtime_error("Cannot create desktop avatar input region");
        if (!SetWindowRgn(glfwGetWin32Window(window_), region, FALSE))
        {
            DeleteObject(region);
            throw std::runtime_error("Cannot apply desktop avatar input region");
        }
        // Ownership passes to Windows on success.
        previousRuns_ = std::move(runs);
        shapeWidth_ = width;
        shapeHeight_ = height;
#endif
    }

    DisplayCommand WindowPlatform::contextMenu(bool topmost) const
    {
#ifdef _WIN32
        const HWND handle = glfwGetWin32Window(window_);
        HMENU menu = CreatePopupMenu();
        if (!menu) return DisplayCommand::None;
        AppendMenuW(menu, MF_STRING, 1, L"Full Window\tF10");
        AppendMenuW(menu, MF_STRING | (topmost ? MF_CHECKED : MF_UNCHECKED), 2, L"Always on top\tF9");
        AppendMenuW(menu, MF_STRING, 3, L"Larger");
        AppendMenuW(menu, MF_STRING, 4, L"Smaller");
        AppendMenuW(menu, MF_SEPARATOR, 0, nullptr);
        AppendMenuW(menu, MF_STRING, 5, L"Exit Julia");
        POINT point{};
        GetCursorPos(&point);
        SetForegroundWindow(handle);
        const auto command = TrackPopupMenuEx(menu, TPM_RETURNCMD | TPM_RIGHTBUTTON,
            point.x, point.y, handle, nullptr);
        DestroyMenu(menu);
        PostMessageW(handle, WM_NULL, 0, 0);
        return command >= 1 && command <= 5 ? static_cast<DisplayCommand>(command) : DisplayCommand::None;
#else
        (void)topmost;
        return DisplayCommand::None;
#endif
    }

    bool replaceSettingsFile(const std::filesystem::path& temporary, const std::filesystem::path& target)
    {
#ifdef _WIN32
        return MoveFileExW(temporary.c_str(), target.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) != 0;
#else
        std::error_code error;
        std::filesystem::rename(temporary, target, error);
        return !error;
#endif
    }
}
