#include <glad/glad.h>
#include <GLFW/glfw3.h>
#ifdef _WIN32
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3native.h>
#endif

#include "DisplaySettings.hpp"
#include "DisplayController.hpp"
#include "WindowPlatform.hpp"
#include "julia/render/AvatarCamera.hpp"
#include "julia/render_opengl/AvatarSurface.hpp"
#include "julia/render_opengl/OpenGLRenderer.hpp"
#include "julia_test.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
    using namespace julia::desktop;

    void layoutAndSettings(const std::filesystem::path& output)
    {
        for (const auto area : {WorkArea{0,0,1920,1040,1}, WorkArea{0,48,2560,1392,1.5f},
            WorkArea{-3840,-200,3840,2080,2}, WorkArea{60,0,964,768,1.25f}, WorkArea{0,0,320,200,3}})
        {
            const auto rect = avatarLayout(area, 380, 1.25f);
            JULIA_CHECK(rect.x >= area.x && rect.y >= area.y);
            JULIA_CHECK(rect.x + rect.width <= area.x + area.width);
            JULIA_CHECK(rect.y + rect.height <= area.y + area.height);
            JULIA_CHECK(rect.y + rect.height == area.y + area.height);
            JULIA_CHECK(rect.width > 0 && rect.height > 0);
            JULIA_CHECK(std::abs(static_cast<float>(rect.width) / rect.height - 1.25f) < .015f);
        }
        const auto normal = avatarLayout({0,0,1920,1040,1}, 380, 1);
        const auto scaled = avatarLayout({0,0,3840,2080,2}, 380, 1);
        JULIA_CHECK(scaled.width == normal.width * 2 && scaled.height == normal.height * 2);
        const auto qhd = avatarLayout({0,0,2560,1392,1}, 380, 1);
        JULIA_CHECK(qhd.height == 380 && qhd.y == 1012 && qhd.x == 2168);
        const auto uhd = avatarLayout({0,0,3840,2112,1}, 380, 1);
        JULIA_CHECK(uhd.height > qhd.height && uhd.height <= 475);
        // More width alone must not enlarge a portrait on an ultrawide monitor.
        JULIA_CHECK(avatarLayout({0,0,5120,1392,1}, 380, 1).height == qhd.height);
        JULIA_CHECK(avatarLayout({0,0,2560,1392,1}, 380, 1, 0).x == 2180);
        const auto file = output / "settings.ini";
        DisplaySettings settings;
        settings.mode = DisplayMode::FullWindow;
        settings.topmost = false;
        settings.avatarHeightDip = 460;
        settings.avatarMarginRightDip = 25;
        settings.avatarMonitor = "test-monitor";
        settings.windowX = -1800;
        settings.maximized = true;
        JULIA_CHECK(saveDisplaySettings(file, settings));
        auto loaded = loadDisplaySettings(file);
        JULIA_CHECK(loaded.mode == settings.mode && loaded.topmost == settings.topmost);
        JULIA_CHECK(loaded.avatarHeightDip == 460 && loaded.windowX == -1800 && loaded.maximized);
        JULIA_CHECK(loaded.avatarMarginRightDip == 25 && loaded.avatarMonitor == "test-monitor");
        settings.mode = DisplayMode::DesktopAvatar;
        JULIA_CHECK(saveDisplaySettings(file, settings)); // Replaces an existing file atomically.
        JULIA_CHECK(loadDisplaySettings(file).mode == DisplayMode::DesktopAvatar);
        { std::ofstream invalid(file); invalid << "mode=bad\ntopmost=7\navatar_height_dip=99999\navatar_margin_right_dip=-1\nwindow_width=-2\nwindow_x=123junk\n"; }
        loaded = loadDisplaySettings(file);
        JULIA_CHECK(loaded.mode == DisplayMode::DesktopAvatar && loaded.topmost);
        JULIA_CHECK(loaded.avatarHeightDip == 380 && loaded.windowWidth == 1280 && loaded.windowX == 80);
        JULIA_CHECK(loaded.avatarMarginRightDip == 12 && loaded.avatarMonitor.empty());
    }

    void layoutMatrix()
    {
        int cases = 0;
        // Physical resolutions, DPI, taskbars at every edge/auto-hide, negative origins,
        // portrait/model aspect ratios, user sizes, fractional rounding, and tiny desktops.
        for (const auto size : {glm::ivec2{1920,1080}, {2560,1440}, {3840,2160}, {1920,1200},
            {3440,1440}, {5120,1440}, {1024,768}, {900,1600}, {321,201}, {1,1}})
            for (float dpi : {1.0f,1.25f,1.5f,1.75f,2.0f,2.25f,3.0f})
                for (int edge = 0; edge < 5; ++edge)
                    for (const auto origin : {glm::ivec2{0,0}, {-3840,-200}, {2560,1440}})
                        for (float aspect : {.25f,.65f,1.0f,1.25f,2.0f,4.0f})
                            for (float requested : {180.0f,380.0f,720.0f})
                            {
                                const int taskbar = std::min(79, (std::min(size.x,size.y) - 1) / 4);
                                WorkArea area{origin.x, origin.y, size.x, size.y, dpi};
                                if (edge == 1) area.height -= taskbar;
                                if (edge == 2) { area.y += taskbar; area.height -= taskbar; }
                                if (edge == 3) { area.x += taskbar; area.width -= taskbar; }
                                if (edge == 4) area.width -= taskbar;
                                const auto rect = avatarLayout(area, requested, aspect);
                                JULIA_CHECK(rect.y + rect.height == area.y + area.height);
                                JULIA_CHECK(rect.x >= area.x && rect.y >= area.y);
                                JULIA_CHECK(rect.x + rect.width <= area.x + area.width);
                                JULIA_CHECK(rect.width >= 1 && rect.height >= 1);
                                JULIA_CHECK(rect.height <= std::max(1, (area.height + 1) / 2));
                                if (rect.height >= 8)
                                    JULIA_CHECK(std::abs(rect.width - rect.height * aspect) <= .501f);
                                ++cases;
                            }
        const auto nan = std::numeric_limits<float>::quiet_NaN();
        const auto safe = avatarLayout({0,0,1920,1040,nan}, nan, nan, nan);
        const auto normal = avatarLayout({0,0,1920,1040,1},380,1);
        JULIA_CHECK(safe.width == normal.width && safe.height == normal.height);
        const auto tiny = avatarLayout({-5,-3,0,-1,1}, 380, 4);
        JULIA_CHECK(tiny.x == -5 && tiny.y == -3 && tiny.width == 1 && tiny.height == 1);
        std::cout << "Desktop layout configurations: " << cases << '\n';
    }

    void portrait()
    {
        for (float scale : {.001f, 1.0f, 1000.0f})
        {
            const julia::render::CameraSubject subject{{-scale,3*scale,-.3f*scale}, {scale,5*scale,.3f*scale}, {}};
            const julia::render::AvatarCamera camera{subject, glm::vec3{0,4*scale,0}};
            JULIA_CHECK(std::abs(camera.waistHeight() - 4 * scale) < scale * .001f);
            JULIA_CHECK(camera.target().y > 4 * scale && camera.target().y < 5 * scale);
            JULIA_CHECK(camera.position().z > subject.max.z);
            const auto top = (subject.max.y - camera.target().y) / (camera.frameHeight() * .5f);
            const auto bottom = (4 * scale - camera.target().y) / (camera.frameHeight() * .5f);
            JULIA_CHECK(top < 1 && top > .7f && std::abs(bottom + 1) < .0001f);
            const glm::vec3 offset{scale, scale * .2f, 0};
            JULIA_CHECK(glm::length(camera.target(offset) - camera.target() - offset) < scale * .0001f);
            // Following an animated waist translates the frame, never its scale.
            const auto animatedBottom = camera.target(offset).y - camera.frameHeight() * .5f;
            JULIA_CHECK(std::abs(animatedBottom - (4 * scale + offset.y)) < scale * .001f);
        }
        bool rejected = false;
        try { julia::render::AvatarCamera bad{{{0,0,0},{0,0,0},{}}}; }
        catch (const std::invalid_argument&) { rejected = true; }
        JULIA_CHECK(rejected);
    }

    void nativeLayout(GLFWwindow* window, const std::filesystem::path& output)
    {
        WindowPlatform native{window};
        JULIA_CHECK(native.workArea("disconnected-monitor") == native.primaryWorkArea());
#ifdef _WIN32
        int count = 0;
        auto** monitors = glfwGetMonitors(&count);
        for (int i = 0; i < count; ++i)
        {
            DisplaySettings settings;
            settings.avatarMonitor = glfwGetWin32Adapter(monitors[i]);
            DisplayController display{window, settings, 1.0f, output / "unused.ini", false};
            display.initialize();
            glfwPollEvents();
            const auto area = native.workArea(settings.avatarMonitor);
            const auto rect = avatarLayout(area,380,1);
            const auto check = [&] {
                RECT client{};
                POINT topLeft{};
                JULIA_CHECK(GetClientRect(glfwGetWin32Window(window), &client));
                JULIA_CHECK(ClientToScreen(glfwGetWin32Window(window), &topLeft));
                JULIA_CHECK(topLeft.x == rect.x && topLeft.y == rect.y);
                JULIA_CHECK(client.right == rect.width && client.bottom == rect.height);
                JULIA_CHECK(topLeft.y + client.bottom == area.y + area.height);
                int fw = 0, fh = 0;
                glfwGetFramebufferSize(window,&fw,&fh);
                JULIA_CHECK(fw == rect.width && fh == rect.height);
            };
            check();
            // Repair external placement even when monitor metrics did not change.
            glfwSetWindowPos(window, rect.x - 3, rect.y - 7);
            display.refreshLayout();
            check();
            display.toggleMode();
            glfwSetWindowPos(window, area.x + 32, area.y + 64);
            display.toggleMode();
            JULIA_CHECK(display.settings().avatarMonitor == settings.avatarMonitor);
            check();
            std::cout << "Native monitor " << settings.avatarMonitor << ": work " << area.x << ',' << area.y
                << ' ' << area.width << 'x' << area.height << " DPI scale " << area.scale << '\n';
        }
#else
        (void)output;
#endif
    }

    void alphaAndInput(GLFWwindow* window)
    {
        using namespace julia::render_opengl;
        AvatarSurface surface;
        OpenGLRenderer renderer;
        renderer.initialize(64,64);
        // Known linear premultiplied gray at 50% coverage: the desktop needs
        // sRGB(0.25)*0.5 ~= 68, not sRGB(0.125) ~= 99 (bright edge halo).
        surface.begin(64,64);
        renderer.beginFrame({.125f,.125f,.125f,.5f});
        auto coverage = surface.resolveCoverage();
        JULIA_CHECK(coverage[3] == 128);
        surface.present();
        unsigned char pixel[4]{};
        glReadPixels(32,32,1,1,GL_RGBA,GL_UNSIGNED_BYTE,pixel);
        JULIA_CHECK(std::abs(static_cast<int>(pixel[0]) - 68) <= 1);
        JULIA_CHECK(pixel[0] == pixel[1] && pixel[1] == pixel[2] &&
            std::abs(static_cast<int>(pixel[3]) - 128) <= 1);
        // A zero-alpha source must produce zero RGB as well.
        surface.begin(64,64);
        renderer.beginFrame({0,0,0,0});
        // A moving 16x16 opaque test patch, with an isolated fractional edge.
        glEnable(GL_SCISSOR_TEST);
        glScissor(8,16,16,16);
        glClearColor(1,1,1,1);
        glClear(GL_COLOR_BUFFER_BIT);
        glScissor(24,16,1,16);
        glClearColor(.1f,.1f,.1f,.25f);
        glClear(GL_COLOR_BUFFER_BIT);
        coverage = surface.resolveCoverage();
        WindowPlatform native{window};
#ifdef _WIN32
        JULIA_CHECK(native.supportsAvatar());
        native.updateShape(coverage,64,64);
        HRGN region = CreateRectRgn(0,0,0,0);
        JULIA_CHECK(GetWindowRgn(glfwGetWin32Window(window),region) != ERROR);
        JULIA_CHECK(PtInRegion(region, 8, 47) && PtInRegion(region,24,47));
        JULIA_CHECK(!PtInRegion(region,0,0) && !PtInRegion(region,25,47));
        DeleteObject(region);
#endif
        surface.present();
        glReadPixels(0,0,1,1,GL_RGBA,GL_UNSIGNED_BYTE,pixel);
        JULIA_CHECK(pixel[0] == 0 && pixel[1] == 0 && pixel[2] == 0 && pixel[3] == 0);
        // Rendering happens offscreen: the old native region must not prevent
        // the next animation silhouette from expanding into previously empty pixels.
        surface.begin(64,64);
        renderer.beginFrame({0,0,0,0});
        glEnable(GL_SCISSOR_TEST);
        glScissor(40,40,16,16);
        glClearColor(.25f,.25f,.25f,1);
        glClear(GL_COLOR_BUFFER_BIT);
        coverage = surface.resolveCoverage();
#ifdef _WIN32
        native.updateShape(coverage,64,64);
        region = CreateRectRgn(0,0,0,0);
        JULIA_CHECK(GetWindowRgn(glfwGetWin32Window(window),region) != ERROR);
        JULIA_CHECK(PtInRegion(region,40,23) && !PtInRegion(region,8,47));
        DeleteObject(region);
        // Test native OS targeting without injecting user input.
        const auto area = native.primaryWorkArea();
        glfwSetWindowPos(window,area.x+40,area.y+80);
        glfwShowWindow(window);
        glfwPollEvents();
        POINT inside{40,23}, empty{2,2};
        ClientToScreen(glfwGetWin32Window(window), &inside);
        ClientToScreen(glfwGetWin32Window(window), &empty);
        JULIA_CHECK(WindowFromPoint(inside) == glfwGetWin32Window(window));
        JULIA_CHECK(WindowFromPoint(empty) != glfwGetWin32Window(window));
        glfwHideWindow(window);
#endif
        native.resetShape();
        surface.present();
        glReadPixels(40,40,1,1,GL_RGBA,GL_UNSIGNED_BYTE,pixel);
        JULIA_CHECK(std::abs(static_cast<int>(pixel[0]) - 137) <= 1 && pixel[3] == 255);
        // Odd-sized reallocation and release while the context is current.
        surface.begin(63,51);
        renderer.resize(63,51);
        renderer.beginFrame({0,0,0,0});
        JULIA_CHECK(surface.resolveCoverage().size() == 63u * 51u * 4u);
        surface.present();
        JULIA_CHECK(glGetError() == GL_NO_ERROR);
    }
}

int main(int argc, char** argv)
{
    try
    {
        const auto output = (argc > 1 ? std::filesystem::path{argv[1]} :
            std::filesystem::temp_directory_path() / "Julia/verification") / "display_unit";
        std::filesystem::create_directories(output);
        layoutAndSettings(output);
        layoutMatrix();
        portrait();
        if (!glfwInit()) throw std::runtime_error("GLFW initialization failed");
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,3);
        glfwWindowHint(GLFW_OPENGL_PROFILE,GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_SRGB_CAPABLE,GLFW_TRUE);
        glfwWindowHint(GLFW_TRANSPARENT_FRAMEBUFFER,GLFW_TRUE);
        glfwWindowHint(GLFW_DECORATED,GLFW_FALSE);
        glfwWindowHint(GLFW_FLOATING,GLFW_TRUE);
        glfwWindowHint(GLFW_VISIBLE,GLFW_FALSE);
        glfwWindowHint(GLFW_FOCUS_ON_SHOW,GLFW_FALSE);
        auto* window = glfwCreateWindow(64,64,"Julia display regression",nullptr,nullptr);
        if (!window) throw std::runtime_error("Cannot create test window");
        glfwMakeContextCurrent(window);
        if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)))
            throw std::runtime_error("Cannot load OpenGL");
        alphaAndInput(window);
        nativeLayout(window, output);
        glfwDestroyWindow(window);
        glfwTerminate();
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return JULIA_TEST_RESULT();
}
