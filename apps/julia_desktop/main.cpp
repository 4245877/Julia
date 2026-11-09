#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "julia/render/OrbitCamera.hpp"
#include "julia/render/AvatarCamera.hpp"
#include "julia/render_opengl/AvatarSurface.hpp"
#include "DisplayController.hpp"
#include "julia/render_opengl/GLModelLoader.hpp"
#include "julia/render_opengl/OpenGLRenderer.hpp"
#include "AnimationVerification.hpp"
#include "EyeVerification.hpp"
#include "CameraInput.hpp"
#include "BodyDiagnostics.hpp"
#include "julia/render_opengl/EyePoseLayer.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <filesystem>
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <system_error>
#include <sstream>
#include <iomanip>
#include <cctype>
#include <fstream>
#include <vector>
#include <memory>

#ifdef _WIN32
#include <windows.h>
#endif

#ifndef JULIA_ASSET_DIR
#define JULIA_ASSET_DIR "assets"
#endif

namespace
{
    std::filesystem::path assetDirectory(const char* executable)
    {
        std::filesystem::path executablePath{executable};
#ifdef _WIN32
        wchar_t buffer[32768]{};
        const DWORD length = GetModuleFileNameW(nullptr, buffer, 32768);
        if (length > 0 && length < 32768)
            executablePath = std::filesystem::path{buffer};
#endif
        const auto bundledAssets = std::filesystem::absolute(executablePath)
            .parent_path() / "assets";
        return std::filesystem::is_directory(bundledAssets)
            ? bundledAssets : std::filesystem::path{JULIA_ASSET_DIR};
    }

    julia::render::CameraSubject cameraSubject(
        const julia::render_opengl::GLModel& model, const glm::mat4& fit)
    {
        julia::render::CameraSubject subject;
        subject.min = glm::vec3{fit * glm::vec4{model.bounds().min, 1}};
        subject.max = glm::vec3{fit * glm::vec4{model.bounds().max, 1}};
        const float height = subject.max.y - subject.min.y;
        std::optional<glm::vec3> head;
        glm::vec3 eyeSum{0};
        int eyeCount = 0;
        for (const auto& node : model.nodes())
        {
            auto name = node.name;
            std::transform(name.begin(), name.end(), name.begin(),
                [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            // Common rig names, with namespace prefixes removed. Unknown rigs
            // use the proportional bounds fallback in OrbitCamera.
            if (const auto colon = name.rfind(':'); colon != std::string::npos)
                name.erase(0, colon + 1);
            const auto point = glm::vec3{fit * node.globalBindTransform * glm::vec4{0,0,0,1}};
            if (point.y < subject.min.y + height * 0.6f ||
                glm::any(glm::lessThan(point, subject.min)) ||
                glm::any(glm::greaterThan(point, subject.max))) continue;
            if (!head && (name == "head" || name.starts_with("head.") || name.starts_with("head_")))
                head = point + glm::vec3{0, height * 0.055f, 0};
            if (name == "eye.l" || name == "eye.r" || name == "lefteye" || name == "righteye" ||
                name == "left_eye" || name == "right_eye")
            {
                eyeSum += point;
                ++eyeCount;
            }
        }
        subject.face = eyeCount == 2 ? std::optional<glm::vec3>{eyeSum * 0.5f} : head;
        return subject;
    }

    struct WindowState
    {
        julia::render_opengl::OpenGLRenderer* renderer = nullptr;
        julia::render::OrbitCamera* orbitCamera = nullptr;
        const julia::render::AvatarCamera* avatarCamera = nullptr;
        julia::desktop::DisplayController* display = nullptr;
        julia::desktop::DisplayCommand displayCommand = julia::desktop::DisplayCommand::None;
        bool contextMenuRequested = false;
        std::optional<std::uint32_t> anchorNode;
        glm::mat4 fitModel{1};
        julia::render_opengl::AnimationPlayer* player = nullptr;
        julia::render_opengl::EyePoseLayer* eyes = nullptr;
        julia::embodiment::BodyInstance* body = nullptr;
        julia::runtime::FixedStepScheduler* scheduler = nullptr;
        const julia::embodiment::BodyDisplay* bodyDisplay = nullptr;
        julia::desktop::BodyDiagnostics* bodyDiagnostics = nullptr;
        const julia::render_opengl::GLModel* model = nullptr;
        julia::render_opengl::CameraConfig cameraConfig{};
        julia::desktop::CameraInput cameraInput;
        julia::desktop::CameraInputContext inputContext;
        double cursorX = 0, cursorY = 0;
        // A future UI routes consumed events here before camera callbacks.
        // The current desktop has only a native title bar and no overlay UI.
        void syncInputContext() { cameraInput.setContext(inputContext); }

        julia::render_opengl::CameraConfig activeCamera() const
        {
            auto updatedConfig = cameraConfig;
            if (display && display->avatar() && avatarCamera)
            {
                glm::vec3 offset{0};
                if (anchorNode && (player || bodyDisplay) && model)
                {
                    const auto bind = model->nodes()[*anchorNode].globalBindTransform;
                    const auto posed = bodyDisplay ? body->resourceRoot(bodyDisplay->root) * bodyDisplay->pose.globals[*anchorNode]
                        : player->globalTransforms()[*anchorNode];
                    offset = glm::vec3{fitModel * (posed * glm::vec4{0,0,0,1} - bind * glm::vec4{0,0,0,1})};
                }
                updatedConfig.position = avatarCamera->position(offset);
                updatedConfig.target = avatarCamera->target(offset);
                updatedConfig.up = {0,1,0};
                updatedConfig.orthographicHeight = avatarCamera->frameHeight();
            }
            else
            {
                updatedConfig.position = orbitCamera->position();
                updatedConfig.target = orbitCamera->target();
                updatedConfig.up = orbitCamera->up();
            }
            return updatedConfig;
        }

        void applyCamera() const
        {
            if (renderer && orbitCamera) renderer->setCamera(activeCamera());
        }
    };

    WindowState* getWindowState(GLFWwindow* window)
    {
        return static_cast<WindowState*>(glfwGetWindowUserPointer(window));
    }

    void glfwErrorCallback(int error, const char* description)
    {
        std::cerr << "GLFW error " << error << ": "
                  << (description ? description : "(no description)") << "\n";
    }

    void APIENTRY glDebugCallback(
        GLenum /*source*/,
        GLenum type,
        GLuint /*id*/,
        GLenum severity,
        GLsizei /*length*/,
        const GLchar* message,
        const void* /*userParam*/
    )
    {
        // Notification-level chatter (buffer hints, etc.) is not actionable.
        if (severity == GL_DEBUG_SEVERITY_NOTIFICATION)
        {
            return;
        }

        std::cerr << "[GL]" << (type == GL_DEBUG_TYPE_ERROR ? " ERROR:" : "")
                  << " " << (message ? message : "") << "\n";
    }

    void framebufferSizeCallback(GLFWwindow* window, int width, int height)
    {
        WindowState* state = getWindowState(window);

        if (state && state->renderer)
        {
            state->renderer->resize(width, height);
            if (state->orbitCamera && (!state->display || !state->display->avatar()))
                state->orbitCamera->setViewport(static_cast<float>(width),
                    static_cast<float>(height), state->cameraConfig.fovDegrees);
            state->cameraInput.cancel();
        }
    }

    void mouseButtonCallback(
        GLFWwindow* window,
        int button,
        int action,
        int mods
    )
    {
        WindowState* state = getWindowState(window);

        if (!state || (button != GLFW_MOUSE_BUTTON_LEFT &&
                       button != GLFW_MOUSE_BUTTON_RIGHT &&
                       button != GLFW_MOUSE_BUTTON_MIDDLE))
        {
            return;
        }

        state->syncInputContext();
        if (state->display && state->display->avatar())
        {
            // Left clicks remain available to character interaction. Native
            // silhouette hit testing has already excluded transparent pixels.
            if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_RELEASE)
                state->contextMenuRequested = true;
            return;
        }
        using Button = julia::desktop::CameraInput::Button;
        const auto cameraButton = button == GLFW_MOUSE_BUTTON_LEFT ? Button::Left :
            button == GLFW_MOUSE_BUTTON_RIGHT ? Button::Right : Button::Middle;
        if (action == GLFW_PRESS)
        {
            state->cameraInput.press(cameraButton, (mods & GLFW_MOD_SHIFT) != 0,
                state->cursorX, state->cursorY);
        }
        else if (action == GLFW_RELEASE) state->cameraInput.release(cameraButton);
    }

    void cursorPositionCallback(GLFWwindow* window, double x, double y)
    {
        WindowState* state = getWindowState(window);

        if (!state || !state->orbitCamera) return;
        state->cursorX = x;
        state->cursorY = y;
        if (state->display && state->display->avatar()) return;
        state->syncInputContext();
        int width = 0, height = 0;
        glfwGetWindowSize(window, &width, &height);
        // Also reject out-of-window positions before GLFW's enter/leave event.
        if (x < 0 || y < 0 || x >= width || y >= height) state->cameraInput.cancel();
        // Cursor deltas and height use logical pixels, including on HiDPI.
        state->cameraInput.move(x, y, static_cast<float>(height), *state->orbitCamera);
    }

    void scrollCallback(
        GLFWwindow* window,
        double /*xOffset*/,
        double yOffset
    )
    {
        WindowState* state = getWindowState(window);

        if (!state || !state->orbitCamera)
        {
            return;
        }

        state->syncInputContext();
        if (state->display && state->display->avatar()) return;
        state->cameraInput.scroll(yOffset, *state->orbitCamera);
    }

    void focusCallback(GLFWwindow* window, int focused)
    {
        if (auto* state = getWindowState(window))
        {
            state->inputContext.focused = focused != 0;
            state->syncInputContext();
        }
    }

    void cursorEnterCallback(GLFWwindow* window, int entered)
    {
        if (auto* state = getWindowState(window))
        {
            state->inputContext.inside = entered != 0;
            if (entered) glfwGetCursorPos(window, &state->cursorX, &state->cursorY);
            state->syncInputContext();
        }
    }

    void keyCallback(GLFWwindow* window, int key, int, int action, int mods)
    {
        if (action != GLFW_PRESS) return;
        auto* state = getWindowState(window);
        if (!state || (!state->player && !state->body)) return;
        state->syncInputContext();
        if (!state->cameraInput.acceptsKeyboard()) return;
        if (key == GLFW_KEY_ESCAPE)
        {
            glfwSetWindowShouldClose(window, GLFW_TRUE);
            return;
        }
        if (key == GLFW_KEY_F10 || key == GLFW_KEY_F9)
        {
            state->displayCommand = key == GLFW_KEY_F10 ? julia::desktop::DisplayCommand::ToggleMode
                : julia::desktop::DisplayCommand::ToggleTopmost;
            state->cameraInput.cancel();
            return;
        }
        if (state->body)
        {
            using namespace julia::embodiment;
            auto& body = *state->body;
            const auto queue = [&](BodyOperation command) { body.enqueue(std::move(command)); };
            if (key == GLFW_KEY_PAUSE)
            { state->scheduler->setPaused(!state->scheduler->paused()); body.collapseHistory(); return; }
            if (key == GLFW_KEY_F6) { queue(ResetBody{}); return; }
            if (state->bodyDiagnostics && state->bodyDiagnostics->key(key,mods)) return;
            if (key == GLFW_KEY_SPACE) queue(ClipCommand{ClipOperation::TogglePlay});
            else if (key == GLFW_KEY_X || key == GLFW_KEY_R)
            { queue(EyeCommand{EyeOperation::Enable,{0,0,0}}); queue(ClipCommand{ClipOperation::Stop}); }
            else if (key == GLFW_KEY_L) queue(ClipCommand{ClipOperation::ToggleLoop});
            else if (key == GLFW_KEY_COMMA || key == GLFW_KEY_PERIOD)
                queue(ClipCommand{ClipOperation::Step,(key == GLFW_KEY_PERIOD ? 1.0 : -1.0)/30});
            else if (key == GLFW_KEY_PAGE_DOWN || key == GLFW_KEY_PAGE_UP)
                queue(ClipCommand{ClipOperation::Next,key == GLFW_KEY_PAGE_DOWN ? 1.0 : -1.0});
            else if (key >= GLFW_KEY_1 && key <= GLFW_KEY_4)
            {
                constexpr std::string_view names[]{"TEST_blink_natural","TEST_blink_independent","TEST_blink_gaze","TEST_blink_head"};
                for (std::size_t i=0;i<body.asset().animations().size();++i)
                    if (body.asset().animations()[i].name.ends_with(names[key-GLFW_KEY_1]))
                    { queue(ClipCommand{ClipOperation::Select,static_cast<double>(i)}); queue(ClipCommand{ClipOperation::Play}); break; }
            }
            else if (key == GLFW_KEY_F1)
            {
                for (std::size_t i=0;i<body.asset().animations().size();++i)
                    std::cout << i << ": " << body.asset().animations()[i].name << '\n';
            }
            else if (key == GLFW_KEY_G) queue(EyeCommand{EyeOperation::Natural});
            else if (key == GLFW_KEY_H) queue(EyeCommand{EyeOperation::AutoBlink});
            else if (key == GLFW_KEY_J) queue(EyeCommand{EyeOperation::AutoGaze});
            else if (key == GLFW_KEY_K || key == GLFW_KEY_U || key == GLFW_KEY_I)
                queue(EyeCommand{EyeOperation::Blink,{key != GLFW_KEY_I ? 1.0f:0,key != GLFW_KEY_U ? 1.0f:0,0}});
            else if (key == GLFW_KEY_HOME) queue(EyeCommand{EyeOperation::Neutral});
            else if (key == GLFW_KEY_T)
            {
                const auto root = state->bodyDisplay ? body.resourceRoot(state->bodyDisplay->root) : glm::mat4{1};
                queue(EyeCommand{EyeOperation::Target,glm::vec3{glm::inverse(state->fitModel*root)*
                    glm::vec4{state->activeCamera().position,1}}});
            }
            else if (key == GLFW_KEY_F8) queue(EyeCommand{EyeOperation::ResetSeed});
            else if (key == GLFW_KEY_LEFT_BRACKET || key == GLFW_KEY_RIGHT_BRACKET)
                queue(EyeCommand{EyeOperation::BlinkInterval,{key == GLFW_KEY_RIGHT_BRACKET ? .5f:-.5f,0,0}});
            else if (key >= GLFW_KEY_KP_1 && key <= GLFW_KEY_KP_9)
            {
                const int direction=key-GLFW_KEY_KP_1;
                queue(EyeCommand{EyeOperation::Gaze,{static_cast<float>(direction%3-1)*8,static_cast<float>(direction/3-1)*5,0}});
            }
            else if ((key == GLFW_KEY_F || key == GLFW_KEY_B) && !state->display->avatar())
            {
                state->cameraInput.cancel();
                if (key == GLFW_KEY_F) state->orbitCamera->focusFace(); else state->orbitCamera->frameBody();
            }
            return;
        }
        // Isolated legacy sampling fixtures below are used only by the existing
        // animation/eye/display verification suites. Interactive input uses BodyInstance.
        auto& player = *state->player;
        if (key == GLFW_KEY_SPACE)
        {
            if (player.state() == julia::render_opengl::PlaybackState::Playing) player.pause();
            else player.play();
        }
        else if (key == GLFW_KEY_X || key == GLFW_KEY_R)
        {
            if (state->eyes) state->eyes->setEnabled(false);
            player.stop();
        }
        else if (key == GLFW_KEY_L) player.setLooping(!player.looping());
        else if (key == GLFW_KEY_COMMA || key == GLFW_KEY_PERIOD)
        {
            player.pause();
            player.seek(player.timeSeconds() + (key == GLFW_KEY_PERIOD ? 1.0 : -1.0) / 30.0);
        }
        else if (key == GLFW_KEY_PAGE_DOWN || key == GLFW_KEY_PAGE_UP)
        {
            const auto count = state->model->animations().size();
            if (count)
            {
                player.selectClip((player.clipIndex() + (key == GLFW_KEY_PAGE_DOWN ? 1 : count - 1)) % count);
                player.play();
            }
        }
        else if (key >= GLFW_KEY_1 && key <= GLFW_KEY_4)
        {
            constexpr std::string_view names[]{"TEST_blink_natural", "TEST_blink_independent",
                "TEST_blink_gaze", "TEST_blink_head"};
            const auto& clips = state->model->animations();
            for (std::size_t i = 0; i < clips.size(); ++i)
                if (clips[i].name.ends_with(names[key - GLFW_KEY_1]))
                {
                    player.selectClip(i);
                    player.play();
                    break;
                }
        }
        else if (key == GLFW_KEY_F1)
        {
            for (std::size_t i = 0; i < state->model->animations().size(); ++i)
                std::cout << i << ": " << state->model->animations()[i].name << " ("
                    << state->model->animations()[i].durationSeconds() << " s)\n";
        }
        else if (state->eyes && key == GLFW_KEY_G)
        {
            auto& eyes = *state->eyes;
            eyes.setEnabled(true);
            eyes.clearTargetPoint();
            eyes.behavior().releaseBlink();
            eyes.behavior().releaseGaze();
            eyes.behavior().setAutoBlink(true);
            eyes.behavior().setAutoGaze(true);
            if (julia::render_opengl::EyePoseLayer::authoredEyes(player.clip())) player.stop();
        }
        else if (state->eyes && (key == GLFW_KEY_H || key == GLFW_KEY_J || key == GLFW_KEY_K ||
            key == GLFW_KEY_U || key == GLFW_KEY_I || key == GLFW_KEY_HOME || key == GLFW_KEY_T ||
            (key >= GLFW_KEY_KP_1 && key <= GLFW_KEY_KP_9) || key == GLFW_KEY_F8 ||
            key == GLFW_KEY_LEFT_BRACKET || key == GLFW_KEY_RIGHT_BRACKET))
        {
            auto& eyes = *state->eyes;
            auto& behavior = eyes.behavior();
            if (key == GLFW_KEY_H) behavior.setAutoBlink(!behavior.autoBlink());
            else if (key == GLFW_KEY_J) behavior.setAutoGaze(!behavior.autoGaze());
            else if (key == GLFW_KEY_K) behavior.triggerBlink();
            else if (key == GLFW_KEY_U || key == GLFW_KEY_I)
                behavior.triggerBlink(key == GLFW_KEY_U, key == GLFW_KEY_I);
            else if (key == GLFW_KEY_HOME) { eyes.clearTargetPoint(); behavior.neutral(); }
            else if (key == GLFW_KEY_T)
            {
                if (eyes.hasTargetPoint()) eyes.clearTargetPoint();
                else eyes.setTargetPoint(glm::vec3{glm::inverse(state->fitModel) *
                    glm::vec4{state->activeCamera().position,1}});
            }
            else if (key == GLFW_KEY_F8) behavior.reset(0x52415048u);
            else if (key == GLFW_KEY_LEFT_BRACKET || key == GLFW_KEY_RIGHT_BRACKET)
            {
                auto settings = behavior.settings();
                settings.blinkMeanSeconds = std::clamp(settings.blinkMeanSeconds +
                    (key == GLFW_KEY_RIGHT_BRACKET ? .5 : -.5), 1.0, 15.0);
                behavior.setSettings(settings);
            }
            else
            {
                const int direction = key - GLFW_KEY_KP_1;
                eyes.clearTargetPoint();
                behavior.setGazeDegrees({static_cast<float>(direction % 3 - 1) * 8,
                    static_cast<float>(direction / 3 - 1) * 5});
            }
            player.update(0);
        }
        else if (key == GLFW_KEY_F || key == GLFW_KEY_B)
        {
            if (state->display && state->display->avatar()) return;
            state->cameraInput.cancel();
            if (key == GLFW_KEY_F) state->orbitCamera->focusFace();
            else state->orbitCamera->frameBody();
        }
    }

    void processInput(GLFWwindow* window, double deltaSeconds)
    {
        auto* state = getWindowState(window);
        if (!state || !state->orbitCamera) return;
        state->syncInputContext();
        if (!state->cameraInput.acceptsKeyboard()) return;
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        if (state->display && state->display->avatar()) return;
        const auto pressed = [window](int key) {
            return glfwGetKey(window, key) == GLFW_PRESS;
        };
        glm::vec2 direction{
            static_cast<float>(pressed(GLFW_KEY_D) || pressed(GLFW_KEY_RIGHT)) -
                static_cast<float>(pressed(GLFW_KEY_A) || pressed(GLFW_KEY_LEFT)),
            static_cast<float>(pressed(GLFW_KEY_E) || pressed(GLFW_KEY_UP)) -
                static_cast<float>(pressed(GLFW_KEY_Q) || pressed(GLFW_KEY_DOWN))
        };
        if (glm::length(direction) > 0.0f)
        {
            const float speed = state->orbitCamera->settings().keyboardPanHalfScreensPerSecond;
            state->orbitCamera->panNormalized(glm::normalize(direction) * speed *
                static_cast<float>(deltaSeconds));
        }
        const float zoom = static_cast<float>(pressed(GLFW_KEY_W)) - static_cast<float>(pressed(GLFW_KEY_S));
        state->orbitCamera->zoom(zoom * state->orbitCamera->settings().keyboardZoomStepsPerSecond *
            static_cast<float>(deltaSeconds));
    }

    void verifyDisplayModes(GLFWwindow* window, julia::render_opengl::AvatarSurface& surface,
        const std::filesystem::path& output)
    {
        auto& state = *getWindowState(window);
        auto& display = *state.display;
        const auto require = [](bool ok, const char* message) {
            if (!ok) throw std::runtime_error(message);
        };
        require(display.avatar(), "Desktop Avatar is not available on this machine");
        state.inputContext = {};
        state.syncInputContext();
        auto cameraBefore = state.orbitCamera->position();
        std::filesystem::create_directories(output);
        std::ofstream metrics(output / "coverage.csv");
        metrics << "frame,width,height,window_bottom,work_bottom,bottom_empty_rows,top_empty_rows,ortho_height\n";
        const auto draw = [&](const std::string& name, bool saveFrame = true) {
            int width = 0, height = 0;
            glfwGetFramebufferSize(window, &width, &height);
            framebufferSizeCallback(window, width, height);
            state.applyCamera();
            if (display.avatar()) surface.begin(width, height);
            else glBindFramebuffer(GL_FRAMEBUFFER, 0);
            state.renderer->beginFrame(display.avatar() ? glm::vec4{0} : glm::vec4{.21586f,.21586f,.21586f,1});
            state.renderer->renderModel(*state.model, state.fitModel, state.player->palette());
            if (display.avatar())
            {
                const auto pixels = surface.resolveCoverage();
                std::size_t empty = 0, opaque = 0, partial = 0;
                for (std::size_t i = 3; i < pixels.size(); i += 4)
                {
                    empty += pixels[i] == 0;
                    opaque += pixels[i] == 255;
                    partial += pixels[i] > 0 && pixels[i] < 255;
                }
                require(empty > pixels.size() / 40 && opaque > 100 && partial > 10,
                    "Avatar silhouette lacks empty/opaque/antialiased pixels");
                // Test the rendered character, not just the window rectangle.
                // OpenGL rows start at the bottom. Ignore barely covered MSAA edges.
                int bottom = height, top = -1;
                for (int row = 0; row < height; ++row)
                    for (int col = 0; col < width; ++col)
                        if (pixels[(static_cast<std::size_t>(row) * width + col) * 4 + 3] >= 128)
                        {
                            bottom = std::min(bottom,row);
                            top = std::max(top,row);
                        }
                julia::desktop::WindowPlatform native{window};
                const auto area = native.workArea(display.settings().avatarMonitor);
                int wx = 0, wy = 0, ww = 0, wh = 0;
                glfwGetWindowPos(window,&wx,&wy);
                glfwGetWindowSize(window,&ww,&wh);
                metrics << name << ',' << width << ',' << height << ',' << wy + wh << ','
                    << area.y + area.height << ',' << bottom << ',' << height - 1 - top << ','
                    << state.activeCamera().orthographicHeight << '\n';
                metrics.flush();
                require(ww == width && wh == height, "Windows framebuffer/client pixel coordinates differ");
                require(wy + wh == area.y + area.height, "Avatar window is detached from work area bottom");
                require(wx >= area.x && wy >= area.y && wx + ww <= area.x + area.width,
                    "Avatar window overlaps reserved system area");
                require(bottom <= 2, "Rendered avatar has transparent rows below the waist");
                require(top < height - 1, "Avatar head/ears exceed portrait headroom");
                display.updateShape(pixels, width, height);
                surface.present();
            }
            std::vector<unsigned char> rgba(static_cast<std::size_t>(width) * height * 4);
            glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
            glReadBuffer(GL_BACK);
            glReadPixels(0,0,width,height,GL_RGBA,GL_UNSIGNED_BYTE,rgba.data());
            if (display.avatar())
            {
                require(rgba[3] == 0 && rgba[rgba.size() - 1] == 0, "Avatar background is not transparent");
                for (std::size_t i = 0; i < rgba.size(); i += 4)
                    require(rgba[i] <= rgba[i + 3] + 1 && rgba[i + 1] <= rgba[i + 3] + 1 &&
                        rgba[i + 2] <= rgba[i + 3] + 1, "Avatar RGB is not display-premultiplied");
            }
            else require(rgba[3] == 255 && std::abs(static_cast<int>(rgba[0]) - 128) <= 1,
                "Full Window grey background changed");
            if (saveFrame)
            {
                std::ofstream file(output / (name + ".pam"), std::ios::binary);
                file << "P7\nWIDTH " << width << "\nHEIGHT " << height << "\nDEPTH 4\nMAXVAL 255\nTUPLTYPE RGB_ALPHA\nENDHDR\n";
                for (int y = height - 1; y >= 0; --y)
                    file.write(reinterpret_cast<const char*>(rgba.data() + static_cast<std::size_t>(y) * width * 4), width * 4);
                require(static_cast<bool>(file), "Cannot save display verification frame");
            }
            require(glGetError() == GL_NO_ERROR, "OpenGL error during display mode verification");
        };
        draw("avatar");
        julia::desktop::WindowPlatform native{window};
        const auto expectedRect = julia::desktop::avatarLayout(native.workArea(display.settings().avatarMonitor),
            display.settings().avatarHeightDip, state.avatarCamera->aspect(), display.settings().avatarMarginRightDip);
        int x = 0, y = 0, width = 0, height = 0;
        glfwGetWindowPos(window, &x, &y);
        glfwGetWindowSize(window, &width, &height);
        require(x == expectedRect.x && y == expectedRect.y && width == expectedRect.width && height == expectedRect.height,
            "Desktop Avatar does not follow selected work area / DPI");
        for (int i = 0; i < 2; ++i)
        {
            keyCallback(window, GLFW_KEY_F9, 0, GLFW_PRESS, 0);
            require(state.displayCommand == julia::desktop::DisplayCommand::ToggleTopmost, "F9 did not route topmost toggle");
            display.execute(state.displayCommand);
            state.displayCommand = julia::desktop::DisplayCommand::None;
            require((glfwGetWindowAttrib(window, GLFW_FLOATING) != 0) == display.settings().topmost,
                "Native always-on-top state incorrect");
        }
        cursorPositionCallback(window, 100, 100);
        mouseButtonCallback(window, GLFW_MOUSE_BUTTON_MIDDLE, GLFW_PRESS, 0);
        cursorPositionCallback(window, 180, 160);
        scrollCallback(window, 0, 3);
        processInput(window, 0);
        state.orbitCamera->update(1);
        require(glm::length(state.orbitCamera->position() - cameraBefore) < .0001f,
            "Desktop input changed saved orbit camera");
        display.execute(julia::desktop::DisplayCommand::Larger);
        draw("avatar_larger");
        display.execute(julia::desktop::DisplayCommand::Smaller);
        // Exercise real GPU rasterization at portrait pixel sizes derived from
        // other monitor/DPI configurations, without changing the user's displays.
        // Place these hidden test windows against the actual monitor work bottom.
        const auto actualArea = native.workArea(display.settings().avatarMonitor);
        int layoutSample = 0;
        for (const auto simulated : {julia::desktop::WorkArea{0,0,1920,1040,1},
            {0,0,2560,1392,1.25f}, {0,0,2560,1368,1.5f}, {0,0,2560,1356,1.75f},
            {0,0,3840,2064,2}, {0,0,3840,2112,1}, {0,48,1920,1152,1},
            {-3440,-200,3440,1392,1.5f}, {0,0,5120,1392,1}, {64,0,960,768,1.25f},
            {0,0,900,1552,1}, {0,0,321,153,3}})
        {
            auto rect = julia::desktop::avatarLayout(simulated,display.settings().avatarHeightDip,
                state.avatarCamera->aspect(),display.settings().avatarMarginRightDip);
            if (rect.width > actualArea.width || rect.height > actualArea.height) continue;
            rect.x = actualArea.x + actualArea.width - rect.width;
            rect.y = actualArea.y + actualArea.height - rect.height;
            native.resetShape();
            native.applyAvatarRect(rect);
            glfwPollEvents();
            draw("layout_" + std::to_string(layoutSample++));
        }
        display.refreshLayout();
        // A wide-window pan used to be lost if the compact viewport was fed
        // into OrbitCamera and clamped its saved screen-space composition.
        display.toggleMode();
        int fullWidth = 0, fullHeight = 0;
        glfwGetFramebufferSize(window, &fullWidth, &fullHeight);
        framebufferSizeCallback(window, fullWidth, fullHeight);
        state.orbitCamera->panNormalized({.85f, 0});
        state.orbitCamera->update(1);
        cameraBefore = state.orbitCamera->position();
        display.toggleMode();
        for (std::size_t i = 0; i < state.model->animations().size(); ++i)
        {
            state.player->selectClip(i);
            state.player->play();
            for (int sample = 0; sample <= 8; ++sample)
            {
                state.player->seek(state.player->clip()->durationSeconds() * sample / 8.0);
                draw("clip_" + std::to_string(i) + "_" + std::to_string(sample), sample == 4);
            }
        }
        const auto animationTime = state.player->timeSeconds();
        const auto animationClip = state.player->clipIndex();
        for (int i = 0; i < 6; ++i)
        {
            state.inputContext = {};
            keyCallback(window, GLFW_KEY_F10, 0, GLFW_PRESS, 0);
            require(state.displayCommand == julia::desktop::DisplayCommand::ToggleMode, "F10 did not route mode switch");
            display.execute(state.displayCommand);
            state.displayCommand = julia::desktop::DisplayCommand::None;
            glfwPollEvents();
            require(display.avatar() == (i % 2 == 1), "Live mode switch failed");
            require(glfwGetWindowAttrib(window, GLFW_DECORATED) == !display.avatar(), "Window decorations incorrect");
            require(glfwGetWindowAttrib(window, GLFW_RESIZABLE) == !display.avatar(), "Window resize state incorrect");
            draw(display.avatar() ? "avatar_restored" : "full_window");
        }
        require(state.player->timeSeconds() == animationTime, "Mode switch reset animation");
        require(state.player->clipIndex() == animationClip &&
            state.player->state() == julia::render_opengl::PlaybackState::Playing, "Mode switch changed playback state");
        require(glm::length(state.orbitCamera->position() - cameraBefore) < .0001f, "Mode switch reset orbit camera");
        std::cout << "Display verification completed: waist pixel coverage, headroom, all clips at nine times, "
            "alpha, MSAA, premultiplied sRGB, size, F9/F10, six live switches, shared scene\n";
    }

    void verifyCameraCallbacks(GLFWwindow* window, const std::filesystem::path& output)
    {
        auto& state = *getWindowState(window);
        auto& camera = *state.orbitCamera;
        const auto require = [](bool ok, const char* message) {
            if (!ok) throw std::runtime_error(message);
        };
        const auto same = [](glm::vec3 a, glm::vec3 b) { return glm::length(a - b) < 0.0001f; };
        const auto begin = [&](int button, int mods = 0) {
            // GLFW ignores cursor warps in an unfocused/hidden window. Feed the
            // same position event the native button callback normally follows.
            cursorPositionCallback(window, 100,100);
            mouseButtonCallback(window, button, GLFW_PRESS, mods);
        };
        const auto capture = [&](const char* name) {
            int width = 0, height = 0;
            glfwGetFramebufferSize(window, &width, &height);
            std::filesystem::create_directories(output);
            state.applyCamera();
            state.renderer->beginFrame();
            state.renderer->renderModel(*state.model, state.model->fitToHeightTransform(2.2f),
                state.bodyDisplay ? std::span<const glm::mat4>{state.bodyDisplay->pose.palette} : state.player->palette());
            std::vector<unsigned char> pixels(static_cast<std::size_t>(width) * height * 3);
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glReadPixels(0,0,width,height,GL_RGB,GL_UNSIGNED_BYTE,pixels.data());
            std::ofstream file(output / (std::string{name} + ".ppm"), std::ios::binary);
            file << "P6\n" << width << ' ' << height << "\n255\n";
            for (int y = height - 1; y >= 0; --y)
                file.write(reinterpret_cast<const char*>(pixels.data() +
                    static_cast<std::size_t>(y) * width * 3), width * 3);
            require(static_cast<bool>(file), "Cannot save camera verification frame");
        };
        state.inputContext = {};
        state.syncInputContext();
        camera.frameBody(true);
        const auto initial = camera.position();
        const auto pivot = camera.target();
        capture("body");
        begin(GLFW_MOUSE_BUTTON_LEFT);
        cursorPositionCallback(window, 220,160);
        mouseButtonCallback(window, GLFW_MOUSE_BUTTON_LEFT, GLFW_RELEASE, 0);
        camera.update(1);
        require(same(camera.position(), initial), "Left interaction moved camera");
        begin(GLFW_MOUSE_BUTTON_RIGHT);
        cursorPositionCallback(window, 220,160);
        mouseButtonCallback(window, GLFW_MOUSE_BUTTON_RIGHT, GLFW_RELEASE, 0);
        camera.update(1);
        require(!same(camera.position(), initial) && same(camera.target(), pivot), "Orbit callback failed");
        const auto beforePan = camera.target();
        begin(GLFW_MOUSE_BUTTON_MIDDLE);
        cursorPositionCallback(window, 140,120);
        mouseButtonCallback(window, GLFW_MOUSE_BUTTON_MIDDLE, GLFW_RELEASE, 0);
        camera.update(1);
        require(!same(camera.target(), beforePan), "Pan callback failed");
        const float distance = camera.distance();
        scrollCallback(window, 0,2);
        camera.update(1);
        require(camera.distance() < distance, "Zoom callback failed");
        keyCallback(window, GLFW_KEY_F, 0, GLFW_PRESS, 0);
        camera.update(1);
        require(camera.target().y > pivot.y, "Face focus callback failed");
        capture("face_angle");
        keyCallback(window, GLFW_KEY_B, 0, GLFW_PRESS, 0);
        camera.update(1);
        require(same(camera.position(), initial), "Camera reset callback failed");
        // Cancellation must survive return to the window without a new press.
        for (int reason = 0; reason < 3; ++reason)
        {
            begin(GLFW_MOUSE_BUTTON_RIGHT);
            if (reason == 0) focusCallback(window, GLFW_FALSE);
            if (reason == 1) cursorEnterCallback(window, GLFW_FALSE);
            if (reason == 2) { state.inputContext.mouseCapturedByUi = true; state.syncInputContext(); }
            scrollCallback(window, 0,10);
            cursorPositionCallback(window, 250,180);
            state.inputContext = {};
            state.syncInputContext();
            cursorPositionCallback(window, 300,180);
            camera.update(1);
            require(!state.cameraInput.active() && same(camera.position(), initial), "Cancelled drag resumed");
        }
        state.inputContext.keyboardCapturedByUi = true;
        keyCallback(window, GLFW_KEY_F, 0, GLFW_PRESS, 0);
        camera.update(1);
        require(same(camera.position(), initial), "UI keyboard capture ignored");
        state.inputContext = {};
        state.syncInputContext();
        camera.focusFace();
        camera.update(1);
        capture("face_front");
        camera.zoom(1000);
        camera.update(1);
        capture("closest_front");
        camera.orbit(-glm::radians(90.0f) / camera.settings().rotationRadiansPerPixel, 0);
        camera.update(1);
        capture("closest_profile");
        int width = 0, height = 0;
        glfwGetWindowSize(window, &width, &height);
        glfwSetWindowSize(window, 240,900);
        glfwPollEvents();
        int portraitWidth = 0, portraitHeight = 0;
        glfwGetFramebufferSize(window, &portraitWidth, &portraitHeight);
        framebufferSizeCallback(window, portraitWidth,portraitHeight);
        camera.frameBody(true);
        require(camera.distance() > distance, "Portrait window did not affect body fitting");
        capture("portrait_body");
        framebufferSizeCallback(window, 0,0);
        glfwSetWindowSize(window, width,height);
        glfwPollEvents();
        glfwGetFramebufferSize(window, &width, &height);
        framebufferSizeCallback(window, width,height);
        camera.frameBody(true);
        state.applyCamera();
        std::cout << "Camera callbacks verified: ownership, orbit/pan/zoom, F/B, UI capture, focus/leave, resize\n";
    }
}

int main(int argc, char** argv)
{
    const bool detourVerification = argc > 1 && std::string_view{argv[1]} == "--verify-detour";
    const bool collisionVerification = argc > 1 && std::string_view{argv[1]} == "--verify-collision";
    const bool reachVerification = argc > 1 && std::string_view{argv[1]} == "--verify-ik";
    const bool bodyVerification = detourVerification || collisionVerification || reachVerification || (argc > 1 && std::string_view{argv[1]} == "--verify-body");
    if (bodyVerification && argc != 3) { std::cerr << "Usage: julia_desktop --verify-body output_directory\n"; return 2; }
    const bool verification = argc > 1 && std::string_view{argv[1]} == "--verify";
    const bool animationVerification = argc > 1 &&
        (std::string_view{argv[1]} == "--verify-animations" || std::string_view{argv[1]} == "--review-animations");
    if (animationVerification && argc != 3)
    {
        std::cerr << "Usage: julia_desktop --verify-animations|--review-animations output_directory\n";
        return 2;
    }
    const bool eyeVerification = argc > 1 &&
        (std::string_view{argv[1]} == "--verify-eyes" || std::string_view{argv[1]} == "--review-eyes");
    bool noEyes = false;
    const bool displayVerification = argc > 1 && std::string_view{argv[1]} == "--verify-display";
    if (eyeVerification && argc != 3)
    {
        std::cerr << "Usage: julia_desktop --verify-eyes|--review-eyes output_directory\n";
        return 2;
    }
    // Модель можно передать аргументом командной строки; иначе берётся
    // ассет по умолчанию из каталога ресурсов.
    std::filesystem::path modelPath = assetDirectory(argv[0]) / "models" / "Raphtalia_prepared.fbx";
    std::filesystem::path settingsPath = assetDirectory(argv[0]).parent_path() / "display.ini";
    std::optional<julia::desktop::DisplayMode> requestedMode;
    std::optional<bool> requestedTopmost;
    std::optional<int> requestedHeight;
    bool persist = !verification && !animationVerification && !eyeVerification && !displayVerification && !bodyVerification;
    bool suppliedModel = false;
    for (int i = 1; i < argc && !verification && !animationVerification && !eyeVerification && !displayVerification && !bodyVerification; ++i)
    {
        const std::string_view arg{argv[i]};
        if (arg == "--desktop") requestedMode = julia::desktop::DisplayMode::DesktopAvatar;
        else if (arg == "--window") requestedMode = julia::desktop::DisplayMode::FullWindow;
        else if (arg == "--no-eyes") noEyes = true;
        else if (arg == "--topmost") requestedTopmost = true;
        else if (arg == "--no-topmost") requestedTopmost = false;
        else if (arg == "--no-save") persist = false;
        else if (arg == "--settings" && i + 1 < argc) settingsPath = argv[++i];
        else if (arg == "--avatar-height" && i + 1 < argc)
        {
            try
            {
                std::size_t consumed = 0;
                const std::string value{argv[++i]};
                const int height = std::stoi(value, &consumed);
                if (consumed != value.size() || height < 180 || height > 720) throw std::invalid_argument("height");
                requestedHeight = height;
            }
            catch (const std::exception&)
            {
                std::cerr << "--avatar-height requires 180..720 logical pixels\n";
                return 2;
            }
        }
        else if (arg == "--help")
        {
            std::cout << "Julia [--desktop|--window] [--topmost|--no-topmost] [--avatar-height 180..720]\n"
                << "      [--settings path] [--no-save] [--no-eyes] [model.fbx]\n"
                << "F10 switches display mode; F9 toggles desktop always-on-top.\n"
                << "Right-click the desktop character for Full Window, size, topmost and Exit.\n";
            return 0;
        }
        else if (arg.starts_with("--") || suppliedModel)
        {
            std::cerr << "Unknown or incomplete argument: " << arg << "; use --help\n";
            return 2;
        }
        else { modelPath = argv[i]; suppliedModel = true; }
    }
    auto displaySettings = verification || animationVerification || eyeVerification || displayVerification || bodyVerification
        ? julia::desktop::DisplaySettings{} : julia::desktop::loadDisplaySettings(settingsPath);
    if (requestedMode) displaySettings.mode = *requestedMode;
    if (requestedTopmost) displaySettings.topmost = *requestedTopmost;
    if (requestedHeight) displaySettings.avatarHeightDip = static_cast<float>(*requestedHeight);
    if (verification || animationVerification || eyeVerification || bodyVerification)
        displaySettings.mode = julia::desktop::DisplayMode::FullWindow;
    julia::desktop::setAvatarConsoleMode(displaySettings.mode == julia::desktop::DisplayMode::DesktopAvatar);

    glfwSetErrorCallback(glfwErrorCallback);

    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW\n";
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    // Required on macOS to get a core-profile context; harmless elsewhere.
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
    glfwWindowHint(GLFW_SRGB_CAPABLE, GLFW_TRUE);
    // Transparency is a creation-only capability, so request it in both modes.
    // Full Window writes alpha=1 and remains the original opaque grey viewer.
    glfwWindowHint(GLFW_TRANSPARENT_FRAMEBUFFER, GLFW_TRUE);
    glfwWindowHint(GLFW_ALPHA_BITS, 8);
    // avatarLayout owns DIP -> physical pixel conversion, including live DPI changes.
    glfwWindowHint(GLFW_SCALE_TO_MONITOR, GLFW_FALSE);
    glfwWindowHint(GLFW_DECORATED, displaySettings.mode == julia::desktop::DisplayMode::FullWindow);

    // Ask for a debug context so GL_KHR_debug can report errors via callback.
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GLFW_TRUE);

    // Сглаживание краёв модели и обводки.
    glfwWindowHint(GLFW_SAMPLES, animationVerification || eyeVerification || bodyVerification ? 4 : 8);

    const bool hidden = verification || displayVerification || bodyVerification || (animationVerification && std::string_view{argv[1]} == "--verify-animations") ||
        (eyeVerification && std::string_view{argv[1]} == "--verify-eyes");
    // Reveal only after the first correctly composed frame; no grey flash or
    // transient rectangular input blocker during model loading.
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    glfwWindowHint(GLFW_FOCUS_ON_SHOW, displaySettings.mode == julia::desktop::DisplayMode::FullWindow);
    GLFWwindow* window = glfwCreateWindow(verification ? 640 : animationVerification || eyeVerification || bodyVerification ? 768 : 1280,
        verification ? 480 : animationVerification || eyeVerification || bodyVerification ? 768 : 900, "Julia", nullptr, nullptr);
    if (!window)
    {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(verification || animationVerification || eyeVerification || bodyVerification ? 0 : 1);

    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)))
    {
        std::cerr << "Failed to initialize glad\n";
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    std::cout << "OpenGL version: " << glGetString(GL_VERSION) << "\n";

    // Available when the driver exposes GL_KHR_debug (glad leaves the pointer
    // null otherwise). Synchronous output keeps messages next to the call.
    if (glDebugMessageCallback)
    {
        glEnable(GL_DEBUG_OUTPUT);
        glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
        glDebugMessageCallback(glDebugCallback, nullptr);
    }

    GLint samples = 0;
    glGetIntegerv(GL_SAMPLES, &samples);
    std::cout << "MSAA samples: " << samples << "\n";

    int framebufferWidth = 0;
    int framebufferHeight = 0;
    glfwGetFramebufferSize(window, &framebufferWidth, &framebufferHeight);

    int result = 0;

    try
    {
        julia::render_opengl::OpenGLRenderer renderer;
        renderer.initialize(framebufferWidth, framebufferHeight, assetDirectory(argv[0]));

        std::cout << "Loading model: " << modelPath.string() << "\n";

        // Both bundled rigged exports contain per-face normals, including the
        // prepared default. Repair these known assets; other models may carry
        // deliberately authored toon normals that must remain unchanged.
        const auto isBundledAsset = [&](const char* name) {
            std::error_code assetComparisonError;
            return std::filesystem::equivalent(modelPath,
                assetDirectory(argv[0]) / "models" / name, assetComparisonError) ||
                std::filesystem::equivalent(modelPath,
                    std::filesystem::path{JULIA_ASSET_DIR} / "models" / name,
                    assetComparisonError);
        };
        const bool repairFacetedNormals = isBundledAsset("Raphtalia.fbx") ||
            isBundledAsset("Raphtalia_prepared.fbx");
        auto asset = julia::model_import::ModelImporter::loadFromFile(modelPath,
            {.rebuildSmoothNormals = repairFacetedNormals});

        auto model = julia::render_opengl::GLModelLoader::upload(asset);

        std::cout << "Model loaded\n";
        std::cout << "Meshes: " << model.meshCount() << "\n";
        std::cout << "Bones: " << model.boneCount() << "\n";
        std::cout << "Animation clips: " << model.animations().size() << "\n";
        std::size_t texturedMeshes = 0;
        for (const auto& mesh : model.meshes())
            if (mesh.material().diffuseTexture) ++texturedMeshes;
        std::cout << "Textured meshes: " << texturedMeshes << '/' << model.meshCount() << "\n";
        if (texturedMeshes == 0)
            std::cerr << "Material diagnostic: this asset has no assigned images; using imported diffuse values.\n";
        std::unique_ptr<julia::desktop::BodyDiagnostics> bodyDiagnostics;
        const auto bodyProfile=julia::embodiment::raphtaliaBodyProfile();
        const auto bodyIssues=julia::embodiment::validateBodyProfile(bodyProfile,*asset);
        julia::embodiment::BodyInstance body{1,asset,bodyIssues.empty() ? &bodyProfile:nullptr,!noEyes};
        julia::runtime::FixedStepScheduler scheduler;
        auto bodyDisplay = body.display(1);
        // Old suites deliberately exercise the standalone player API. Their
        // fixture exists only in those test processes, never in the live app.
        const bool legacyVerification = displayVerification || animationVerification || eyeVerification;
        std::unique_ptr<julia::embodiment::EyePoseLayer> legacyEyes;
        std::unique_ptr<julia::embodiment::AnimationPlayer> legacyPlayer;
        if (legacyVerification)
        {
            legacyEyes=std::make_unique<julia::embodiment::EyePoseLayer>(*asset);
            legacyEyes->setEnabled(!animationVerification && !noEyes);
            legacyPlayer=std::make_unique<julia::embodiment::AnimationPlayer>(*asset);
            legacyPlayer->setEyeLayer(legacyEyes.get());
        }
        const auto& player=legacyPlayer ? *legacyPlayer:body.player();
        const auto& eyes=legacyEyes ? *legacyEyes:body.eyes();
        const auto* bodyControl=body.control();
        std::cout << "Procedural eyes: " << (eyes.available() ? "ready (188 baked eyelid bones)" : "unavailable for this rig") << '\n';
        if (bodyIssues.empty())
        {
            if (!legacyVerification) bodyDiagnostics=std::make_unique<julia::desktop::BodyDiagnostics>(body);
        }
        else
        {
            std::cerr << "Body calibration unavailable:";
            for (const auto& issue:bodyIssues) std::cerr << '\n' << issue.message;
            std::cerr << '\n';
            if (bodyVerification) throw std::runtime_error("Prepared body profile could not bind");
        }
        const glm::mat4 fitModel = model.fitToHeightTransform(2.2f);

        julia::render_opengl::CameraConfig cameraConfig{};
        const auto subject = cameraSubject(model, fitModel);
        const float subjectHeight = subject.max.y - subject.min.y;
        cameraConfig.nearPlane = subjectHeight * 0.01f;
        cameraConfig.farPlane = subjectHeight * 100.0f;
        julia::render::OrbitCamera orbitCamera{subject};
        orbitCamera.setViewport(static_cast<float>(framebufferWidth),
            static_cast<float>(framebufferHeight), cameraConfig.fovDegrees);
        orbitCamera.frameBody(true);

        std::optional<std::uint32_t> waistNode;
        std::optional<glm::vec3> waistPoint;
        for (std::uint32_t i = 0; i < model.nodes().size(); ++i)
        {
            auto name = model.nodes()[i].name;
            std::transform(name.begin(), name.end(), name.begin(),
                [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            if (const auto colon = name.rfind(':'); colon != std::string::npos) name.erase(0, colon + 1);
            if (name == "spine" || name.starts_with("spine_01") || name == "spine.001")
            {
                const auto point = glm::vec3{fitModel * model.nodes()[i].globalBindTransform * glm::vec4{0,0,0,1}};
                if (point.y > subject.min.y + subjectHeight * .35f &&
                    point.y < subject.min.y + subjectHeight * .66f)
                {
                    waistNode = i;
                    waistPoint = point;
                    break;
                }
            }
        }
        julia::render::AvatarCamera avatarCamera{subject, waistPoint};
        julia::render_opengl::AvatarSurface avatarSurface;
        julia::desktop::DisplayController display{window, displaySettings, avatarCamera.aspect(), settingsPath, persist};
        std::cout << "Avatar portrait: aspect " << avatarCamera.aspect() << ", waist "
            << avatarCamera.waistHeight() << ", height " << avatarCamera.frameHeight()
            << (waistNode ? ", rig anchor " + model.nodes()[*waistNode].name : ", bounds fallback") << '\n';

        WindowState windowState{};
        windowState.renderer = &renderer;
        windowState.orbitCamera = &orbitCamera;
        windowState.avatarCamera = &avatarCamera;
        windowState.display = &display;
        windowState.anchorNode = waistNode;
        windowState.fitModel = fitModel;
        windowState.cameraConfig = cameraConfig;
        windowState.player = legacyPlayer.get();
        windowState.eyes = legacyEyes.get();
        windowState.body = legacyVerification ? nullptr:&body;
        windowState.scheduler = &scheduler;
        windowState.bodyDisplay = legacyVerification ? nullptr:&bodyDisplay;
        windowState.bodyDiagnostics = bodyDiagnostics.get();
        windowState.model = &model;
        glfwGetCursorPos(window, &windowState.cursorX, &windowState.cursorY);
        windowState.inputContext.focused = hidden || glfwGetWindowAttrib(window, GLFW_FOCUSED) != 0;
        windowState.inputContext.inside = hidden || glfwGetWindowAttrib(window, GLFW_HOVERED) != 0;
        windowState.syncInputContext();

        glfwSetWindowUserPointer(window, &windowState);
        glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
        glfwSetMouseButtonCallback(window, mouseButtonCallback);
        glfwSetCursorPosCallback(window, cursorPositionCallback);
        glfwSetScrollCallback(window, scrollCallback);
        glfwSetWindowFocusCallback(window, focusCallback);
        glfwSetCursorEnterCallback(window, cursorEnterCallback);
        glfwSetKeyCallback(window, keyCallback);

        if (!verification && !animationVerification && !eyeVerification && !bodyVerification) display.initialize();
        glfwGetFramebufferSize(window, &framebufferWidth, &framebufferHeight);
        framebufferSizeCallback(window, framebufferWidth, framebufferHeight);

        windowState.applyCamera();

        std::cout
            << "Display: F10 Desktop Avatar / Full Window; F9 desktop always-on-top.\n"
            << "Desktop: right-click character for window, size, topmost and Exit.\n"
            << "Camera: right drag orbit; middle or Shift+right drag pan; wheel/W/S zoom.\n"
            << "A/D/Q/E or arrows pan; F face (keep angle); B reset front/body; Esc exit.\n"
            << "Left mouse is reserved for character interaction.\n";
        std::cout << "Animation: PgUp/PgDn select, Space play/pause, X/R stop + bind, L loop,\n"
            << "1..4 blink clips, comma/period step 1/30 s, F face / B body, F1 list clips.\n";
        std::cout << "Eyes: G natural/resume, H auto blink, J auto gaze, K blink, U/I wink,\n"
            << "numpad 1..9 gaze, Home forward, T hold camera point, [/] blink interval, F8 fixed seed.\n";
        std::cout << "Body: F2 direct mode, Shift+F3/F3 previous/next joint, F4 coordinate, +/- 5 deg, Backspace zero, F5 axes. Pause full simulation, F6 full reset.\n";
        std::cout << "IK (direct mode): F7 reach/retarget wrist, Shift+F7 arm, Z/C X-/+, V/N Y-/+, O/P Z-/+ (25 mm, Shift 5 mm), Delete finish segment and hold.\n";
        std::cout << "Collision: F11 cycles off/free/blocked demo scene while idle; F12 selects DirectOnly/AllowDetour for the next F7 command.\n";
        keyCallback(window, GLFW_KEY_F1, 0, GLFW_PRESS, 0);
        if (bodyVerification)
        {
            const auto verify=detourVerification ? julia::desktop::verifyDesktopDetour:collisionVerification ? julia::desktop::verifyDesktopCollision:
                reachVerification ? julia::desktop::verifyDesktopReach:julia::desktop::verifyDesktopBody;
            verify(window,renderer,model,body,*bodyDiagnostics,fitModel,
                std::filesystem::path{argv[2]},keyCallback);
            glfwSetWindowShouldClose(window,GLFW_TRUE);
        }
        if (verification) verifyCameraCallbacks(window, argc > 2 ? std::filesystem::path{argv[2]} :
            std::filesystem::temp_directory_path() / "Julia/verification/desktop_camera");
        if (displayVerification)
        {
            verifyDisplayModes(window, avatarSurface, argc > 2 ? std::filesystem::path{argv[2]} :
                std::filesystem::temp_directory_path() / "Julia/verification/desktop_display");
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        }
        if (animationVerification)
        {
            verifyDesktopAnimations(window, renderer, model, *legacyPlayer, fitModel,
                std::filesystem::path{argv[2]}, keyCallback);
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        }
        if (eyeVerification)
        {
            verifyDesktopEyes(window, renderer, model, *legacyPlayer, *legacyEyes, fitModel,
                std::filesystem::path{argv[2]}, keyCallback);
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        }

        double previousTime = glfwGetTime();
        double titleTime = previousTime;
        int titleFrames = 0;
        int verificationFrame = 0;
        std::uint64_t pausedVerificationTick = 0;
        bool revealed = false;
        double layoutTime = -1;
        while (!glfwWindowShouldClose(window))
        {
            // Poll before updating to render the latest input in this frame.
            glfwPollEvents();
            if (windowState.contextMenuRequested)
            {
                windowState.contextMenuRequested = false;
                julia::desktop::WindowPlatform native{window};
                windowState.displayCommand = native.contextMenu(display.settings().topmost);
                windowState.cameraInput.cancel();
            }
            if (windowState.displayCommand != julia::desktop::DisplayCommand::None)
            {
                display.execute(windowState.displayCommand);
                windowState.displayCommand = julia::desktop::DisplayCommand::None;
                windowState.cameraInput.cancel();
                glfwGetFramebufferSize(window, &framebufferWidth, &framebufferHeight);
                framebufferSizeCallback(window, framebufferWidth, framebufferHeight);
            }
            const double now = glfwGetTime();
            const double deltaSeconds = verification ? 1.0 / 120.0 : now - previousTime;
            previousTime = now;
            if (now - layoutTime >= .5)
            {
                display.refreshLayout();
                layoutTime = now;
            }
            if (verification)
            {
                // Stage 4 exercises the production queued-input/scheduler/display
                // path in the ordinary loop, alongside the existing camera checks.
                const auto queuedKey=[&](int key)
                {
                    const auto saved=body.snapshot();
                    keyCallback(window,key,0,GLFW_PRESS,0);
                    if (body.snapshot().tick!=saved.tick || body.snapshot().clipTime!=saved.clipTime ||
                        body.snapshot().coordinates!=saved.coordinates || body.snapshot().pose.palette!=saved.pose.palette)
                        throw std::runtime_error("Keyboard changed completed simulation state before tick");
                };
                if (verificationFrame==2) queuedKey(GLFW_KEY_G);
                if (verificationFrame==4) queuedKey(GLFW_KEY_PAGE_DOWN);
                if (verificationFrame==8) queuedKey(GLFW_KEY_SPACE);
                if (verificationFrame==9) queuedKey(GLFW_KEY_KP_6);
                if (verificationFrame==12) queuedKey(GLFW_KEY_K);
                if (verificationFrame==16) queuedKey(GLFW_KEY_PERIOD);
                if (bodyControl && verificationFrame==24) queuedKey(GLFW_KEY_F2);
                if (bodyControl && verificationFrame==25) { queuedKey(GLFW_KEY_EQUAL); queuedKey(GLFW_KEY_EQUAL); }
                if (bodyControl && verificationFrame==26) queuedKey(GLFW_KEY_MINUS);
                if (bodyControl && verificationFrame==40) queuedKey(GLFW_KEY_F2);
                if (verificationFrame==50) queuedKey(GLFW_KEY_T);
                if (verificationFrame==60) queuedKey(GLFW_KEY_F8);
                if (verificationFrame==70) body.enqueue(julia::embodiment::SetRoot{{{.03f,0,0},{1,0,0,0}}});
                if (verificationFrame==80) queuedKey(GLFW_KEY_F6);
                if (verificationFrame==90) { queuedKey(GLFW_KEY_PAUSE); pausedVerificationTick=body.snapshot().tick; }
                if (verificationFrame>90 && verificationFrame<94 && body.snapshot().tick!=pausedVerificationTick)
                    throw std::runtime_error("Full simulation pause advanced body");
                if (verificationFrame==94) queuedKey(GLFW_KEY_PAUSE);
                // Exercise the actual desktop input callbacks without depending
                // on OS focus or synthetic global mouse/keyboard injection.
                if (verificationFrame == 10 || verificationFrame == 70)
                {
                    const int button = verificationFrame == 10
                        ? GLFW_MOUSE_BUTTON_RIGHT : GLFW_MOUSE_BUTTON_MIDDLE;
                    cursorPositionCallback(window, 100,100);
                    mouseButtonCallback(window, button, GLFW_PRESS, 0);
                    cursorPositionCallback(window, 180.0,130.0);
                    mouseButtonCallback(window, button, GLFW_RELEASE, 0);
                }
                if (verificationFrame == 40) scrollCallback(window, 0.0, 2.0);
                if (++verificationFrame >= 120)
                    glfwSetWindowShouldClose(window, GLFW_TRUE);
            }
            if (glfwGetWindowAttrib(window, GLFW_ICONIFIED))
            {
                windowState.cameraInput.cancel();
                scheduler.advance(deltaSeconds,[&] { body.step(); },true);
                body.collapseHistory();
                glfwWaitEventsTimeout(0.05);
                continue;
            }
            // A long OS pause must not turn held keys into a large jump.
            processInput(window, deltaSeconds <= 0.25 ? deltaSeconds : 0.0);
            if (!display.avatar()) orbitCamera.update(deltaSeconds);
            const auto suspendCount=scheduler.diagnostics().suspendedFrames;
            scheduler.advance(deltaSeconds,[&] { body.step(); });
            if (scheduler.diagnostics().suspendedFrames!=suspendCount) body.collapseHistory();
            bodyDisplay=body.display(scheduler.paused() ? 1:scheduler.alpha());
            windowState.applyCamera();

            glfwGetFramebufferSize(window, &framebufferWidth, &framebufferHeight);
            if (framebufferWidth <= 0 || framebufferHeight <= 0) continue;
            if (display.avatar()) avatarSurface.begin(framebufferWidth, framebufferHeight);
            else glBindFramebuffer(GL_FRAMEBUFFER, 0);
            renderer.beginFrame(display.avatar() ? glm::vec4{0} : glm::vec4{0.21586f,0.21586f,0.21586f,1});

            glm::mat4 modelMatrix{1.0f};

            modelMatrix = fitModel * body.resourceRoot(bodyDisplay.root);

            renderer.renderModel(model, modelMatrix, bodyDisplay.pose.palette);
            if (bodyDiagnostics) bodyDiagnostics->draw(renderer,modelMatrix,bodyDisplay);
            if (display.avatar())
            {
                const auto coverage = avatarSurface.resolveCoverage();
                display.updateShape(coverage, framebufferWidth, framebufferHeight);
                avatarSurface.present();
            }
            ++titleFrames;
            if (now - titleTime >= .25)
            {
                constexpr const char* states[]{"stopped/bind", "playing", "paused", "finished"};
                std::ostringstream title;
                title << "JULIA | ";
                if (bodyDiagnostics && bodyControl && bodyControl->enabled()) title << bodyDiagnostics->title() << " | ";
                title << (player.clip() ? player.clip()->name : "no clips")
                    << " | " << states[static_cast<int>(player.state())] << ' '
                    << std::fixed << std::setprecision(2) << player.timeSeconds() << '/'
                    << (player.clip() ? player.clip()->durationSeconds() : 0.0) << " s | loop "
                    << (player.looping() ? "on" : "off") << " | " << std::setprecision(0)
                    << titleFrames / (now - titleTime) << " FPS | eyes " << (eyes.enabled() ? "on" : "off")
                    << " blink " << eyes.behavior().output().blinkLeft << '/' << eyes.behavior().output().blinkRight
                    << " | tick " << body.snapshot().tick << " dropped " << scheduler.diagnostics().discardedSeconds
                    << " s overload " << scheduler.diagnostics().overloadFrames << (scheduler.paused() ? " SIM PAUSED":"")
                    << " | RMB orbit MMB pan Wheel zoom F face B reset | PgUp/Dn Space X";
                glfwSetWindowTitle(window, title.str().c_str());
                titleFrames = 0;
                titleTime = now;
            }

            if (verification && glGetError() != GL_NO_ERROR)
                throw std::runtime_error("OpenGL error during desktop verification");
            if (verification && verificationFrame==120)
            {
                if (body.snapshot().tick!=116 || body.snapshot().epoch!=1 || body.pendingCommands()!=0)
                    throw std::runtime_error("Desktop fixed-step input/reset/pause integration failed");
                const auto output=std::filesystem::path{JULIA_ASSET_DIR}.parent_path()/"build/diagnostics/stage4/desktop";
                std::filesystem::create_directories(output);
                std::ofstream report(output/"runtime.txt");
                if (!report) throw std::runtime_error("Cannot write runtime diagnostic evidence");
                report << "production_loop_frames=120\ncompleted_ticks=" << body.snapshot().tick
                    << "\nfull_pause_frames=4\nreset_epoch=" << body.snapshot().epoch
                    << "\ndisplay_from_tick=" << bodyDisplay.fromTick << "\ndisplay_to_tick=" << bodyDisplay.toTick
                    << "\ndisplay_time=" << std::setprecision(12) << bodyDisplay.time
                    << "\nqueued_clip_gaze_target_direct_reset=PASS\ncamera_source=same_BodyDisplay_as_render\nGL_errors=0\nresult=PASS\n";
            }

            glfwSwapBuffers(window);
            if (!hidden && !revealed)
            {
                glfwShowWindow(window);
                revealed = true;
            }
            // A companion should not busy-loop when the driver ignores VSync.
            // Event-aware pacing keeps input responsive and animations at 60 Hz.
            if (display.avatar() && !hidden)
            {
                const double remaining = 1.0 / 60.0 - (glfwGetTime() - now);
                if (remaining > 0) glfwWaitEventsTimeout(remaining);
            }
        }
        display.save();
        // Callbacks must not outlive the scene objects during stack unwinding.
        glfwSetWindowUserPointer(window, nullptr);
        if (verification)
            std::cout << "Desktop verification completed: orbit, zoom, pan, 120 frames\n";
    }
    catch (const std::exception& exception)
    {
        std::cerr << "Fatal error: " << exception.what() << "\n";
        result = 1;
    }

    glfwSetWindowUserPointer(window, nullptr);

    glfwDestroyWindow(window);
    glfwTerminate();

    return result;
}
