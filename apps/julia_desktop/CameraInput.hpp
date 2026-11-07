#pragma once

#include "julia/render/OrbitCamera.hpp"
#include <cmath>

namespace julia::desktop
{
    struct CameraInputContext
    {
        bool focused = true;
        bool inside = true;
        bool mouseCapturedByUi = false;
        bool keyboardCapturedByUi = false;
    };

    // GLFW-independent gesture ownership. UI must update context before routing
    // events (and each frame); a cancelled drag always requires a fresh press.
    class CameraInput
    {
    public:
        enum class Button { Left, Right, Middle };
        static constexpr double DragThresholdPixels = 4.0;

        void setContext(CameraInputContext context)
        {
            context_ = context;
            if (!acceptsMouse()) cancel();
        }
        bool acceptsMouse() const
        {
            return context_.focused && context_.inside && !context_.mouseCapturedByUi;
        }
        bool acceptsKeyboard() const
        {
            return context_.focused && !context_.keyboardCapturedByUi;
        }
        void cancel() { active_ = false; dragging_ = false; }
        bool active() const { return active_; }

        void press(Button button, bool shift, double x, double y)
        {
            if (!acceptsMouse() || button == Button::Left || active_ ||
                !std::isfinite(x) || !std::isfinite(y)) return;
            button_ = button;
            panning_ = button == Button::Middle || shift;
            active_ = true;
            dragging_ = false;
            startX_ = lastX_ = x;
            startY_ = lastY_ = y;
        }
        void release(Button button)
        {
            if (active_ && button == button_) cancel();
        }
        void move(double x, double y, float windowHeight, render::OrbitCamera& camera)
        {
            if (!acceptsMouse()) { cancel(); return; }
            if (!active_ || !std::isfinite(x) || !std::isfinite(y)) return;
            double dx = x - lastX_, dy = y - lastY_;
            lastX_ = x;
            lastY_ = y;
            if (!dragging_)
            {
                dx = x - startX_;
                dy = y - startY_;
                const double length = std::hypot(dx, dy);
                if (length <= DragThresholdPixels) return;
                const double fraction = (length - DragThresholdPixels) / length;
                dx *= fraction;
                dy *= fraction;
                dragging_ = true;
            }
            if (panning_) camera.pan(static_cast<float>(dx), static_cast<float>(dy), windowHeight);
            else camera.orbit(static_cast<float>(dx), static_cast<float>(dy));
        }
        void scroll(double steps, render::OrbitCamera& camera) const
        {
            if (acceptsMouse()) camera.zoom(static_cast<float>(steps));
        }

    private:
        CameraInputContext context_;
        Button button_ = Button::Right;
        bool active_ = false, dragging_ = false, panning_ = false;
        double startX_ = 0, startY_ = 0, lastX_ = 0, lastY_ = 0;
    };
}
