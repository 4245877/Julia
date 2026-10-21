#pragma once

#include "julia/render_opengl/OpenGLRenderer.hpp"
#include <filesystem>

struct GLFWwindow;
using DesktopKeyCallback = void (*)(GLFWwindow*, int, int, int, int);

// Runs inside the actual desktop process/context, with its imported model,
// renderer, player and input callback. Evidence is written only on explicit use.
void verifyDesktopAnimations(GLFWwindow* window,
    julia::render_opengl::OpenGLRenderer& renderer,
    const julia::render_opengl::GLModel& model,
    julia::render_opengl::AnimationPlayer& player, const glm::mat4& fit,
    const std::filesystem::path& output, DesktopKeyCallback input);
