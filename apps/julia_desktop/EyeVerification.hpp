#pragma once

#include "AnimationVerification.hpp"
#include "julia/render_opengl/EyePoseLayer.hpp"

// Explicit integration audit in the real desktop renderer and input context.
void verifyDesktopEyes(GLFWwindow* window,
    julia::render_opengl::OpenGLRenderer& renderer,
    const julia::render_opengl::GLModel& model,
    julia::render_opengl::AnimationPlayer& player,
    julia::render_opengl::EyePoseLayer& eyes, const glm::mat4& fit,
    const std::filesystem::path& output, DesktopKeyCallback input);
