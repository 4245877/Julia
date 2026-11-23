#pragma once
#include "ReadySceneVerification.hpp"
#include <span>

namespace julia::desktop
{
    // Simulation world metres. The camera looks from eye at target with the given up vector.
    struct HandCamera { glm::vec3 target{0}, eye{0,0,1}, up{0,1,0}; float fovDegrees=32; };
    struct HandLine { glm::vec3 a{0}, b{0}, color{1}; };
    // Full Window or AvatarSurface through the regular renderer; optional close-up and overlay lines.
    using CaptureHandFrame=std::function<SceneFrame(const embodiment::BodyDisplay&,bool avatar,
        const HandCamera* camera,std::span<const HandLine> overlay)>;
    // Optional pose file: one pose per line, "name" followed by 21 angles in degrees:
    // MCP flexion, spread, PIP, DIP for index..pinky, then thumb palmar abduction, radial
    // abduction, pronation, MCP and IP flexion. Each pose is rendered from the standard views.
    void verifyDesktopHands(embodiment::BodyInstance& body,const std::filesystem::path& output,
        const CaptureHandFrame& capture,const std::filesystem::path& extraPoses={});
}
