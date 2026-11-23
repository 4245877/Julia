#pragma once
#include "julia/embodiment/BodyInstance.hpp"
#include <filesystem>
#include <functional>

namespace julia::desktop
{
    struct SceneFrame
    {
        int width=0,height=0;
        std::vector<unsigned char> rgba;
    };
    struct SceneCloseUp
    {
        glm::vec3 target, offset; // Simulation world metres.
    };
    // Draw through the regular Full Window or AvatarSurface path.
    using CaptureSceneFrame=std::function<SceneFrame(const embodiment::BodyDisplay&,bool avatar,
        std::optional<SceneCloseUp> closeUp,float opacity,bool drawCharacter)>;
    void saveSceneFrame(const std::filesystem::path& file,const SceneFrame& frame);
    void verifyDesktopReadyCube(embodiment::BodyInstance& body,const std::filesystem::path& output,
        std::string_view rate,const CaptureSceneFrame& capture);
    void verifyDesktopReadySphere(embodiment::BodyInstance& body,const std::filesystem::path& output,
        std::string_view rate,const CaptureSceneFrame& capture);
}
