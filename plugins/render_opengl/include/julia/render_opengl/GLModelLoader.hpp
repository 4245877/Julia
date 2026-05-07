#pragma once

#include "julia/render_opengl/GLModel.hpp"

#include <filesystem>

namespace julia::render_opengl
{
    class GLModelLoader
    {
    public:
        static GLModel loadFromFile(const std::filesystem::path& path);
    };
}