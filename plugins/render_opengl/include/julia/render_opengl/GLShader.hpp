#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>

#include <span>
#include <string>

namespace julia::render_opengl
{
    class GLShader
    {
    public:
        GLShader() = default;
        ~GLShader();

        GLShader(const GLShader&) = delete;
        GLShader& operator=(const GLShader&) = delete;

        GLShader(GLShader&& other) noexcept;
        GLShader& operator=(GLShader&& other) noexcept;

        void createFromSource(const char* vertexSource, const char* fragmentSource);
        void bind() const;

        void setBool(const std::string& name, bool value) const;
        void setInt(const std::string& name, int value) const;
        void setFloat(const std::string& name, float value) const;
        void setVec3(const std::string& name, const glm::vec3& value) const;
        void setMat4(const std::string& name, const glm::mat4& value) const;
        void setMat4Array(const std::string& name, std::span<const glm::mat4> values) const;

        GLuint id() const
        {
            return program_;
        }

        bool valid() const
        {
            return program_ != 0;
        }

    private:
        GLuint program_ = 0;

        static GLuint compileStage(GLenum type, const char* source);
        void release();
    };
}