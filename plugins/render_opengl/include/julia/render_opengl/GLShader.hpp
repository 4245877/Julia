#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>

#include <span>
#include <string>
#include <unordered_map>

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

        // Precondition for every set* below: this program must be the currently
        // bound one (call bind() first). glUniform* always writes into the
        // program returned by GL_CURRENT_PROGRAM, not into program_; a debug
        // assertion guards against setting uniforms on the wrong shader.
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

        // Resolved once per uniform name and reused: glGetUniformLocation is a
        // string lookup that would otherwise run several times per mesh per
        // frame. Mutable because the set* accessors are logically const.
        mutable std::unordered_map<std::string, GLint> uniformLocations_;

        GLint uniformLocation(const std::string& name) const;

        static GLuint compileStage(GLenum type, const char* source);
        void release();
    };
}