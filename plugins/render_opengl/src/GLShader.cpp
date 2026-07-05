#include "julia/render_opengl/GLShader.hpp"

#include <glm/gtc/type_ptr.hpp>

#include <cassert>
#include <stdexcept>
#include <string>
#include <vector>

namespace julia::render_opengl
{
    namespace
    {
        void assertProgramBound(GLuint program)
        {
#ifndef NDEBUG
            GLint current = 0;
            glGetIntegerv(GL_CURRENT_PROGRAM, &current);
            assert(
                static_cast<GLuint>(current) == program &&
                "GLShader uniform set while a different program is bound; "
                "call bind() on this shader first."
            );
#else
            (void)program;
#endif
        }
    }

    GLShader::~GLShader()
    {
        release();
    }

    GLShader::GLShader(GLShader&& other) noexcept
        : program_(other.program_)
    {
        other.program_ = 0;
    }

    GLShader& GLShader::operator=(GLShader&& other) noexcept
    {
        if (this != &other)
        {
            release();

            program_ = other.program_;
            other.program_ = 0;
        }

        return *this;
    }

    void GLShader::createFromSource(const char* vertexSource, const char* fragmentSource)
    {
        release();

        const GLuint vertexShader = compileStage(GL_VERTEX_SHADER, vertexSource);

        GLuint fragmentShader = 0;
        try
        {
            fragmentShader = compileStage(GL_FRAGMENT_SHADER, fragmentSource);
        }
        catch (...)
        {
            // compileStage already deleted the failed fragment shader; make
            // sure the successfully compiled vertex shader does not leak.
            glDeleteShader(vertexShader);
            throw;
        }

        program_ = glCreateProgram();
        glAttachShader(program_, vertexShader);
        glAttachShader(program_, fragmentShader);
        glLinkProgram(program_);

        GLint success = 0;
        glGetProgramiv(program_, GL_LINK_STATUS, &success);

        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);

        if (!success)
        {
            GLint logLength = 0;
            glGetProgramiv(program_, GL_INFO_LOG_LENGTH, &logLength);

            std::string log;
            log.resize(static_cast<std::size_t>(logLength));

            glGetProgramInfoLog(program_, logLength, nullptr, log.data());

            release();

            throw std::runtime_error("OpenGL shader link error:\n" + log);
        }
    }

    void GLShader::bind() const
    {
        glUseProgram(program_);
    }

    void GLShader::setBool(const std::string& name, bool value) const
    {
        assertProgramBound(program_);
        glUniform1i(uniformLocation(name), value ? 1 : 0);
    }

    void GLShader::setInt(const std::string& name, int value) const
    {
        assertProgramBound(program_);
        glUniform1i(uniformLocation(name), value);
    }

    void GLShader::setFloat(const std::string& name, float value) const
    {
        assertProgramBound(program_);
        glUniform1f(uniformLocation(name), value);
    }

    void GLShader::setVec3(const std::string& name, const glm::vec3& value) const
    {
        assertProgramBound(program_);
        glUniform3fv(uniformLocation(name), 1, glm::value_ptr(value));
    }

    void GLShader::setMat4(const std::string& name, const glm::mat4& value) const
    {
        assertProgramBound(program_);
        glUniformMatrix4fv(
            uniformLocation(name),
            1,
            GL_FALSE,
            glm::value_ptr(value)
        );
    }

    void GLShader::setMat4Array(const std::string& name, std::span<const glm::mat4> values) const
    {
        if (values.empty())
        {
            return;
        }

        assertProgramBound(program_);

        const GLint location = uniformLocation(name + "[0]");

        if (location < 0)
        {
            return;
        }

        glUniformMatrix4fv(
            location,
            static_cast<GLsizei>(values.size()),
            GL_FALSE,
            glm::value_ptr(values.front())
        );
    }

    GLint GLShader::uniformLocation(const std::string& name) const
    {
        const auto cached = uniformLocations_.find(name);
        if (cached != uniformLocations_.end())
        {
            return cached->second;
        }

        const GLint location = glGetUniformLocation(program_, name.c_str());
        uniformLocations_.emplace(name, location);

        return location;
    }

    GLuint GLShader::compileStage(GLenum type, const char* source)
    {
        const GLuint shader = glCreateShader(type);

        glShaderSource(shader, 1, &source, nullptr);
        glCompileShader(shader);

        GLint success = 0;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);

        if (!success)
        {
            GLint logLength = 0;
            glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &logLength);

            std::string log;
            log.resize(static_cast<std::size_t>(logLength));

            glGetShaderInfoLog(shader, logLength, nullptr, log.data());
            glDeleteShader(shader);

            throw std::runtime_error("OpenGL shader compile error:\n" + log);
        }

        return shader;
    }

    void GLShader::release()
    {
        if (program_ != 0)
        {
            glDeleteProgram(program_);
            program_ = 0;
        }

        // Locations are only valid for the program that produced them.
        uniformLocations_.clear();
    }
}