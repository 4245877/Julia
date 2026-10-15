#include "julia/render_opengl/GLShader.hpp"

#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cassert>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include <utility>

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
        : program_(std::exchange(other.program_, 0)),
          uniformLocations_(std::move(other.uniformLocations_))
    {
        other.uniformLocations_.clear();
    }

    GLShader& GLShader::operator=(GLShader&& other) noexcept
    {
        if (this != &other)
        {
            release();

            program_ = std::exchange(other.program_, 0);
            uniformLocations_ = std::move(other.uniformLocations_);
            other.uniformLocations_.clear();
        }

        return *this;
    }

    void GLShader::createFromSource(const char* vertexSource, const char* fragmentSource)
    {
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

        const GLuint candidate = glCreateProgram();
        if (!candidate)
        {
            glDeleteShader(vertexShader);
            glDeleteShader(fragmentShader);
            throw std::runtime_error("OpenGL failed to create a shader program");
        }
        glAttachShader(candidate, vertexShader);
        glAttachShader(candidate, fragmentShader);
        glLinkProgram(candidate);

        GLint success = 0;
        glGetProgramiv(candidate, GL_LINK_STATUS, &success);

        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);

        if (!success)
        {
            GLint logLength = 0;
            glGetProgramiv(candidate, GL_INFO_LOG_LENGTH, &logLength);

            std::string log;
            log.resize(static_cast<std::size_t>(std::max(logLength, 1)));

            glGetProgramInfoLog(candidate, static_cast<GLsizei>(log.size()), nullptr, log.data());

            glDeleteProgram(candidate);

            throw std::runtime_error("OpenGL shader link error:\n" + log);
        }

        // Commit only after successful linking; a failed reload keeps the
        // working program and its location cache intact.
        GLint current = 0;
        glGetIntegerv(GL_CURRENT_PROGRAM, &current);
        if (program_ && static_cast<GLuint>(current) == program_)
            glUseProgram(candidate);
        if (program_) glDeleteProgram(program_);
        program_ = candidate;
        uniformLocations_.clear();
    }

    void GLShader::bind() const
    {
        if (!program_) throw std::logic_error("Cannot bind an empty GLShader");
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

    void GLShader::setMat3(const std::string& name, const glm::mat3& value) const
    {
        assertProgramBound(program_);
        glUniformMatrix3fv(uniformLocation(name), 1, GL_FALSE, glm::value_ptr(value));
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

        if (values.size() > static_cast<std::size_t>(std::numeric_limits<GLsizei>::max()))
            throw std::length_error("Matrix uniform array exceeds GLsizei");

        const GLint location = uniformLocation(name + "[0]");

        if (location < 0)
        {
            return;
        }

        const std::string arrayName = name + "[0]";
        const char* uniformName = arrayName.c_str();
        GLuint index = GL_INVALID_INDEX;
        glGetUniformIndices(program_, 1, &uniformName, &index);
        if (index != GL_INVALID_INDEX)
        {
            GLint count = 0;
            GLint type = 0;
            glGetActiveUniformsiv(program_, 1, &index, GL_UNIFORM_SIZE, &count);
            glGetActiveUniformsiv(program_, 1, &index, GL_UNIFORM_TYPE, &type);
            if (type != GL_FLOAT_MAT4 || values.size() > static_cast<std::size_t>(count))
                throw std::invalid_argument("Matrix array does not match shader uniform: " + name);
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
        if (!source || !*source)
            throw std::invalid_argument("Shader source must not be null or empty");
        const GLuint shader = glCreateShader(type);
        if (!shader) throw std::runtime_error("OpenGL failed to create a shader stage");

        glShaderSource(shader, 1, &source, nullptr);
        glCompileShader(shader);

        GLint success = 0;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);

        if (!success)
        {
            GLint logLength = 0;
            glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &logLength);

            std::string log;
            log.resize(static_cast<std::size_t>(std::max(logLength, 1)));

            glGetShaderInfoLog(shader, static_cast<GLsizei>(log.size()), nullptr, log.data());
            glDeleteShader(shader);

            throw std::runtime_error(std::string("OpenGL ") +
                (type == GL_VERTEX_SHADER ? "vertex" : "fragment") +
                " shader compile error:\n" + log);
        }

        return shader;
    }

    void GLShader::release()
    {
        if (program_ != 0)
        {
            GLint current = 0;
            glGetIntegerv(GL_CURRENT_PROGRAM, &current);
            if (static_cast<GLuint>(current) == program_) glUseProgram(0);
            glDeleteProgram(program_);
            program_ = 0;
        }

        // Locations are only valid for the program that produced them.
        uniformLocations_.clear();
    }
}
