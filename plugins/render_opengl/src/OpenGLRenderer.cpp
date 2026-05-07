#include "julia/render_opengl/OpenGLRenderer.hpp"

#include <glad/glad.h>

#include <algorithm>
#include <array>

#include <glm/gtc/matrix_transform.hpp>

namespace julia::render_opengl
{
    namespace
    {
        constexpr int MaxBones = 128;

        constexpr const char* VertexShaderSource = R"GLSL(
#version 460 core

layout (location = 0) in vec3 aPosition;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoord;
layout (location = 3) in ivec4 aBoneIds;
layout (location = 4) in vec4 aBoneWeights;

const int MAX_BONES = 128;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;

uniform bool uUseSkinning;
uniform mat4 uBoneMatrices[MAX_BONES];

out vec3 vWorldPosition;
out vec3 vNormal;
out vec2 vTexCoord;

void main()
{
    mat4 skinMatrix = mat4(1.0);

    if (uUseSkinning)
    {
        skinMatrix = mat4(0.0);

        for (int i = 0; i < 4; ++i)
        {
            int boneId = aBoneIds[i];
            float weight = aBoneWeights[i];

            if (boneId >= 0 && boneId < MAX_BONES && weight > 0.0)
            {
                skinMatrix += uBoneMatrices[boneId] * weight;
            }
        }
    }

    vec4 localPosition = skinMatrix * vec4(aPosition, 1.0);
    vec4 worldPosition = uModel * localPosition;

    vWorldPosition = worldPosition.xyz;

    mat3 normalMatrix = mat3(transpose(inverse(uModel)));
    vNormal = normalize(normalMatrix * aNormal);

    vTexCoord = aTexCoord;

    gl_Position = uProjection * uView * worldPosition;
}
)GLSL";

        constexpr const char* FragmentShaderSource = R"GLSL(
#version 460 core

in vec3 vWorldPosition;
in vec3 vNormal;
in vec2 vTexCoord;

uniform vec3 uDiffuseColor;

out vec4 FragColor;

void main()
{
    vec3 normal = normalize(vNormal);

    vec3 lightDirection = normalize(vec3(-0.4, -1.0, -0.3));
    float diffuse = max(dot(normal, -lightDirection), 0.0);

    vec3 ambientColor = uDiffuseColor * 0.28;
    vec3 diffuseColor = uDiffuseColor * diffuse * 0.85;

    vec3 finalColor = ambientColor + diffuseColor;

    FragColor = vec4(finalColor, 1.0);
}
)GLSL";
    }

    void OpenGLRenderer::initialize(int width, int height)
    {
        width_ = width;
        height_ = height;

        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);

        // Пока culling лучше не включать:
        // у некоторых FBX бывают неожиданные winding/axis.
        glDisable(GL_CULL_FACE);

        shader_.createFromSource(VertexShaderSource, FragmentShaderSource);

        resize(width_, height_);
    }

    void OpenGLRenderer::resize(int width, int height)
    {
        width_ = std::max(width, 1);
        height_ = std::max(height, 1);

        glViewport(0, 0, width_, height_);

        updateCamera();
    }

    void OpenGLRenderer::beginFrame(const glm::vec3& clearColor)
    {
        glClearColor(clearColor.r, clearColor.g, clearColor.b, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

    void OpenGLRenderer::renderModel(
        const GLModel& model,
        const glm::mat4& modelMatrix,
        std::span<const glm::mat4> boneMatrices
    )
    {
        shader_.bind();

        shader_.setMat4("uModel", modelMatrix);
        shader_.setMat4("uView", view_);
        shader_.setMat4("uProjection", projection_);

        const bool useSkinning = !boneMatrices.empty();

        shader_.setBool("uUseSkinning", useSkinning);

        if (useSkinning)
        {
            const std::size_t count = std::min<std::size_t>(boneMatrices.size(), MaxBones);
            shader_.setMat4Array("uBoneMatrices", boneMatrices.subspan(0, count));
        }

        model.draw(shader_);
    }

    void OpenGLRenderer::updateCamera()
    {
        const float aspect =
            static_cast<float>(width_) /
            static_cast<float>(std::max(height_, 1));

        projection_ = glm::perspective(
            glm::radians(45.0f),
            aspect,
            0.01f,
            100.0f
        );

        const glm::vec3 cameraPosition{0.0f, 1.35f, 4.0f};
        const glm::vec3 cameraTarget{0.0f, 1.1f, 0.0f};
        const glm::vec3 cameraUp{0.0f, 1.0f, 0.0f};

        view_ = glm::lookAt(cameraPosition, cameraTarget, cameraUp);
    }
}