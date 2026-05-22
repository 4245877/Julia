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
const float OUTLINE_WIDTH = 0.0065;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;

uniform bool uUseSkinning;
uniform bool uOutlinePass;
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

    vec3 localNormal = mat3(skinMatrix) * aNormal;
    if (dot(localNormal, localNormal) < 0.000001)
    {
        localNormal = aNormal;
    }

    vec4 worldPosition = uModel * localPosition;

    mat3 normalMatrix = mat3(transpose(inverse(uModel)));
    vec3 worldNormal = normalize(normalMatrix * localNormal);

    if (uOutlinePass)
    {
        worldPosition.xyz += worldNormal * OUTLINE_WIDTH;
    }

    vWorldPosition = worldPosition.xyz;
    vNormal = worldNormal;
    vTexCoord = aTexCoord;

    gl_Position = uProjection * uView * worldPosition;
}
)GLSL";

        constexpr const char* FragmentShaderSource = R"GLSL(
#version 460 core

in vec3 vWorldPosition;
in vec3 vNormal;
in vec2 vTexCoord;

uniform mat4 uView;
uniform vec3 uDiffuseColor;
uniform bool uOutlinePass;

out vec4 FragColor;

void main()
{
    if (uOutlinePass)
    {
        vec3 outlineColor = vec3(0.18, 0.17, 0.24);
        FragColor = vec4(outlineColor, 1.0);
        return;
    }

    vec3 normal = normalize(vNormal);

    vec3 lightDirection = normalize(vec3(-0.35, -0.90, -0.28));
    vec3 viewPosition = inverse(uView)[3].xyz;
    vec3 viewDirection = normalize(viewPosition - vWorldPosition);

    float ndl = dot(normal, -lightDirection);

    // Мягкий anime-свет без резкой cel-границы.
    float halfLambert = ndl * 0.5 + 0.5;
    float softLight = smoothstep(0.18, 0.88, halfLambert);

    // Нежные цветные тени: чуть холодные, сиренево-синие.
    vec3 shadowTint = vec3(0.64, 0.66, 0.92);
    vec3 lightTint = vec3(1.08, 1.03, 0.96);

    vec3 shadowColor = uDiffuseColor * shadowTint * 0.72;
    vec3 lightColor = uDiffuseColor * lightTint;

    vec3 color = mix(shadowColor, lightColor, softLight);

    // Очень мягкая дополнительная светлая зона.
    float softBloomLight = smoothstep(0.68, 1.0, halfLambert);
    color += uDiffuseColor * vec3(0.10, 0.08, 0.07) * softBloomLight;

    // Чистый нежный rim-light по краям формы.
    float rim = 1.0 - max(dot(normal, viewDirection), 0.0);
    rim = pow(rim, 2.8);
    rim *= smoothstep(0.05, 0.85, halfLambert);

    vec3 rimColor = vec3(0.86, 0.82, 1.0);
    color += rimColor * rim * 0.18;

    // Лёгкое осветление, чтобы картинка была мягче и чище.
    color = mix(color, vec3(1.0), 0.035);

    // Аккуратная защита от пересвета.
    color = clamp(color, 0.0, 1.0);

    FragColor = vec4(color, 1.0);
}
)GLSL";

        constexpr const char* BackgroundVertexShaderSource = R"GLSL(
#version 460 core

out vec2 vUv;

vec2 positions[3] = vec2[](
    vec2(-1.0, -1.0),
    vec2( 3.0, -1.0),
    vec2(-1.0,  3.0)
);

void main()
{
    vec2 p = positions[gl_VertexID];
    vUv = p * 0.5 + 0.5;
    gl_Position = vec4(p, 0.0, 1.0);
}
)GLSL";

        constexpr const char* BackgroundFragmentShaderSource = R"GLSL(
#version 460 core

in vec2 vUv;

uniform vec3 uTopColor;
uniform vec3 uBottomColor;

out vec4 FragColor;

void main()
{
    float t = smoothstep(0.0, 1.0, clamp(vUv.y, 0.0, 1.0));
    vec3 color = mix(uBottomColor, uTopColor, t);
    FragColor = vec4(color, 1.0);
}
)GLSL";
    }

    OpenGLRenderer::~OpenGLRenderer()
    {
        releaseBackground();
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

        backgroundShader_.createFromSource(
            BackgroundVertexShaderSource,
            BackgroundFragmentShaderSource
        );

        glGenVertexArrays(1, &backgroundVao_);

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

        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);

        backgroundShader_.bind();
        backgroundShader_.setVec3("uTopColor", glm::vec3{ 0.96f, 0.90f, 0.95f });
        backgroundShader_.setVec3("uBottomColor", glm::vec3{ 0.78f, 0.86f, 0.96f });

        glBindVertexArray(backgroundVao_);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glBindVertexArray(0);

        glDepthMask(GL_TRUE);
        glEnable(GL_DEPTH_TEST);
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

        // 1) Тонкий контур: рисуем слегка расширенную оболочку.
        shader_.setBool("uOutlinePass", true);

        glEnable(GL_CULL_FACE);
        glCullFace(GL_FRONT);
        glDepthMask(GL_FALSE);

        model.draw(shader_);

        glDepthMask(GL_TRUE);
        glCullFace(GL_BACK);
        glDisable(GL_CULL_FACE);

        // 2) Основная мягкая anime-заливка.
        shader_.setBool("uOutlinePass", false);

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

        const glm::vec3 cameraPosition{ 0.0f, 1.25f, 3.6f };
        const glm::vec3 cameraTarget{ 0.0f, 1.15f, 0.0f };
        const glm::vec3 cameraUp{ 0.0f, 1.0f, 0.0f };

        view_ = glm::lookAt(cameraPosition, cameraTarget, cameraUp);
    }

    void OpenGLRenderer::releaseBackground()
    {
        if (backgroundVao_ != 0)
        {
            glDeleteVertexArrays(1, &backgroundVao_);
            backgroundVao_ = 0;
        }
    }
}