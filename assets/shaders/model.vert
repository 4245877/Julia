#version 460 core

layout (location = 0) in vec3 aPosition;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoord;
layout (location = 3) in ivec4 aBoneIds;
layout (location = 4) in vec4 aBoneWeights;

const int MAX_BONES = 128;
const float OUTLINE_WIDTH = 0.0045;

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