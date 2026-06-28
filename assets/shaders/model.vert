#version 330 core

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
        mat4 accumulated = mat4(0.0);
        float totalWeight = 0.0;

        for (int i = 0; i < 4; ++i)
        {
            int boneId = aBoneIds[i];
            float weight = aBoneWeights[i];

            if (boneId >= 0 && boneId < MAX_BONES && weight > 0.0)
            {
                accumulated += uBoneMatrices[boneId] * weight;
                totalWeight += weight;
            }
        }

        // Renormalize by the weight that actually contributed, so vertices
        // whose influences were dropped (e.g. bone index >= MAX_BONES) are
        // still placed correctly instead of being scaled down. Vertices with
        // no valid influence fall back to identity instead of collapsing to
        // the origin (mat4(0.0) * position == 0).
        if (totalWeight > 0.0)
        {
            skinMatrix = accumulated / totalWeight;
        }
    }

    vec4 localPosition = skinMatrix * vec4(aPosition, 1.0);

    vec3 localNormal = aNormal;

    if (uUseSkinning)
    {
        localNormal = mat3(skinMatrix) * aNormal;
    }

    if (dot(localNormal, localNormal) < 0.000001)
    {
        localNormal = vec3(0.0, 1.0, 0.0);
    }

    localNormal = normalize(localNormal);

    vec4 worldPosition = uModel * localPosition;

    mat3 normalMatrix = transpose(inverse(mat3(uModel)));
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