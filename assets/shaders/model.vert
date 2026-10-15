#version 330 core

layout (location = 0) in vec3 aPosition;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoord;
layout (location = 3) in ivec4 aBoneIds;
layout (location = 4) in vec4 aBoneWeights;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;
// Computed once per draw on the CPU, not inverted once per vertex.
uniform mat3 uNormalMatrix;
uniform bool uUseSkinning;
uniform int uBoneCount;
// Four RGBA32F texels are the four columns of a column-major GLM matrix.
// Buffer textures are core in GL 3.3 and avoid the small default-uniform limit.
uniform samplerBuffer uBonePalette;
uniform bool uOutlinePass;
uniform vec3 uViewport;
uniform float uOutlineWidthPixels;

out vec3 vWorldPosition;
out vec3 vNormal;
out vec2 vTexCoord;
flat out float vSkinOrientation;

vec3 safeNormal(vec3 value, vec3 fallback)
{
    float squaredLength = dot(value, value);
    return squaredLength > 1e-20 ? value * inversesqrt(squaredLength) : fallback;
}

mat4 boneMatrix(int index)
{
    int offset = index * 4;
    return mat4(texelFetch(uBonePalette, offset),
                texelFetch(uBonePalette, offset + 1),
                texelFetch(uBonePalette, offset + 2),
                texelFetch(uBonePalette, offset + 3));
}

void main()
{
    mat4 skin = mat4(1.0);
    if (uUseSkinning)
    {
        mat4 accumulated = mat4(0.0);
        float totalWeight = 0.0;
        for (int i = 0; i < 4; ++i)
        {
            int id = aBoneIds[i];
            float weight = aBoneWeights[i];
            if (id >= 0 && id < uBoneCount && weight > 0.0 &&
                !isnan(weight) && !isinf(weight))
            {
                accumulated += boneMatrix(id) * weight;
                totalWeight += weight;
            }
        }
        if (totalWeight > 0.0) skin = accumulated / totalWeight;
    }

    vec3 localNormal = safeNormal(aNormal, vec3(0.0, 1.0, 0.0));
    vSkinOrientation = 1.0;
    if (uUseSkinning)
    {
        // Cofactors give inverse-transpose direction without an inverse/divide.
        // Multiplying by sign(det) handles reflections. For a collapsed blend,
        // use the surviving cofactor direction, or the input normal if rank < 2.
        mat3 linearSkin = mat3(skin);
        mat3 cofactor = mat3(cross(linearSkin[1], linearSkin[2]),
                             cross(linearSkin[2], linearSkin[0]),
                             cross(linearSkin[0], linearSkin[1]));
        float det = dot(linearSkin[0], cofactor[0]);
        vSkinOrientation = det < 0.0 ? -1.0 : 1.0;
        localNormal = safeNormal(cofactor * localNormal * (det < 0.0 ? -1.0 : 1.0),
                                 localNormal);
    }

    vec4 worldPosition = uModel * skin * vec4(aPosition, 1.0);
    vec3 worldNormal = safeNormal(uNormalMatrix * localNormal, vec3(0.0, 1.0, 0.0));
    vWorldPosition = worldPosition.xyz;
    vNormal = worldNormal;
    vTexCoord = aTexCoord;

    vec4 viewPosition = uView * worldPosition;
    vec4 clipPosition = uProjection * viewPosition;
    if (uOutlinePass && clipPosition.w > 0.0)
    {
        vec3 viewNormal = mat3(uView) * worldNormal;
        vec4 projectedNormal = uProjection * vec4(viewNormal, 0.0);
        // Derivative of clip.xy / clip.w in the normal direction. This also
        // accounts for off-axis perspective, unlike viewNormal.xy alone.
        vec2 pixelDirection = (projectedNormal.xy * clipPosition.w -
                               clipPosition.xy * projectedNormal.w) * uViewport.xy;
        float directionLength = length(pixelDirection);
        if (directionLength > 1e-8)
        {
            float taper = mix(1.1, 0.65, abs(viewNormal.z));
            clipPosition.xy += pixelDirection / directionLength *
                (2.0 * uOutlineWidthPixels * taper / max(uViewport.xy, vec2(1.0))) *
                clipPosition.w;
        }
    }
    gl_Position = clipPosition;
}
