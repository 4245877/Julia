#version 330 core

layout (location = 0) in vec3 aPosition;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoord;
layout (location = 3) in ivec4 aBoneIds;
layout (location = 4) in vec4 aBoneWeights;
layout (location = 5) in vec3 aOutlineNormal;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;
uniform mat3 uNormalMatrix;
uniform bool uUseSkinning;
uniform int uBoneCount;
uniform samplerBuffer uBonePalette;
uniform bool uOutlinePass;
uniform bool uShadowPass;
uniform mat4 uLightViewProjection;
uniform vec3 uViewport;
uniform float uOutlineWidthPixels;
uniform float uOutlineScale = 1.0;

out vec3 vWorldPosition;
out vec3 vNormal;
out vec3 vMaterialTangent;
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

    vec4 worldPosition = uModel * skin * vec4(aPosition, 1.0);
    vTexCoord = aTexCoord;
    if (uShadowPass)
    {
        // Depth casters do not need normal cofactors, outline transforms or
        // strand tangents. Keep the shared skinning/alpha path without paying
        // for the fill's vertex lighting work in this additional pass.
        gl_Position = uLightViewProjection * worldPosition;
        return;
    }

    vec3 localNormal = safeNormal(aNormal, vec3(0.0, 1.0, 0.0));
    vec3 outlineNormal = safeNormal(aOutlineNormal, localNormal);
    vSkinOrientation = 1.0;
    if (uUseSkinning)
    {
        mat3 linearSkin = mat3(skin);
        mat3 cofactor = mat3(cross(linearSkin[1], linearSkin[2]),
                             cross(linearSkin[2], linearSkin[0]),
                             cross(linearSkin[0], linearSkin[1]));
        float det = dot(linearSkin[0], cofactor[0]);
        vSkinOrientation = det < 0.0 ? -1.0 : 1.0;
        cofactor *= vSkinOrientation;
        localNormal = safeNormal(cofactor * localNormal, localNormal);
        outlineNormal = safeNormal(cofactor * outlineNormal, outlineNormal);
    }

    vec3 worldNormal = safeNormal(uNormalMatrix * localNormal, vec3(0.0, 1.0, 0.0));
    vWorldPosition = worldPosition.xyz;
    vNormal = worldNormal;
    // An object-space strand axis follows model and skeletal motion. Assets can
    // supply a dedicated groom/tangent map in a future material extension.
    vMaterialTangent = safeNormal(mat3(uModel) * mat3(skin) * vec3(0.0, 1.0, 0.0),
                                  vec3(0.0, 1.0, 0.0));
    vec4 clipPosition = uProjection * uView * worldPosition;
    if (uOutlinePass && uOutlineScale > 0.0 && clipPosition.w > 0.0)
    {
        vec3 worldOutline = safeNormal(uNormalMatrix * outlineNormal, worldNormal);
        vec3 viewNormal = mat3(uView) * worldOutline;
        vec4 projectedNormal = uProjection * vec4(viewNormal, 0.0);
        vec2 pixelDirection = (projectedNormal.xy * clipPosition.w -
                               clipPosition.xy * projectedNormal.w) * uViewport.xy;
        float directionLength = length(pixelDirection);
        if (directionLength > 1e-8)
        {
            // The same pixel width at every distance, FOV and aspect ratio.
            // A mild taper avoids thick marks on surfaces facing the camera.
            float taper = mix(1.0, 0.8, abs(viewNormal.z));
            clipPosition.xy += pixelDirection / directionLength *
                (2.0 * uOutlineWidthPixels * uOutlineScale * taper /
                 max(uViewport.xy, vec2(1.0))) * clipPosition.w;
        }
    }
    gl_Position = clipPosition;
}
