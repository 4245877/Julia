#version 330 core

in vec3 vWorldPosition;
in vec3 vNormal;
in vec2 vTexCoord;
flat in float vSkinOrientation;

uniform vec3 uViewPos;
uniform vec3 uDiffuseColor;
uniform sampler2D uDiffuseTexture;
uniform bool uHasDiffuseTexture;
uniform bool uOutlinePass;
uniform bool uUseSkinning;
uniform float uLightResponse;
uniform float uSpecularStrength;
uniform float uRimStrength;
uniform float uOpacity;
uniform float uAlphaCutoff;
// Must match AlphaMode in GLMesh.hpp: Opaque=0, Mask=1, Blend=2.
uniform int uAlphaMode;

out vec4 FragColor;

// A small normal-space transition keeps the toon bands stable at close range;
// screen-space derivatives still antialias them when the model is far away.
// Only lighting is filtered, never albedo, UVs, geometry or material boundaries.
float celStep(float edge, float value, float halfWidth)
{
    float aa = max(fwidth(value), halfWidth);
    return smoothstep(edge - aa, edge + aa, value);
}

float celStep(float edge, float value)
{
    return celStep(edge, value, 0.004);
}

vec3 adjustSaturation(vec3 color, float amount)
{
    float luminance = dot(color, vec3(0.299, 0.587, 0.114));
    return mix(vec3(luminance), color, amount);
}

vec4 sampleAlbedo()
{
    // Material factors are linear; GL_SRGB8_ALPHA8 decodes texture RGB only.
    vec4 albedo = vec4(max(uDiffuseColor, vec3(0.0)), clamp(uOpacity, 0.0, 1.0));

    if (uHasDiffuseTexture)
    {
        albedo *= texture(uDiffuseTexture, vTexCoord);
    }

    return clamp(albedo, 0.0, 1.0);
}

void main()
{
    // Skeletal reflections can reverse winding independently of uModel.
    bool frontFacing = gl_FrontFacing == (vSkinOrientation >= 0.0);
    if (uOutlinePass && uUseSkinning && frontFacing) discard;
    vec4 base = sampleAlbedo();
    if (uAlphaMode == 1 && base.a < uAlphaCutoff) discard;
    if (uAlphaMode == 2 && base.a <= 0.0) discard;
    float alpha = uAlphaMode == 2 ? base.a : 1.0;
    vec3 albedo = base.rgb;

    if (uOutlinePass)
    {
        // Контур не чисто чёрный: тёмная, чуть перенасыщенная версия
        // базового цвета, как тушь поверх заливки в ZZZ.
        vec3 inkBase = vec3(0.045, 0.035, 0.065);
        vec3 lineColor = inkBase + adjustSaturation(albedo, 1.40) * albedo * 0.16;

        FragColor = vec4(clamp(lineColor, 0.0, 1.0), alpha);
        return;
    }

    float normalLength = dot(vNormal, vNormal);
    vec3 normal = normalLength > 1e-20 ? vNormal * inversesqrt(normalLength) : vec3(0.0, 1.0, 0.0);
    if (!frontFacing) normal = -normal;

    vec3 toView = uViewPos - vWorldPosition;
    vec3 viewDirection = dot(toView, toView) > 1e-20 ? normalize(toView) : normal;

    float response = clamp(uLightResponse, 0.0, 1.25);

    vec3 keyLightDirection = normalize(vec3(0.34, 0.78, 0.52));
    float ndl = dot(normal, keyLightDirection);

    // Two toon bands with a narrow, resolution-independent light transition.
    // Материал с высоким lightResponse "открывается" свету раньше.
    float litEdge = 0.10 - response * 0.18;
    float lit = celStep(litEdge, ndl, 0.025);
    float core = 1.0 - celStep(litEdge - 0.52, ndl, 0.025);

    // Тень уводится в холодный сиренево-синий с лёгким подъёмом
    // насыщенности — фирменный приём ZZZ вместо простого затемнения.
    vec3 litColor = albedo * vec3(1.05, 1.01, 0.94);
    vec3 shadowColor = adjustSaturation(albedo, 1.15) * vec3(0.58, 0.55, 0.74);
    vec3 coreColor = adjustSaturation(albedo, 1.20) * vec3(0.38, 0.33, 0.55);

    vec3 color = mix(shadowColor, litColor, lit);
    color = mix(color, coreColor, core * (1.0 - lit));

    // Ступенчатый отражённый свет снизу, чтобы теневая масса
    // не превращалась в плоское пятно. Тоже жёсткий срез, не градиент.
    vec3 bounceDirection = normalize(vec3(-0.60, -0.20, 0.35));
    float bounce = celStep(0.25, dot(normal, bounceDirection)) * (1.0 - lit);
    color += albedo * vec3(0.05, 0.06, 0.10) * bounce;

    float rimStrength = clamp(uRimStrength, 0.0, 2.0);
    float fresnel = 1.0 - max(dot(normal, viewDirection), 0.0);

    // Яркий жёсткий рим по кромке на световой стороне —
    // главный акцент вместо россыпи бликов.
    float rimLit = celStep(0.68, fresnel) * celStep(-0.05, ndl);
    color += vec3(1.0, 0.97, 0.90) * rimLit * rimStrength * 0.55;

    // Тонкий холодный отсвет на теневой кромке отделяет силуэт от фона.
    float rimShadow = celStep(0.80, fresnel) * (1.0 - lit);
    color += vec3(0.30, 0.38, 0.72) * rimShadow * rimStrength * 0.35;

    // Единственный блик: маленький, ступенчатый, только в световой зоне.
    vec3 halfSum = keyLightDirection + viewDirection;
    vec3 halfDirection = dot(halfSum, halfSum) > 1e-20 ? normalize(halfSum) : normal;
    float specularShape = pow(max(dot(normal, halfDirection), 0.0), 140.0);
    float specular = celStep(0.5, specularShape) * lit;
    color += vec3(1.0, 0.98, 0.92) * specular * clamp(uSpecularStrength, 0.0, 2.0) * 0.40;

    // Лёгкий подъём насыщенности после cel-рампы; клемп вместо
    // тонмаппинга сохраняет плоские постеризованные поля.
    color = adjustSaturation(color, 1.10);
    color = clamp(color, 0.0, 1.0);

    FragColor = vec4(color, alpha);
}
