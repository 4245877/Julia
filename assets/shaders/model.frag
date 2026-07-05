#version 330 core

in vec3 vWorldPosition;
in vec3 vNormal;
in vec2 vTexCoord;

uniform mat4 uView;
uniform vec3 uDiffuseColor;
uniform sampler2D uDiffuseTexture;
uniform bool uHasDiffuseTexture;
uniform bool uOutlinePass;
uniform float uLightResponse;
uniform float uSpecularStrength;
uniform float uRimStrength;

out vec4 FragColor;

// Жёсткий cel-срез: только узкая (~1 px) кромка сглаживания,
// никакого плавного ламбертовского градиента.
float celStep(float edge, float value)
{
    float aa = max(fwidth(value), 0.004);
    return smoothstep(edge - aa, edge + aa, value);
}

vec3 adjustSaturation(vec3 color, float amount)
{
    float luminance = dot(color, vec3(0.299, 0.587, 0.114));
    return mix(vec3(luminance), color, amount);
}

vec3 sampleAlbedo()
{
    vec3 albedo = max(uDiffuseColor, vec3(0.0));

    if (uHasDiffuseTexture)
    {
        albedo *= texture(uDiffuseTexture, vTexCoord).rgb;
    }

    return clamp(albedo, vec3(0.02), vec3(1.0));
}

void main()
{
    vec3 albedo = sampleAlbedo();

    if (uOutlinePass)
    {
        // Контур не чисто чёрный: тёмная, чуть перенасыщенная версия
        // базового цвета, как тушь поверх заливки в ZZZ.
        vec3 inkBase = vec3(0.045, 0.035, 0.065);
        vec3 lineColor = inkBase + adjustSaturation(albedo, 1.40) * albedo * 0.16;

        FragColor = vec4(lineColor, 1.0);
        return;
    }

    vec3 normal = normalize(vNormal);

    vec3 viewPosition = inverse(uView)[3].xyz;
    vec3 viewDirection = normalize(viewPosition - vWorldPosition);

    float response = clamp(uLightResponse, 0.0, 1.25);

    vec3 keyLightDirection = normalize(vec3(0.34, 0.78, 0.52));
    float ndl = dot(normal, keyLightDirection);

    // Один жёсткий срез свет/тень плюс узкая зона глубокой тени.
    // Материал с высоким lightResponse "открывается" свету раньше.
    float litEdge = 0.10 - response * 0.18;
    float lit = celStep(litEdge, ndl);
    float core = 1.0 - celStep(litEdge - 0.52, ndl);

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
    vec3 halfDirection = normalize(keyLightDirection + viewDirection);
    float specularShape = pow(max(dot(normal, halfDirection), 0.0), 140.0);
    float specular = celStep(0.5, specularShape) * lit;
    color += vec3(1.0, 0.98, 0.92) * specular * clamp(uSpecularStrength, 0.0, 2.0) * 0.40;

    // Лёгкий подъём насыщенности после cel-рампы; клемп вместо
    // тонмаппинга сохраняет плоские постеризованные поля.
    color = adjustSaturation(color, 1.10);
    color = clamp(color, 0.0, 1.0);

    FragColor = vec4(color, 1.0);
}
