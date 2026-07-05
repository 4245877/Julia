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

float softCel(float value, float edge, float softness)
{
    return smoothstep(edge - softness, edge + softness, value);
}

vec3 srgbToLinear(vec3 color)
{
    return pow(clamp(color, 0.0, 1.0), vec3(2.2));
}

vec3 toneMap(vec3 color)
{
    color = max(color, vec3(0.0));
    color *= 1.05;
    color = color / (vec3(1.0) + color * 0.62);
    return clamp(color, 0.0, 0.97);
}

void main()
{
    if (uOutlinePass)
    {
        vec3 outlineColor = srgbToLinear(vec3(0.105, 0.095, 0.135));
        FragColor = vec4(outlineColor, 0.92);
        return;
    }

    vec3 normal = normalize(vNormal);

    vec3 viewPosition = inverse(uView)[3].xyz;
    vec3 viewDirection = normalize(viewPosition - vWorldPosition);

    vec3 albedo = max(uDiffuseColor, vec3(0.0));

    if (uHasDiffuseTexture)
    {
        albedo *= texture(uDiffuseTexture, vTexCoord).rgb;
    }

    albedo = clamp(albedo, vec3(0.025), vec3(0.96));

    float albedoLuminance = dot(albedo, vec3(0.2126, 0.7152, 0.0722));
    float brightAlbedo = smoothstep(0.56, 0.86, albedoLuminance);
    float detailGuard = mix(1.0, 0.54, brightAlbedo);
    float lightResponse = clamp(uLightResponse, 0.0, 1.25);

    vec3 keyLightDirection = normalize(vec3(0.28, 0.84, 0.38));
    vec3 fillLightDirection = normalize(vec3(-0.62, 0.25, 0.18));

    float keyNdl = dot(normal, keyLightDirection);
    float fillNdl = dot(normal, fillLightDirection);

    // Half-Lambert даёт более мягкое anime-освещение.
    float keyWrapped = keyNdl * 0.5 + 0.5;
    float fillWrapped = fillNdl * 0.5 + 0.5;

    // fwidth делает границу cel-тени стабильнее на экране.
    float bandSoftness = max(0.045, fwidth(keyWrapped) * 2.5);

    float mainBand = softCel(keyWrapped, 0.54, bandSoftness);
    float brightBand = softCel(keyWrapped, 0.86, bandSoftness * 1.25);
    float fillBand = softCel(fillWrapped, 0.43, 0.18);

    vec3 shadowTint = vec3(0.58, 0.60, 0.78);
    vec3 midTint    = vec3(0.88, 0.84, 0.90);
    vec3 lightTint  = vec3(0.96, 0.91, 0.84);

    vec3 shadowColor = albedo * shadowTint * 0.46;
    vec3 midColor    = albedo * midTint    * 0.74;
    vec3 lightColor  = albedo * lightTint  * 0.88;

    vec3 color = mix(shadowColor, midColor, mainBand);
    color = mix(color, lightColor, brightBand * 0.42 * lightResponse * detailGuard);

    // Заполняющий свет не создаёт второй жёсткой тени,
    // а только мягко вытаскивает форму из темноты.
    color += albedo * vec3(0.065, 0.058, 0.085) *
        fillBand * lightResponse * detailGuard;

    // Мягкий верхний свет.
    float topLight = smoothstep(-0.20, 0.86, normal.y);
    color += albedo * vec3(0.036, 0.030, 0.024) *
        topLight * lightResponse * detailGuard;

    // Аккуратный стилизованный блик.
    vec3 halfDirection = normalize(keyLightDirection + viewDirection);
    float specularRaw = pow(max(dot(normal, halfDirection), 0.0), 64.0);
    float specular = smoothstep(0.025, 0.110, specularRaw);
    specular *= smoothstep(0.35, 0.95, keyWrapped);

    float specularGuard = mix(1.0, 0.28, brightAlbedo);
    color += vec3(1.0, 0.88, 0.68) *
        specular * max(uSpecularStrength, 0.0) * specularGuard;

    // Rim light мягкий, без грязного контура по всей модели.
    float rim = 1.0 - max(dot(normal, viewDirection), 0.0);
    rim = smoothstep(0.48, 0.94, rim);
    rim *= smoothstep(0.32, 0.96, keyWrapped + fillWrapped * 0.20);

    float rimGuard = mix(1.0, 0.45, brightAlbedo);
    color += vec3(0.58, 0.56, 0.84) *
        rim * max(uRimStrength, 0.0) * rimGuard;

    // Немного насыщенности, но без кислотности.
    float luminance = dot(color, vec3(0.299, 0.587, 0.114));
    color = mix(vec3(luminance), color, 1.02);

    // Мягкая защита от пересвета.
    color = toneMap(color);

    FragColor = vec4(color, 1.0);
}
