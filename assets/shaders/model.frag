#version 460 core

in vec3 vWorldPosition;
in vec3 vNormal;
in vec2 vTexCoord;

uniform mat4 uView;
uniform vec3 uDiffuseColor;
uniform sampler2D uDiffuseTexture;
uniform bool uHasDiffuseTexture;
uniform bool uOutlinePass;

out vec4 FragColor;

float softCel(float value, float edge, float softness)
{
    return smoothstep(edge - softness, edge + softness, value);
}

void main()
{
    if (uOutlinePass)
    {
        vec3 outlineColor = vec3(0.105, 0.095, 0.135);
        FragColor = vec4(outlineColor, 0.92);
        return;
    }

    vec3 normal = normalize(vNormal);

    vec3 viewPosition = inverse(uView)[3].xyz;
    vec3 viewDirection = normalize(viewPosition - vWorldPosition);

    vec3 textureColor = texture(uDiffuseTexture, vTexCoord).rgb;

    vec3 albedo = uHasDiffuseTexture
        ? textureColor * uDiffuseColor
        : uDiffuseColor;

    albedo = clamp(albedo, vec3(0.04), vec3(1.0));

    vec3 keyLightDirection = normalize(vec3(0.35, 0.88, 0.42));
    vec3 fillLightDirection = normalize(vec3(-0.70, 0.32, 0.22));

    float keyNdl = dot(normal, keyLightDirection);
    float fillNdl = dot(normal, fillLightDirection);

    // Half-Lambert даёт более мягкое anime-освещение.
    float keyWrapped = keyNdl * 0.5 + 0.5;
    float fillWrapped = fillNdl * 0.5 + 0.5;

    // fwidth делает границу cel-тени стабильнее на экране.
    float bandSoftness = max(0.045, fwidth(keyWrapped) * 2.5);

    float mainBand = softCel(keyWrapped, 0.54, bandSoftness);
    float brightBand = softCel(keyWrapped, 0.84, bandSoftness * 1.35);
    float fillBand = softCel(fillWrapped, 0.42, 0.16);

    vec3 shadowTint = vec3(0.60, 0.61, 0.82);
    vec3 midTint    = vec3(0.92, 0.88, 0.94);
    vec3 lightTint  = vec3(1.10, 1.04, 0.94);

    vec3 shadowColor = albedo * shadowTint * 0.58;
    vec3 midColor    = albedo * midTint    * 0.90;
    vec3 lightColor  = albedo * lightTint  * 1.06;

    vec3 color = mix(shadowColor, midColor, mainBand);
    color = mix(color, lightColor, brightBand * 0.55);

    // Заполняющий свет не создаёт второй жёсткой тени,
    // а только мягко вытаскивает форму из темноты.
    color += albedo * vec3(0.13, 0.12, 0.18) * fillBand;

    // Мягкий верхний свет.
    float topLight = smoothstep(-0.25, 0.85, normal.y);
    color += albedo * vec3(0.075, 0.060, 0.045) * topLight;

    // Аккуратный стилизованный блик.
    vec3 halfDirection = normalize(keyLightDirection + viewDirection);
    float specularRaw = pow(max(dot(normal, halfDirection), 0.0), 56.0);
    float specular = smoothstep(0.018, 0.070, specularRaw);
    specular *= smoothstep(0.30, 0.95, keyWrapped);

    color += vec3(1.0, 0.92, 0.78) * specular * 0.055;

    // Rim light мягкий, без грязного контура по всей модели.
    float rim = 1.0 - max(dot(normal, viewDirection), 0.0);
    rim = smoothstep(0.42, 0.92, rim);
    rim *= smoothstep(0.25, 0.95, keyWrapped + fillWrapped * 0.25);

    color += vec3(0.78, 0.75, 1.0) * rim * 0.145;

    // Немного насыщенности, но без кислотности.
    float luminance = dot(color, vec3(0.299, 0.587, 0.114));
    color = mix(vec3(luminance), color, 1.08);

    // Мягкая защита от пересвета.
    color = color / (color + vec3(0.18));
    color = pow(color, vec3(0.92));

    color = clamp(color, 0.0, 1.0);

    FragColor = vec4(color, 1.0);
}