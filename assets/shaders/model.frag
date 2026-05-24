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
        vec3 outlineColor = vec3(0.135, 0.120, 0.165);
        FragColor = vec4(outlineColor, 0.88);
        return;
    }

    vec3 normal = normalize(vNormal);

    vec3 viewPosition = inverse(uView)[3].xyz;
    vec3 viewDirection = normalize(viewPosition - vWorldPosition);

    // Базовый цвет материала из FBX.
    // Делаем защиту от слишком белых/плоских материалов.
    vec3 albedo = clamp(uDiffuseColor, vec3(0.04), vec3(1.0));

    // Основной тёплый источник света спереди-сверху.
    vec3 keyLightDirection = normalize(vec3(0.35, 0.90, 0.45));

    // Мягкий дополнительный заполняющий свет.
    vec3 fillLightDirection = normalize(vec3(-0.75, 0.35, 0.20));

    float keyNdl = max(dot(normal, keyLightDirection), 0.0);
    float fillNdl = max(dot(normal, fillLightDirection), 0.0);

    // Мягкая anime-градация без грязной резкой границы.
    float keySoft = smoothstep(0.08, 0.78, keyNdl);
    float fillSoft = smoothstep(0.00, 0.70, fillNdl);

    vec3 shadowTint = vec3(0.58, 0.60, 0.82);
    vec3 midTint    = vec3(0.90, 0.86, 0.92);
    vec3 lightTint  = vec3(1.10, 1.04, 0.94);

    vec3 shadowColor = albedo * shadowTint * 0.55;
    vec3 midColor    = albedo * midTint    * 0.88;
    vec3 lightColor  = albedo * lightTint  * 1.05;

    vec3 color = mix(shadowColor, midColor, fillSoft * 0.55);
    color = mix(color, lightColor, keySoft);

    // Лёгкий верхний свет, чтобы лицо, волосы и плечи читались лучше.
    float topLight = smoothstep(-0.20, 0.85, normal.y);
    color += albedo * vec3(0.08, 0.065, 0.045) * topLight;

    // Аккуратный блик, чтобы модель не была пластиковой, но имела объём.
    vec3 halfDirection = normalize(keyLightDirection + viewDirection);
    float specular = pow(max(dot(normal, halfDirection), 0.0), 42.0);
    specular *= smoothstep(0.30, 1.0, keyNdl);
    color += vec3(1.0, 0.92, 0.78) * specular * 0.055;

    // Rim light помогает отделить волосы, руки и юбку от фона.
    float rim = 1.0 - max(dot(normal, viewDirection), 0.0);
    rim = pow(rim, 2.35);
    rim *= smoothstep(0.05, 0.85, keyNdl + fillNdl * 0.35);

    vec3 rimColor = vec3(0.82, 0.78, 1.0);
    color += rimColor * rim * 0.20;

    // Небольшое усиление насыщенности, чтобы материалы меньше сливались.
    float luminance = dot(color, vec3(0.299, 0.587, 0.114));
    color = mix(vec3(luminance), color, 1.10);

    // Не выбеливаем модель слишком сильно.
    color = mix(color, vec3(1.0), 0.012);

    color = clamp(color, 0.0, 1.0);

    FragColor = vec4(color, 1.0);
}