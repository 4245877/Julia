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