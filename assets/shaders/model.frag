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

float inkStep(float edge, float value)
{
    float width = max(fwidth(value) * 1.35, 0.006);
    return smoothstep(edge - width, edge + width, value);
}

float inkLine(float value, float center, float width)
{
    float distanceToLine = abs(value - center);
    float aa = max(fwidth(value) * 1.80, 0.006);
    return 1.0 - smoothstep(width, width + aa, distanceToLine);
}

void main()
{
    if (uOutlinePass)
    {
        FragColor = vec4(0.012, 0.010, 0.018, 1.0);
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

    albedo = clamp(albedo, vec3(0.025), vec3(0.98));

    float response = clamp(uLightResponse, 0.0, 1.25);

    vec3 keyLightDirection = normalize(vec3(0.18, 0.86, 0.48));
    vec3 fillLightDirection = normalize(vec3(-0.72, 0.18, 0.22));

    float keyNdl = dot(normal, keyLightDirection);
    float fillNdl = dot(normal, fillLightDirection);

    // Thresholded value: no Lambert gradient, only large readable fields.
    float celValue = keyNdl + normal.y * 0.14 + response * 0.16 - 0.14;

    // Keep only a narrow antialiasing fringe at each hard cut.
    float midBand = inkStep(-0.28, celValue);
    float lightBand = inkStep(0.24, celValue);

    vec3 shadowTint = vec3(0.46, 0.48, 0.66);
    vec3 midTint = vec3(0.74, 0.72, 0.82);
    vec3 lightTint = vec3(1.02, 0.94, 0.80);

    vec3 shadowColor = albedo * shadowTint * 0.58;
    vec3 midColor = albedo * midTint * 0.82;
    vec3 lightColor = albedo * lightTint * 1.02;

    vec3 color = mix(shadowColor, midColor, midBand);
    color = mix(color, lightColor, lightBand);

    // The fill is a cel-shaped bounce patch, not a second smooth light.
    float bounceBand = inkStep(0.10, fillNdl + 0.12) * (1.0 - lightBand);
    color += albedo * vec3(0.025, 0.030, 0.052) * bounceBand * response;

    // A thin terminator and an inner contour sharpen the painted shadow mass.
    float terminator = inkLine(celValue, -0.28, 0.020) * (1.0 - lightBand);
    vec3 inkColor = vec3(0.018, 0.014, 0.026);
    color = mix(color, inkColor, terminator * 0.14);

    float viewRim = 1.0 - max(dot(normal, viewDirection), 0.0);
    float innerInk = inkStep(0.74, viewRim) * (1.0 - lightBand * 0.55);
    color = mix(color, inkColor, innerInk * 0.20);

    // Sparse blocky highlight. Most surfaces stay matte and illustrated.
    vec3 halfDirection = normalize(keyLightDirection + viewDirection);
    float specularShape = pow(max(dot(normal, halfDirection), 0.0), 96.0);
    float specular = inkStep(0.20, specularShape) * lightBand;
    float specularStrength = max(uSpecularStrength, 0.0) * 0.58;
    color += vec3(1.0, 0.86, 0.58) * specular * specularStrength;

    // Cool edge accent is weak; the black contour remains dominant.
    float coolEdge = inkStep(0.82, viewRim) * inkStep(-0.10, fillNdl);
    color += vec3(0.18, 0.22, 0.42) * coolEdge * max(uRimStrength, 0.0) * 0.46;

    // Slight saturation lift after the cel ramp.
    float luminance = dot(color, vec3(0.299, 0.587, 0.114));
    color = mix(vec3(luminance), color, 1.08);

    // Final clamp instead of tone mapping keeps the posterized bands intact.
    color = clamp(color, 0.0, 1.0);

    FragColor = vec4(color, 1.0);
}
