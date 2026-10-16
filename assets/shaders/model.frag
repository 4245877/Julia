#version 330 core

in vec3 vWorldPosition;
in vec3 vNormal;
in vec3 vMaterialTangent;
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
uniform float uSpecularPower = 80.0;
uniform float uRimStrength;
uniform float uOutlineScale = 1.0;
// Generic=0, Skin=1, Hair=2, Eye=3, Metal=4, Fabric=5, Leather=6, Ink=7.
uniform int uMaterialKind;
uniform float uOpacity;
uniform float uAlphaCutoff;
uniform int uAlphaMode;

uniform vec3 uKeyLightDirection = vec3(0.34, 0.78, 0.52);
uniform vec3 uKeyLightColor = vec3(1.05, 1.01, 0.94);
uniform vec3 uShadowTint = vec3(0.62, 0.60, 0.74);
uniform bool uShadowPass;
uniform bool uReceiveShadows;
uniform sampler2DShadow uShadowMap;
uniform mat4 uLightViewProjection;
// inverse map size, world units per shadow texel, inverse depth range.
uniform vec3 uShadowParams;
uniform float uShadowStrength = 0.75;

out vec4 FragColor;

vec3 safeNormal(vec3 value, vec3 fallback)
{
    float squaredLength = dot(value, value);
    return squaredLength > 1e-20 ? value * inversesqrt(squaredLength) : fallback;
}

float celStep(float edge, float value, float halfWidth)
{
    float aa = max(fwidth(value) * 0.75, halfWidth);
    return smoothstep(edge - aa, edge + aa, value);
}

vec4 sampleAlbedo()
{
    // Both the texture decode and framebuffer encode are handled by sRGB GL
    // formats. Material factors and all lighting calculations remain linear.
    vec4 albedo = vec4(max(uDiffuseColor, vec3(0.0)), clamp(uOpacity, 0.0, 1.0));
    if (uHasDiffuseTexture) albedo *= texture(uDiffuseTexture, vTexCoord);
    return clamp(albedo, 0.0, 1.0);
}

void alphaTest(float alpha)
{
    if (uAlphaMode == 1 && alpha < uAlphaCutoff) discard;
    if (uAlphaMode == 2 && alpha <= 0.0) discard;
}

float shadowVisibility()
{
    if (!uReceiveShadows) return 1.0;
    vec4 position = uLightViewProjection * vec4(vWorldPosition, 1.0);
    vec3 coord = position.xyz / position.w * 0.5 + 0.5;
    // Receiver-plane depth correction: every PCF tap compares against the
    // receiver at that tap, rather than the depth at the kernel center. A
    // constant bias alone produces stripes on sloping faces and hair cards.
    // Evaluate derivatives before the nonuniform frustum/alpha tests.
    vec3 dx = dFdx(coord);
    vec3 dy = dFdy(coord);
    float determinant = dx.x * dy.y - dx.y * dy.x;
    vec2 gradient = vec2(0.0);
    if (abs(determinant) > 1e-12)
        gradient = vec2(dy.y * dx.z - dx.y * dy.z, dx.x * dy.z - dy.x * dx.z) / determinant;
    // A bilinear tap can reach almost a whole texel to the farther of its two
    // centers. Cover that footprint as well, with a small precision margin.
    float bias = min(dot(abs(gradient), vec2(uShadowParams.x * 1.05)), 0.02) +
        uShadowParams.y * uShadowParams.z * 0.35 + 1.2e-7;
    if (any(lessThan(coord, vec3(0.0))) || any(greaterThan(coord, vec3(1.0)))) return 1.0;
    float visibility = 0.0;
    for (int y = -1; y <= 1; ++y)
        for (int x = -1; x <= 1; ++x)
        {
            vec2 offset = vec2(x, y) * uShadowParams.x;
            visibility += texture(uShadowMap, vec3(coord.xy + offset, coord.z + dot(gradient, offset) - bias));
        }
    return visibility / 9.0;
}

void main()
{
    // The depth pass shares skinning, UVs and the alpha cutout with the fill.
    // Opaque casters need no albedo fetch or lighting work.
    if (uShadowPass)
    {
        if (uAlphaMode == 1) alphaTest(sampleAlbedo().a);
        FragColor = vec4(0.0);
        return;
    }

    bool frontFacing = gl_FrontFacing == (vSkinOrientation >= 0.0);
    vec4 base = sampleAlbedo();
    float alpha = uAlphaMode == 2 ? base.a : 1.0;
    vec3 albedo = base.rgb;
    if (uOutlinePass)
    {
        if (uOutlineScale <= 0.0 || (uUseSkinning && frontFacing)) discard;
        alphaTest(base.a);
        // Colored ink stays dark without squaring albedo or clipping its hue.
        FragColor = vec4(vec3(0.012, 0.010, 0.018) + albedo * 0.065, alpha);
        return;
    }

    vec3 normal = safeNormal(vNormal, vec3(0.0, 1.0, 0.0));
    if (!frontFacing) normal = -normal;
    vec3 viewDirection = safeNormal(uViewPos - vWorldPosition, normal);
    vec3 lightDirection = safeNormal(uKeyLightDirection, normalize(vec3(0.34, 0.78, 0.52)));
    float ndl = dot(normal, lightDirection);
    float response = clamp(uLightResponse, 0.0, 1.25);
    float edge = 0.10 - response * 0.18;
    float lit = celStep(edge, ndl, uMaterialKind == 1 ? 0.04 : 0.025);
    float core = 1.0 - celStep(edge - 0.52, ndl, 0.035);

    vec3 tint = uShadowTint;
    float coreStrength = 0.16;
    if (uMaterialKind == 1)
    {
        // Warm, readable skin shadows; avoid a dark purple core on cheeks.
        tint = mix(tint, vec3(0.80, 0.59, 0.61), 0.65);
        coreStrength = 0.06;
    }
    else if (uMaterialKind == 2) tint *= vec3(1.02, 0.98, 1.04);
    else if (uMaterialKind == 3) { tint = mix(tint, vec3(0.80), 0.65); coreStrength = 0.04; }

    float visibility = shadowVisibility();
    float castVisibility = mix(1.0, visibility, clamp(uShadowStrength, 0.0, 1.0));
    float direct = lit * castVisibility;
    vec3 shadowColor = albedo * tint * (1.0 - core * coreStrength);
    vec3 color = mix(shadowColor, albedo * uKeyLightColor, direct);
    // Broad, low-amplitude environmental fill keeps the shadow mass readable.
    float bounce = celStep(0.25, dot(normal, normalize(vec3(-0.60, -0.20, 0.35))), 0.025);
    color += albedo * vec3(0.025, 0.03, 0.045) * bounce * (1.0 - direct);

    vec3 halfSum = lightDirection + viewDirection;
    float halfLength = dot(halfSum, halfSum);
    vec3 halfDirection = safeNormal(halfSum, normal);
    float ndh = max(dot(normal, halfDirection), 0.0);
    // Invert the old pow threshold: filtering N.H directly is cheaper and
    // remains stable when a very narrow lobe becomes smaller than a pixel.
    float specEdge = exp2(-1.0 / clamp(uSpecularPower, 2.0, 512.0));
    float specular = celStep(specEdge, ndh, 0.0015);
    if (uMaterialKind == 2)
    {
        vec3 tangent = vMaterialTangent - normal * dot(vMaterialTangent, normal);
        float tangentLength = dot(tangent, tangent);
        vec3 fallback = abs(normal.y) < 0.95 ? cross(normal, vec3(0.0, 1.0, 0.0)) :
                                                            cross(normal, vec3(1.0, 0.0, 0.0));
        tangent = safeNormal(tangent, safeNormal(fallback, vec3(1.0, 0.0, 0.0)));
        float th = dot(tangent, halfDirection);
        float strandLobe = sqrt(max(1.0 - th * th, 0.0));
        float strandSpecular = celStep(specEdge, strandLobe, 0.003) * celStep(0.80, ndh, 0.04);
        // The strand direction vanishes on caps aligned with the axis. Fade
        // into the isotropic lobe there instead of exposing a fallback seam.
        specular = mix(specular, strandSpecular, smoothstep(0.01, 0.09, tangentLength));
    }
    specular *= lit * castVisibility * (halfLength > 1e-12 ? 1.0 : 0.0);
    vec3 specTint = uMaterialKind == 4 ? mix(vec3(1.0), albedo, 0.55) : vec3(1.0, 0.98, 0.94);
    if (uMaterialKind == 2) specTint = mix(specTint, sqrt(albedo), 0.45);
    // Blend toward the highlight color to retain detail on bright materials.
    // This avoids an additive hard clamp erasing the texture near a glint.
    float highlight = clamp(specular * uSpecularStrength, 0.0, 0.95);
    color = mix(color, max(color, specTint * uKeyLightColor), highlight);

    float fresnel = 1.0 - max(dot(normal, viewDirection), 0.0);
    float rim = celStep(0.76, fresnel, 0.02) * celStep(0.0, ndl, 0.04);
    float rimShadow = celStep(0.88, fresnel, 0.02) * (1.0 - lit);
    color += (vec3(1.0, 0.97, 0.90) * rim * castVisibility * 0.35 +
              vec3(0.30, 0.38, 0.62) * rimShadow * 0.18) * clamp(uRimStrength, 0.0, 2.0);

    if (uMaterialKind == 7) color = albedo; // Authored brows/lashes retain their ink value.
    // Keep derivative evaluation above nonuniform alpha discard, including at
    // hair-card boundaries, where derivatives after discard are undefined.
    alphaTest(base.a);
    FragColor = vec4(clamp(color, 0.0, 1.0), alpha);
}
