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
uniform float uShadowSoftness = 0.065;
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

float paintedLobe(float cosine, float power, float normalVariance)
{
    // Filter the angular footprint before thresholding, retaining the lobe's
    // energy as it widens. Subpixel highlights fade instead of flashing white
    // when a coarse triangle or a distant strand crosses the light direction.
    float width = 1.0 - exp2(-1.0 / clamp(power, 2.0, 512.0));
    float filteredWidth = width + min(normalVariance, 0.08);
    return celStep(1.0 - filteredWidth, cosine, max(width * 0.22, 0.0005)) *
           width / filteredWidth;
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
    // A smooth skin surface and its coarse depth triangles have different
    // terminators. Offset the receiver a few world-space shadow texels along
    // the lighting normal to avoid triangular self-shadow patches. Orient
    // towards the light, never the camera (including double-sided surfaces).
    vec3 receiver = vWorldPosition;
    if (uMaterialKind == 1)
    {
        vec3 surfaceNormal = safeNormal(vNormal, vec3(0.0, 1.0, 0.0));
        if (dot(surfaceNormal, uKeyLightDirection) < 0.0) surfaceNormal = -surfaceNormal;
        receiver += surfaceNormal * uShadowParams.y * 2.5;
    }
    vec4 position = uLightViewProjection * vec4(receiver, 1.0);
    vec3 coord = position.xyz / position.w * 0.5 + 0.5;
    // Receiver-plane depth correction: every PCF tap compares against the
    // receiver at that tap, rather than the depth at the kernel center. A
    // constant bias alone produces stripes on sloping faces and hair cards.
    // Differentiate the actual depth triangles, not the smoothly displaced
    // receiver: derivatives of the offset normal would introduce a different
    // comparison plane on each triangle and recreate checkerboard shadow acne.
    // Evaluate before the nonuniform frustum/alpha tests.
    vec4 unshiftedPosition = uLightViewProjection * vec4(vWorldPosition, 1.0);
    vec3 unshiftedCoord = unshiftedPosition.xyz / unshiftedPosition.w * 0.5 + 0.5;
    vec3 dx = dFdx(unshiftedCoord);
    vec3 dy = dFdy(unshiftedCoord);
    float determinant = dx.x * dy.y - dx.y * dy.x;
    vec2 gradient = vec2(0.0);
    if (abs(determinant) > 1e-12)
        gradient = vec2(dy.y * dx.z - dx.y * dy.z, dx.x * dy.z - dy.x * dx.z) / determinant;
    // A bilinear tap can reach almost a whole texel to the farther of its two
    // centers. Cover that footprint as well, with a small precision margin.
    float bias = min(dot(abs(gradient), vec2(uShadowParams.x * 1.05)), 0.02) +
        uShadowParams.y * uShadowParams.z * (uMaterialKind == 1 ? 2.0 : 0.35) + 1.2e-7;
    if (any(lessThan(coord, vec3(0.0))) || any(greaterThan(coord, vec3(1.0)))) return 1.0;
    // Skin receives a wider penumbra; fabric/hair/metal keep crisp cel shadows.
    // The footprint is fixed in light space, so camera motion cannot move it.
    int radius = uMaterialKind == 1 ? 2 : 1;
    float spacing = uMaterialKind == 1 ? 2.0 : 1.0;
    float visibility = 0.0;
    for (int y = -radius; y <= radius; ++y)
        for (int x = -radius; x <= radius; ++x)
        {
            vec2 offset = vec2(x, y) * uShadowParams.x * spacing;
            visibility += texture(uShadowMap, vec3(coord.xy + offset, coord.z + dot(gradient, offset) - bias));
        }
    return visibility / float((2 * radius + 1) * (2 * radius + 1));
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
    // Skin gets a small wrap at the terminator; cast shadows still occlude it.
    // Material-space widths keep the painted transition stable at any zoom.
    if (uMaterialKind == 1) edge -= 0.08;
    float softness = clamp(uShadowSoftness, 0.005, 0.5);
    float lit = celStep(edge, ndl, softness);
    float core = 1.0 - celStep(edge - 0.52, ndl, max(softness, 0.09));

    vec3 tint = uShadowTint;
    float coreStrength = 0.10;
    if (uMaterialKind == 1)
    {
        // Warm, readable skin shadows; avoid a dark purple core on cheeks.
        tint = mix(tint, vec3(0.86, 0.66, 0.64), 0.65);
        coreStrength = 0.035;
    }
    else if (uMaterialKind == 2) tint *= vec3(1.02, 0.98, 1.04);
    else if (uMaterialKind == 3) { tint = mix(tint, vec3(0.80), 0.65); coreStrength = 0.04; }

    float visibility = shadowVisibility();
    float castVisibility = mix(1.0, visibility, clamp(uShadowStrength, 0.0, 1.0));
    float direct = lit * castVisibility;
    vec3 shadowColor = albedo * tint * (1.0 - core * coreStrength);
    vec3 color = mix(shadowColor, albedo * uKeyLightColor, direct);
    // A continuous hemisphere fill avoids a second razor-thin terminator
    // cutting across low-poly surfaces. This fill never follows the camera.
    float sky = normal.y * 0.5 + 0.5;
    vec3 bounce = mix(vec3(0.035, 0.025, 0.022), vec3(0.025, 0.035, 0.055), sky);
    color += albedo * bounce * (1.0 - direct);

    vec3 halfSum = lightDirection + viewDirection;
    float halfLength = dot(halfSum, halfSum);
    vec3 halfDirection = safeNormal(halfSum, normal);
    float ndh = max(dot(normal, halfDirection), 0.0);
    vec3 normalDx = dFdx(normal), normalDy = dFdy(normal);
    float variance = 0.5 * (dot(normalDx, normalDx) + dot(normalDy, normalDy));
    float specular = paintedLobe(ndh, uSpecularPower, variance);
    vec3 highlightColor = vec3(1.0, 0.98, 0.94);
    float strength = clamp(uSpecularStrength, 0.0, 1.0);
    float ndv = max(dot(normal, viewDirection), 0.0);
    if (uMaterialKind == 5)
    {
        // Woven cloth has a broad, dyed grazing sheen, no mirror reflection.
        // Multiplication preserves dark folds and the texture's saturation.
        float sheen = (1.0 - ndv) * (1.0 - ndv) * smoothstep(-0.05, 0.65, ndl);
        color += albedo * sheen * strength * 0.12 * castVisibility;
        specular = 0.0;
    }
    else if (uMaterialKind == 1)
    {
        specular = paintedLobe(ndh, clamp(uSpecularPower, 8.0, 32.0), variance);
        strength = min(strength, 0.055);
        highlightColor = mix(albedo, vec3(1.0, 0.88, 0.80), 0.65);
    }
    else if (uMaterialKind == 2)
    {
        vec3 tangent = vMaterialTangent - normal * dot(vMaterialTangent, normal);
        float tangentLength = dot(tangent, tangent);
        vec3 fallback = abs(normal.y) < 0.95 ? cross(normal, vec3(0.0, 1.0, 0.0)) :
                                                            cross(normal, vec3(1.0, 0.0, 0.0));
        tangent = safeNormal(tangent, safeNormal(fallback, vec3(1.0, 0.0, 0.0)));
        float th = dot(tangent, halfDirection);
        float strandLobe = sqrt(max(1.0 - th * th, 0.0));
        float strandSpecular = paintedLobe(strandLobe, uSpecularPower, variance) *
                               celStep(0.72, ndh, 0.09);
        // The strand direction vanishes on caps aligned with the axis. Fade
        // into the isotropic lobe there instead of exposing a fallback seam.
        specular = mix(specular, strandSpecular, smoothstep(0.01, 0.09, tangentLength));
        highlightColor = min(albedo * 1.65 + sqrt(albedo) * 0.10, vec3(0.85));
        strength = min(strength, 0.45);
    }
    else if (uMaterialKind == 4)
    {
        // Two painted metal bands, with a tinted shoulder and a small core.
        float shoulder = paintedLobe(ndh, max(uSpecularPower * 0.18, 4.0), variance);
        specular = max(specular, shoulder * 0.22);
        highlightColor = mix(vec3(0.95), sqrt(albedo), 0.45);
        strength = min(strength, 0.75);
    }
    else if (uMaterialKind == 6)
    {
        specular = paintedLobe(ndh, clamp(uSpecularPower, 8.0, 40.0), variance);
        highlightColor = min(albedo * 1.45 + vec3(0.025), vec3(0.9));
        strength = min(strength, 0.20);
    }
    else if (uMaterialKind != 3)
    {
        // Unknown materials start matte; only explicit eye/metal profiles
        // should be able to turn a dark surface into a near-white reflection.
        highlightColor = min(albedo * 1.4 + vec3(0.025), vec3(0.9));
        strength = min(strength, 0.12);
    }
    specular *= lit * castVisibility * (halfLength > 1e-12 ? 1.0 : 0.0);
    // Blend toward the highlight color to retain detail on bright materials.
    // This avoids an additive hard clamp erasing the texture near a glint.
    float highlight = clamp(specular * strength, 0.0, 0.85);
    color = mix(color, max(color, highlightColor * uKeyLightColor), highlight);

    float fresnel = 1.0 - ndv;
    float rim = celStep(0.80, fresnel, 0.06) * smoothstep(0.0, 0.55, ndl);
    // A single light-side rim; no blue Fresnel halo around the entire model.
    vec3 rimTint = uMaterialKind == 3 || uMaterialKind == 4 ? vec3(1.0, 0.97, 0.90) :
                   albedo * vec3(1.0, 0.97, 0.90);
    color += rimTint * rim * castVisibility * min(max(uRimStrength, 0.0), 0.25) * 0.35;

    if (uMaterialKind == 7) color = albedo; // Authored brows/lashes retain their ink value.
    // Keep derivative evaluation above nonuniform alpha discard, including at
    // hair-card boundaries, where derivatives after discard are undefined.
    alphaTest(base.a);
    FragColor = vec4(clamp(color, 0.0, 1.0), alpha);
}
