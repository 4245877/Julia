#version 330 core

in vec2 vUv;

uniform vec3 uBgColor;       // Parchment
uniform vec3 uSurfaceColor;  // Warm Ivory
uniform vec3 uSkyColor;      // Dawn Blue
uniform vec3 uGoldColor;     // Muted Ochre
uniform vec3 uAccentColor;   // Rosewood Wine
uniform vec3 uForestColor;   // Dusty Plum
uniform vec3 uPrimaryColor;  // Chestnut Amber
uniform vec3 uBorderColor;   // Soft Silver

out vec4 FragColor;

float hash(vec2 p)
{
    return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453123);
}

vec3 srgbToLinear(vec3 color)
{
    return pow(clamp(color, 0.0, 1.0), vec3(2.2));
}

// Постерный срез: плоские поля с узкой (~1 px) кромкой сглаживания.
// Никаких мягких живописных переходов — фон собирается из чётких форм.
float posterStep(float edge, float value)
{
    float aa = max(fwidth(value) * 1.5, 0.002);
    return smoothstep(edge - aa, edge + aa, value);
}

void main()
{
    vec2 uv = clamp(vUv, 0.0, 1.0);

    vec3 primaryColor = srgbToLinear(uPrimaryColor);
    vec3 forestColor = srgbToLinear(uForestColor);
    vec3 goldColor = srgbToLinear(uGoldColor);
    vec3 accentColor = srgbToLinear(uAccentColor);
    vec3 skyColor = srgbToLinear(uSkyColor);
    vec3 bgColor = srgbToLinear(uBgColor);
    vec3 surfaceColor = srgbToLinear(uSurfaceColor);
    vec3 borderColor = srgbToLinear(uBorderColor);

    vec2 focus = vec2(0.50, 0.52);

    // 1) Три плоских диагональных поля вместо плавного градиента:
    //    светлое внизу (под персонажем), среднее в центре, тёмное сверху.
    float diagonal = uv.y - uv.x * 0.22;
    vec3 topColor = mix(bgColor, forestColor, 0.35);

    vec3 color = mix(surfaceColor, bgColor, posterStep(0.30, diagonal));
    color = mix(color, topColor, posterStep(0.72, diagonal));

    // 2) Широкий винный луч по диагонали — графический акцент,
    //    который позже частично перекроется диском.
    float beamValue = uv.x * 0.62 - uv.y + 0.30;
    float beam =
        posterStep(-0.045, beamValue) *
        (1.0 - posterStep(0.045, beamValue));

    color = mix(color, accentColor, beam * 0.14);

    // 3) Группа косых полос в правом верхнем углу — намёк на спидлайны.
    float stripeCoord = uv.x + uv.y * 0.35;
    float stripePhase = fract(stripeCoord * 14.0);
    float stripes =
        smoothstep(0.53, 0.57, stripePhase) *
        (1.0 - smoothstep(0.78, 0.82, stripePhase));
    float stripeZone = posterStep(1.18, uv.x + uv.y);

    color = mix(color, skyColor, stripes * stripeZone * 0.16);

    // 4) Растровая точечная сетка в левом нижнем углу:
    //    классический полиграфический screen-tone, радиус точек
    //    затухает от угла к центру.
    vec2 dotGrid = uv * vec2(46.0, 26.0);
    vec2 dotCell = fract(dotGrid) - 0.5;
    float cornerFade =
        1.0 - smoothstep(0.05, 0.55, distance(uv, vec2(0.06, 0.10)));
    float dotRadius = 0.34 * cornerFade;
    float dots = 1.0 - smoothstep(
        max(dotRadius - 0.06, 0.0),
        dotRadius + 0.02,
        length(dotCell)
    );

    color = mix(color, forestColor, dots * step(0.02, dotRadius) * 0.30);

    // 5) Плоский диск за персонажем — большое читаемое пятно света
    //    с жёсткой кромкой вместо мягкого портретного овала.
    float focusDistance = distance(uv, focus);
    float disc = 1.0 - posterStep(0.335, focusDistance);

    color = mix(color, mix(surfaceColor, skyColor, 0.18), disc * 0.50);

    // Тонкое серебряное кольцо по краю диска.
    float ring =
        posterStep(0.355, focusDistance) *
        (1.0 - posterStep(0.372, focusDistance));

    color = mix(color, borderColor, ring * 0.55);

    // Второе кольцо — разорванная охристая дуга, чуть дальше от центра.
    float arcAngle = atan(uv.y - focus.y, uv.x - focus.x);
    float outerRing =
        posterStep(0.408, focusDistance) *
        (1.0 - posterStep(0.418, focusDistance));
    float arcSector = smoothstep(0.10, 0.20, sin(arcAngle + 0.6));

    color = mix(color, goldColor, outerRing * arcSector * 0.45);

    // 6) Плоская каштановая "плита" пола с серебряной линией сверху:
    //    заземляет фигуру, как панель интерфейса.
    float floorBand = 1.0 - posterStep(0.085, uv.y);
    color = mix(color, mix(primaryColor, bgColor, 0.45), floorBand * 0.55);

    float floorLine =
        posterStep(0.085, uv.y) *
        (1.0 - posterStep(0.094, uv.y));

    color = mix(color, borderColor, floorLine * 0.40);

    // 7) Постеризованная виньетка: два жёстких кольца затемнения
    //    вместо плавного радиального градиента.
    float vignetteInner = posterStep(0.58, focusDistance);
    float vignetteOuter = posterStep(0.78, focusDistance);

    color = mix(color, forestColor, vignetteInner * 0.12 + vignetteOuter * 0.14);

    // Лёгкое печатное зерно, чтобы плоские поля не были пластиковыми.
    float grain = hash(floor(uv * vec2(720.0, 420.0))) - 0.5;
    color += grain * 0.008;

    color = clamp(color, 0.0, 1.0);

    FragColor = vec4(color, 1.0);
}
