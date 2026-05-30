#version 460 core

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

float softCircle(vec2 uv, vec2 center, float radius, float softness)
{
    float d = distance(uv, center);
    return 1.0 - smoothstep(radius - softness, radius, d);
}

float softEllipse(vec2 uv, vec2 center, vec2 radius, float softness)
{
    vec2 p = (uv - center) / radius;
    float d = length(p);
    return 1.0 - smoothstep(1.0 - softness, 1.0, d);
}

void main()
{
    vec2 uv = clamp(vUv, 0.0, 1.0);


    vec3 color;

    // Тёплая пергаментная основа.
    float vertical = smoothstep(0.0, 1.0, uv.y);
    color = mix(uSurfaceColor, uBgColor, vertical);

    // Светлый портретный овал за персонажем.
    // Он помогает Рафталии читаться на фоне и не спорит с моделью.
    float portraitLight = softEllipse(
        uv,
        vec2(0.50, 0.55),
        vec2(0.34, 0.48),
        0.55
    );
    color = mix(color, uSurfaceColor, portraitLight * 0.34);

    // Каштаново-медовое тепло снизу слева:
    // связь с волосами, домом и живым человеческим теплом.
    float chestnutGlow = softCircle(
        uv,
        vec2(0.14, 0.08),
        0.88,
        0.72
    );
    color = mix(color, uPrimaryColor, chestnutGlow * 0.20);

    // Охристое заземление внизу:
    // кожа, ремни, броня, телесность мечницы.
    float ochreGround = 1.0 - smoothstep(0.0, 0.62, uv.y);
    color = mix(color, uGoldColor, ochreGround * 0.12);

    // Винное сердечное свечение по центру.
    // Оно должно быть заметным, но не агрессивным.
    float heartGlow = softEllipse(
        uv,
        vec2(0.50, 0.46),
        vec2(0.24, 0.32),
        0.72
    );
    color = mix(color, uAccentColor, heartGlow * 0.13);

    // Пыльно-сливовая тень по краям:
    // глубина, память о боли, взрослая сдержанность.
    float vignette = smoothstep(
        0.38,
        0.92,
        distance(uv, vec2(0.50, 0.54))
    );
    color = mix(color, uForestColor, vignette * 0.19);

    // Более мягкие боковые тени, как от тёмных панелей костюма.
    float sideShadow =
        (1.0 - smoothstep(0.00, 0.24, uv.x)) +
        smoothstep(0.76, 1.00, uv.x);

    color = mix(color, uForestColor, sideShadow * 0.055);

    // Небесно-стальной холодный воздух сверху справа.
    // Нужен, чтобы тёплая гамма не стала слишком тяжёлой.
    float dawnAir = softCircle(
        uv,
        vec2(0.82, 0.88),
        0.74,
        0.64
    );
    color = mix(color, uSkyColor, dawnAir * 0.24);

    // Тонкий серебряный диагональный отблеск:
    // намёк на клинок и позднюю броню, но без лишней резкости.
    float bladeLine = exp(
        -pow((uv.x - (0.18 + uv.y * 0.48)) * 8.2, 2.0)
    );
    bladeLine *= smoothstep(0.10, 0.46, uv.y);
    bladeLine *= 1.0 - smoothstep(0.82, 1.0, uv.y);

    color = mix(color, uBorderColor, bladeLine * 0.075);

    // Очень мягкая розовая эмоция около верхней центральной зоны.
    // Это отсылка к глазам, но без отдельного uniform.
    vec3 irisRose = mix(uAccentColor, uSurfaceColor, 0.30);

    float irisMist = softCircle(
        uv,
        vec2(0.36, 0.70),
        0.42,
        0.36
    );

    color = mix(color, irisRose, irisMist * 0.055);

    // Небольшие светлые частицы по краям.
    // В центре их почти нет, чтобы они не мешали модели.
    vec2 grid = uv * vec2(9.0, 6.0);
    vec2 id = floor(grid);
    vec2 cell = fract(grid) - 0.5;

    float rnd = hash(id);
    vec2 offset = vec2(
        hash(id + vec2(17.31, 9.17)),
        hash(id + vec2(3.11, 41.73))
    ) - 0.5;

    offset *= 0.52;

    float mote = 1.0 - smoothstep(
        0.0,
        0.030 + rnd * 0.020,
        length(cell - offset)
    );

    float moteVisibility = step(0.70, rnd);
    float keepAwayFromFace = smoothstep(
        0.26,
        0.55,
        distance(uv, vec2(0.50, 0.56))
    );

    color = mix(
        color,
        mix(uBorderColor, uSurfaceColor, 0.42),
        mote * moteVisibility * keepAwayFromFace * 0.11
    );

    // Лёгкая текстура бумаги, чтобы фон не был пластиковым.
    float grain = hash(floor(uv * vec2(720.0, 420.0))) - 0.5;
    color += grain * 0.010;

    color = clamp(color, 0.0, 1.0);

    FragColor = vec4(color, 1.0);
}