/*#version 330 core

uniform vec2 resolution;
uniform float time;
out vec4 theColor;

void main()
{
    // Normalized pixel coordinates (from 0 to 1)
    vec2 uv = gl_FragCoord.xy / resolution.xy;

    // Time varying pixel color
    vec3 col = 0.5 + 0.5 * cos(time + vec3(uv, 0.0));

    // Output to screen
    theColor = vec4(col, 1.0);
}
*/