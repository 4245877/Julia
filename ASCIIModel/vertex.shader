int vec2 positions;
void main()
{
    gl_Position = vec4(positions.xy, 0.0, 1.0);
}