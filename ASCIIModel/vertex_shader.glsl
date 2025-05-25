#version 330 core
layout (location = 0) in vec3 aPos; // Позиция вершины

uniform mat4 view;
uniform mat4 projection;
// uniform mat4 model; // Если вы используете модельную матрицу

void main()
{
    // gl_Position = projection * view * model * vec4(aPos, 1.0);
    gl_Position = projection * view * vec4(aPos, 1.0); // Для простого треугольника без модели
}