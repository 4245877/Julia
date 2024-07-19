#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <fstream>
#include <filesystem>



std::string readFile(std::filesystem::path path)
{
    std::ifstream in(path, std::ios::binary);
    const auto sz = std::filesystem::file_size(path);
    std::string result(sz, '\0');
    in.read(result.data(), sz);

    return result;
}

unsigned int compileShader(unsigned int shaderType, std::string& scr)
{
    unsigned id = glCreateShader(shaderType);
    const char* raw = scr.c_str();
    glShaderSource(id, 1, &raw, nullptr);
    glCompileShader(id);
    return id;
}


unsigned int createShader(std::string& vertex, std::string& fragment)
{
    unsigned vs = compileShader(GL_VERTEX_SHADER, vertex);
    unsigned fs = compileShader(GL_FRAGMENT_SHADER, fragment);

    unsigned program = glCreateProgram();
    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);
    glValidateProgram(program);

    glDeleteShader(vs);
    glDeleteShader(fs);

    return program;
}





int main(void)
{
    GLFWwindow* window;

    /* Initialize the library */
    if (!glfwInit())
        return -1;

    /* Create a windowed mode window and its OpenGL context */
    window = glfwCreateWindow(1920, 1080, "OpenGL", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    /* Make the window's context current */
    glfwMakeContextCurrent(window);

    //Glad
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cout << "Feil opengl context" << std::endl;
        return -1;
    }

    float vetecies[]{
        -1.0, 1.0,
        1.0, -1.0,
        1.0, 1.0,

        1.0, 1.0,
        -1.0, 1.0,
        -1.0, -1.0
    };

    GLuint VBO;
    glGenBuffers(1, &VBO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, 6 * 2 * sizeof(float), vetecies, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 6 * sizeof(float), 0);


    glEnableVertexAttribArray(0);

    std::string vertex = readFile("Vertex.shader");
    std::string fragment = readFile("Fragment.shader");

    unsigned int shader = createShader(vertex, fragment);

    glUseProgram(shader);

    glUniform2f(glGetUniformLocation(shader, "resolution"), 1280, 720);

    /* Loop until the user closes the window */
    while (!glfwWindowShouldClose(window))
    {
        glUniform1f(glGetUniformLocation(shader, "time"), glfwGetTime());
        glDrawArrays(GL_TRIANGLES, 0, 6);

        /* Swap front and back buffers */
        glfwSwapBuffers(window);

        /* Poll for and process events */
        glfwPollEvents();
    }

    glDeleteProgram(shader);
    glfwTerminate();
    return 0;
}
