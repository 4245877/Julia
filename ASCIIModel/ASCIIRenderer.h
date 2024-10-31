// Render Module (������ �������): ���������� ����������� 3D ������, ���������, ���������� � ����������.
//OpenGL Context: ���������� ���������� OpenGL � ���������.
//Shader Management : ���������� � ���������� ��������(vertex, fragment, � ������).
//Frame Management : ���������� �������(������, �������� ������ � �.�.).
//Model Rendering : ��������� ������ � ���������� ��������.
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <glm.hpp>
#include <vec2.hpp>
#include <vec3.hpp>
#include <vec4.hpp>
#include <mat4x4.hpp>
#include <gtc/matrix_transform.hpp>
#include <gtc/type_ptr.hpp>
#include <iostream>
#include <fstream>
#include "Audio.h"
#include "Mesh.h"
#include "Model Loader Module.h"

//==================================================
// �������� ��� �������� ������������� ���������====
//==================================================

//�������� OpenGL
bool checkOpenGLInitialization() {
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "\x1b[31mFailed to initialize GLAD\x1b[0m" << std::endl;
        return false; // ���������� false, ���� ��������� ������
    }
    return true; // ���������� true, ���� ������������� ������ �������
}

//�������� Assimp
bool checkAssimpInitialization() {
    Assimp::Importer importer;
    // ������� ��������� �������������� ���� ��� ��������
    const aiScene* scene = importer.ReadFile("non_existent_file.obj", 0);

    if (scene == nullptr) {
        std::cerr << "\x1b[31mFailed to initialize Assimp: \x1b[0m" << importer.GetErrorString() << std::endl;
        return false; // ���������� false, ���� ��������� ������
    }
    return true; // ���������� true, ���� ������������� ������ �������
}

// �������� GLM
bool checkGLMInitialization() {
    // ���������, ����� �� �� ������� ������� ������ � �������
    glm::vec3 testVector(1.0f, 2.0f, 3.0f);
    glm::mat4 testMatrix = glm::mat4(1.0f);

    // �������� �������� ������� � ������� (����� �������� ����� ������� ��������)
    if (testVector.x != 1.0f || testMatrix[0][0] != 1.0f) {
        std::cerr << "\x1b[31mFailed to initialize GLM\x1b[0m" << std::endl;
        return false; // ���������� false, ���� ��������� ������
    }
    return true; // ���������� true, ���� ������������� ������ �������
}

// �������� ���� ���������
bool EngineCheckInitializations() {
    return checkOpenGLInitialization() && checkAssimpInitialization() && checkGLMInitialization();
}

// ������� ��������� ���������� ������ ��� �������� ���� �� ������ ���������
void getScreenResolution(int& width, int& height) {
    // ������������� GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return;
    }

    // ��������� �������������
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    if (mode) {
        width = mode->width;
        height = mode->height;
    }
    else {
        std::cerr << "Failed to get video mode" << std::endl;
    }

}

//==================================================
// ����� OpenGLWindow � ��� Engine==================
//==================================================

class Engine;

class OpenGLWindow
{
public:
    OpenGLWindow(int width, int height, const char* title);
    ~OpenGLWindow();

    void run(); // ������ ��������� �����

private:
    int windowWidth;
    int windowHeight;
    const char* windowTitle;
    GLFWwindow* window;

    bool initialize(); // ������������� GLFW � GLAD
    void setWindowPosition(); // ������������� ���������� ����
    void processInput(); // ��������� �����
    void render(); // ��������� �����
};

OpenGLWindow::OpenGLWindow(int width, int height, const char* title = nullptr)
    : windowWidth(width), windowHeight(height), windowTitle(title ? title : ""), window(nullptr)
{
    if (!initialize())
    {
        std::cerr << "Failed to initialize OpenGLWindow." << std::endl;
        exit(EXIT_FAILURE);
    }
    setWindowPosition(); // ������������� ������� ���� ����� �������������
}
OpenGLWindow::~OpenGLWindow()
{
    glfwTerminate();
}

bool OpenGLWindow::initialize()
{
    // ������������� GLFW
    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW." << std::endl;
        return false;
    }

    // ������������� ��������� GLFW
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_DECORATED, GLFW_FALSE); // ������� ����� � ������ ����������

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // ��� macOS
#endif

    // �������� ����
    window = glfwCreateWindow(windowWidth, windowHeight, windowTitle, nullptr, nullptr);
    if (window == nullptr)
    {
        std::cerr << "Failed to create GLFW window." << std::endl;
        glfwTerminate();
        return false;
    }

    // ������������� �������� ����
    glfwMakeContextCurrent(window);

    // ������������� GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cerr << "Failed to initialize GLAD." << std::endl;
        return false;
    }

    // ������ ������� ������� ���������
    glViewport(0, 0, windowWidth, windowHeight);

    return true;
}
void OpenGLWindow::setWindowPosition()
{
    // �������� ������ ������
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());

    if (mode)
    {
        // ������������ ���������� ��� ������� ������� ����
        int posX = mode->width - windowWidth;
        int posY = mode->height - windowHeight - 40; // ��������� ������ ������ ����� (����� 40 ��������)

        // ������������� ������� ����
        glfwSetWindowPos(window, posX, posY);
    }
}
void OpenGLWindow::processInput()
{
    // �������� ���� ��� ������� ESC
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
    {
        glfwSetWindowShouldClose(window, true);
    }
}
void OpenGLWindow::render()
{
    // ������� ����� (������ ���� ����)
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
}
void OpenGLWindow::run()
{
    // �������� ���� ����
    while (!glfwWindowShouldClose(window))
    {
        // ��������� �����
        processInput();

        // ��������� �����
        render();

        // ������ ������
        glfwSwapBuffers(window);

        // ������������ �������
        glfwPollEvents();
    }
}


class Engine{
private:
    // 1. �������� � ���� OpenGL
    // 2. ������� 
    // 3. ������ � ��������:
    // 4. ��������� ����������
    // 5. ������� ������ � �������
    // 6. �������� ������

    // ���� � ��������
    OpenGLWindow window;

    // �������
    unsigned int shaderProgram;
    unsigned int vertexShader;
    unsigned int fragmentShader;

    // ������ � ��������
    //std::unique_ptr<Model> model;
    //std::map<std::string, Animation> animations;
    //Animation* currentAnimation;

    // ��������� ����������
    glm::mat4 viewMatrix;
    glm::mat4 projectionMatrix;
    //std::vector<Light> lights;
    float deltaTime;
    float lastFrameTime;

    // ����� � �������
    //std::vector<Bone> bones;
    //glm::mat4 globalInverseTransform;

    // ���������� ������
    bool compileShader(const std::string& vertexSource, const std::string& fragmentSource);
    std::string loadShaderSource(const std::string& shaderName);
    bool loadTextures();
    void loadModelData(const std::string& filepath);
    void updateAnimation(float deltaTime);
    void applyTransformations();
    void updateLights();
    void renderScene();
    void applyShaderUniforms();
    void cleanup();
    static int initialization();
    static void finalization();

    SoundEngine soundEngine;
public:
    Engine(int width, int height)
        : window(width, height) {}
    void runWindow() {
        window.run();
    }
    bool initialize(); // ������������� GLFW � �������� ����.
    bool loadModel(const std::string& filepath);
    void setAnimation(const std::string& animationName);
    void update(float deltaTime);
    void render();
    void shutdown();
};

std::string Engine::loadShaderSource(const std::string& shaderName) {
    std::string path = "../ASCIIModel/" + shaderName;
    std::ifstream shaderFile(path);
    if (!shaderFile) {
        std::cerr << "\x1b[31mFailed to open shader file: \x1b[0m" << path << std::endl;
        return "";
    }

    // ������ ����� �������� � ������
    std::string shaderSource((std::istreambuf_iterator<char>(shaderFile)),
        std::istreambuf_iterator<char>());
    return shaderSource;
}