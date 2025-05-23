// Render Module (������ �������): ���������� ����������� 3D ������, ���������, ���������� � ����������.
//OpenGL Context: ���������� ���������� OpenGL � ���������.
//Shader Management : ���������� � ���������� ��������(vertex, fragment, � ������).
//Frame Management : ���������� �������(������, �������� ������ � �.�.).
//Model Rendering : ��������� ������ � ���������� ��������.
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <gtc/type_ptr.hpp>
#include <iostream>
#include "Audio.h"
#include "Mesh.h"
#include "Model Loader Module.h"
#include "Camera.h"

//==================================================
// �������� ��� �������� ������������� ���������====
//==================================================

bool checkOpenGLInitialization();//�������� OpenGL
bool checkAssimpInitialization();//�������� Assimp
bool checkGLMInitialization();// �������� GLM
bool EngineCheckInitializations();// �������� ���� ���������

// ������� ��������� ���������� ������ ��� �������� ���� �� ������ ���������
void getScreenResolution(float& width, float& height);

//==================================================
// ����� OpenGLWindow � ��� Engine==================
//==================================================


class Engine;
class Renderer;
class ModelManager;

class OpenGLWindow
{
public:
    OpenGLWindow(float width, float height, const char* title);
    ~OpenGLWindow();

    void run(); // ������ ��������� �����
    float GetWindowWidth() { return windowWidth; }
    float GetWindowHeight() { return windowHeight; }
private:
    float windowWidth;
    float windowHeight;
    const char* windowTitle;
    GLFWwindow* window;



    bool initialize(); // ������������� GLFW � GLAD
    void setWindowPosition(); // ������������� ���������� ����
    void processInput(); // ��������� �����
    void render(); // ��������� �����
};


 

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



    // ������
    Camera camera;


    // ������ � ��������

    std::unique_ptr<Model> model;
    std::map<std::string, Animation> animations;
    Animation* currentAnimation;

    // ��������� ����������
    glm::mat4 viewMatrix;
    glm::mat4 projectionMatrix;
    //std::vector<Light> lights;
    float deltaTime;
    float lastFrameTime;

    // ����� � �������
    //std::vector<Bone> bones;
    glm::mat4 globalInverseTransform;

    // ���������� ������
    std::string loadShaderSource(const std::string& shaderName);
    bool compileShader(const std::string& vertexSource, const std::string& fragmentSource);
    bool loadTextures();
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
    Engine(float cameraPosX, float cameraPosY, float cameraPosZ,
        float upX, float upY, float upZ, float yaw, float pitch,
        float windowWidth, float windowHeight);

    void runWindow();
    void setAnimation(const std::string& animationName);
    void update(float deltaTime);
    void render();
    void shutdown();
};

//=========��� ��������, ���������� � ���������� ��������.=========
class ShaderManager {
public:
    ShaderManager();
    bool loadAndCompile(const std::string& vertexPath, const std::string& fragmentPath);
    void use();
    void setUniform(const std::string& name, const glm::mat4& value);
private:
    unsigned int programID;
};

//=========��� �������� ������� � ���������� ����������.=========
class ModelManager {
public:
    ModelManager();
    bool loadModel(const std::string& path);
    void updateAnimation(float deltaTime);
    std::unique_ptr<Model> model;
    std::map<std::string, Animation> animations;
};

//=========��� ���������� ��������� ���������� (������� �������, ��������� ������, ����� ���������).=========
class Renderer {
public:
    Renderer();
    void render(const ModelManager& modelMgr, const ShaderManager& shaderMgr, const Camera& camera);
};