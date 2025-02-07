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


// ���������� ��������� ��������� ��������� �������� ������. ������������ � �������� ����������, ����� ��������� �������� �� ����������� ��� ������� ������� ������� �����
enum Camera_Movement {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
};

// ��������� ������ �� ���������
const float YAW = -90.0f;
const float PITCH = 0.0f;
const float SPEED = 2.5f;
const float SENSITIVITY = 0.1f;
const float ZOOM = 45.0f;

// ����������� ����� ������, ������� ������������ ������� ������ � ��������� ��������������� ���� ������, ������� � ������� ��� ������������� � OpenGL
class Camera
{
public:
    // �������� ������
    glm::vec3 Position;
    glm::vec3 Front;
    glm::vec3 Up;
    glm::vec3 Right;
    glm::vec3 WorldUp;

    // ���� ������
    float Yaw;
    float Pitch;

    // ��������� ������
    float MovementSpeed;
    float MouseSensitivity;
    float Zoom;

    // �����������, ������������ �������
    Camera(glm::vec3 position = glm::vec3(0.0f, 0.0f, 0.0f),
        glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f),
        float yaw = YAW,
        float pitch = PITCH);

    // �����������, ������������ �������
    Camera(float posX, float posY, float posZ, float upX, float upY, float upZ, float yaw, float pitch);

    // ���������� ������� ����, ����������� � �������������� ����� ������ � LookAt-������� 
    glm::mat4 GetViewMatrix();

    // ������������ ������� ������, ���������� �� ������������ ������� �����. ��������� ������� �������� � ���� ������������� ������� ������������ (��� ��������������� ��� �� ������� ������)
    void ProcessKeyboard(Camera_Movement direction, float deltaTime);


private:
    // ��������� ������-����� �� (�����������) ����� ������ ������
    void updateCameraVectors();
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
    Camera camera{};


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