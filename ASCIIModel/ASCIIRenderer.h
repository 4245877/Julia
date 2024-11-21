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

bool checkOpenGLInitialization();//�������� OpenGL
bool checkAssimpInitialization();//�������� Assimp
bool checkGLMInitialization();// �������� GLM
bool EngineCheckInitializations();// �������� ���� ���������

// ������� ��������� ���������� ������ ��� �������� ���� �� ������ ���������
void getScreenResolution(int& width, int& height);

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


// ��������� ������ �� ���������
const float YAW = -90.0f;
const float PITCH = 0.0f;
const float SPEED = 2.5f;
const float SENSITIVITY = 0.1f;
const float ZOOM = 45.0f;

class Camera
{
private:
    // ����� ��� ���������� �������� ����������� ������ �� ������ ������� �������� yaw � pitch
    void updateCameraVectors();

public:
    // ��������� ������
    glm::vec3 position;     // ������� ������
    glm::vec3 front;        // ������ ����������� ������
    glm::vec3 up;           // ������ "�����" ��� ������
    glm::vec3 right;        // ������ "������", ������������� ��������������
    glm::vec3 worldUp;      // ������ "�����" ����

    float yaw;              // ���� �������� ������ �� �����������
    float pitch;            // ���� �������� ������ �� ���������
    float fov;              // ���� ������ (Field of View)

    // ����������� ������ � ���������� �����������
    Camera(glm::vec3 startPosition, glm::vec3 startUp, float startYaw, float startPitch, float startFov);

    // ����� ��� ��������� ������� ���� (view matrix)
    glm::mat4 getViewMatrix() const;

    // ��������� �������� (����������/���������� ���� ������)
    void processMouseScroll(float yoffset);

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
    Camera Camera1;

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
    Engine(int width, int height);

    void runWindow();
    bool loadModel(const std::string& filepath);
    void setAnimation(const std::string& animationName);
    void update(float deltaTime);
    void render();
    void shutdown();
};


