// Render Module (Модуль рендера): управление рендерингом 3D модели, шейдерами, освещением и текстурами.
//OpenGL Context: управление контекстом OpenGL и шейдерами.
//Shader Management : компиляция и применение шейдеров(vertex, fragment, и других).
//Frame Management : управление кадрами(буферы, очищение экрана и т.д.).
//Model Rendering : отрисовка модели и применение анимаций.
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <gtc/type_ptr.hpp>
#include <iostream>
#include "Audio.h"
#include "Mesh.h"
#include "Model Loader Module.h"


//==================================================
// Функциии для проверки инициализации библиотек====
//==================================================

bool checkOpenGLInitialization();//Проверка OpenGL
bool checkAssimpInitialization();//Проверка Assimp
bool checkGLMInitialization();// Проверка GLM
bool EngineCheckInitializations();// Проверка всех библиотек

// Функция получения разрешения экрана для создания окна на разных мониторах
void getScreenResolution(float& width, float& height);

//==================================================
// Класы OpenGLWindow и сам Engine==================
//==================================================


class Engine;
class OpenGLWindow
{
public:
    OpenGLWindow(float width, float height, const char* title);
    ~OpenGLWindow();

    void run(); // Запуск основного цикла
    float GetWindowWidth() { return windowWidth; }
    float GetWindowHeight() { return windowHeight; }
private:
    float windowWidth;
    float windowHeight;
    const char* windowTitle;
    GLFWwindow* window;



    bool initialize(); // Инициализация GLFW и GLAD
    void setWindowPosition(); // Устанавливаем координаты окна
    void processInput(); // Обработка ввода
    void render(); // Рендеринг кадра
};


// Определяем несколько возможных вариантов движения камеры. Используется в качестве абстракции, чтобы держаться подальше от специфичных для оконной системы методов ввода
enum Camera_Movement {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
};

// Параметры камеры по умолчанию
const float YAW = -90.0f;
const float PITCH = 0.0f;
const float SPEED = 2.5f;
const float SENSITIVITY = 0.1f;
const float ZOOM = 45.0f;

// Абстрактный класс камеры, который обрабатывает входные данные и вычисляет соответствующие углы Эйлера, векторы и матрицы для использования в OpenGL
class Camera
{
public:
    // Атрибуты камеры
    glm::vec3 Position;
    glm::vec3 Front;
    glm::vec3 Up;
    glm::vec3 Right;
    glm::vec3 WorldUp;

    // Углы Эйлера
    float Yaw;
    float Pitch;

    // Настройки камеры
    float MovementSpeed;
    float MouseSensitivity;
    float Zoom;

    // Конструктор, использующий векторы
    Camera(glm::vec3 position = glm::vec3(0.0f, 0.0f, 0.0f),
        glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f),
        float yaw = YAW,
        float pitch = PITCH);

    // Конструктор, использующий скаляры
    Camera(float posX, float posY, float posZ, float upX, float upY, float upZ, float yaw, float pitch);

    // Возвращаем матрицу вида, вычисленную с использованием углов Эйлера и LookAt-матрицы 
    glm::mat4 GetViewMatrix();

    // Обрабатываем входные данные, полученные от клавиатурной системы ввода. Принимаем входной параметр в виде определенного камерой перечисления (для абстрагирования его от оконных систем)
    void ProcessKeyboard(Camera_Movement direction, float deltaTime);


private:
    // Вычисляем вектор-прямо по (обновленным) углам Эйлера камеры
    void updateCameraVectors();
};

class Engine{
private:
    // 1. Контекст и окно OpenGL
    // 2. Шейдеры 
    // 3. Модель и анимации:
    // 4. Состояние рендеринга
    // 5. Система костей и риггинг
    // 6. Звуковой движок

    // Окно и контекст
    OpenGLWindow window;
    


    // Шейдеры
    unsigned int shaderProgram;
    unsigned int vertexShader;
    unsigned int fragmentShader;

    // Камера
    Camera camera{};


    // Модель и анимации
    std::unique_ptr<Model> model;
    std::map<std::string, Animation> animations;
    Animation* currentAnimation;

    // Состояние рендеринга
    glm::mat4 viewMatrix;
    glm::mat4 projectionMatrix;
    //std::vector<Light> lights;
    float deltaTime;
    float lastFrameTime;

    // Кости и риггинг
    //std::vector<Bone> bones;
    glm::mat4 globalInverseTransform;

    // Внутренние методы
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