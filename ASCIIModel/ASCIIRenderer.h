// Render Module (Модуль рендера): управление рендерингом 3D модели, шейдерами, освещением и текстурами.
//OpenGL Context: управление контекстом OpenGL и шейдерами.
//Shader Management : компиляция и применение шейдеров(vertex, fragment, и других).
//Frame Management : управление кадрами(буферы, очищение экрана и т.д.).
//Model Rendering : отрисовка модели и применение анимаций.
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
// Функциии для проверки инициализации библиотек====
//==================================================

bool checkOpenGLInitialization();//Проверка OpenGL
bool checkAssimpInitialization();//Проверка Assimp
bool checkGLMInitialization();// Проверка GLM
bool EngineCheckInitializations();// Проверка всех библиотек

// Функция получения разрешения экрана для создания окна на разных мониторах
void getScreenResolution(int& width, int& height);

//==================================================
// Класы OpenGLWindow и сам Engine==================
//==================================================


class Engine;
class OpenGLWindow
{
public:
    OpenGLWindow(int width, int height, const char* title);
    ~OpenGLWindow();

    void run(); // Запуск основного цикла

private:
    int windowWidth;
    int windowHeight;
    const char* windowTitle;
    GLFWwindow* window;

    bool initialize(); // Инициализация GLFW и GLAD
    void setWindowPosition(); // Устанавливаем координаты окна
    void processInput(); // Обработка ввода
    void render(); // Рендеринг кадра
};


// Параметры камеры по умолчанию
const float YAW = -90.0f;
const float PITCH = 0.0f;
const float SPEED = 2.5f;
const float SENSITIVITY = 0.1f;
const float ZOOM = 45.0f;

class Camera
{
private:
    // Метод для обновления векторов направления камеры на основе текущих значений yaw и pitch
    void updateCameraVectors();

public:
    // Параметры камеры
    glm::vec3 position;     // Позиция камеры
    glm::vec3 front;        // Вектор направления камеры
    glm::vec3 up;           // Вектор "вверх" для камеры
    glm::vec3 right;        // Вектор "вправо", автоматически рассчитывается
    glm::vec3 worldUp;      // Вектор "вверх" мира

    float yaw;              // Угол поворота камеры по горизонтали
    float pitch;            // Угол поворота камеры по вертикали
    float fov;              // Угол обзора (Field of View)

    // Конструктор камеры с начальными параметрами
    Camera(glm::vec3 startPosition, glm::vec3 startUp, float startYaw, float startPitch, float startFov);

    // Метод для получения матрицы вида (view matrix)
    glm::mat4 getViewMatrix() const;

    // Изменение масштаба (увеличение/уменьшение угла обзора)
    void processMouseScroll(float yoffset);

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
    Camera Camera1;

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


