// Render Module (Модуль рендера): управление рендерингом 3D модели, шейдерами, освещением и текстурами.
//OpenGL Context: управление контекстом OpenGL и шейдерами.
//Shader Management : компиляция и применение шейдеров(vertex, fragment, и других).
//Frame Management : управление кадрами(буферы, очищение экрана и т.д.).
//Model Rendering : отрисовка модели и применение анимаций.
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <gtc/type_ptr.hpp>
#include <iostream>
#include <fstream> // Для загрузки шейдеров из файлов
#include <sstream> // Для чтения файлов в строку
#include <vector> // Для логов ошибок
#include <map>    // Если понадобится для анимаций или других ресурсов

#include "Audio.h"
#include "Mesh.h"
#include "Model Loader Module.h"
#include "Camera.h"
#include "OpenGLWindow.h"

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
class Renderer;
class ModelManager;


 

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


    GLuint VAO; // Объект вершинного массива
    GLuint VBO; // Объект вершинного буфера

    // Камера
    Camera camera;


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

    void initializeGraphics();
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

//=========Для загрузки, компиляции и применения шейдеров.=========
class ShaderManager {
public:
    ShaderManager();
    bool loadAndCompile(const std::string& vertexPath, const std::string& fragmentPath);
    void use();
    void setUniform(const std::string& name, const glm::mat4& value);
private:
    unsigned int programID;
};

//=========Для загрузки моделей и управления анимациями.=========
class ModelManager {
public:
    ModelManager();
    bool loadModel(const std::string& path);
    void updateAnimation(float deltaTime);
    std::unique_ptr<Model> model;
    std::map<std::string, Animation> animations;
};

//=========Для управления процессом рендеринга (очистка буферов, настройка кадров, вызов отрисовки).=========
class Renderer {
public:
    Renderer();
    void render(const ModelManager& modelMgr, const ShaderManager& shaderMgr, const Camera& camera);
};