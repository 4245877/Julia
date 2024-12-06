#include "ASCIIRenderer.h"

//==================================================
// Функциии для проверки инициализации библиотек====
//==================================================

//Проверка OpenGL
bool checkOpenGLInitialization() {
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "\x1b[31mFailed to initialize GLAD\x1b[0m" << std::endl;
        return false; // Возвращаем false, если произошла ошибка
    }
    return true; // Возвращаем true, если инициализация прошла успешно
}

//Проверка Assimp
bool checkAssimpInitialization() {
    Assimp::Importer importer;
    // Пробуем загрузить несуществующий файл для проверки
    const aiScene* scene = importer.ReadFile("non_existent_file.obj", 0);

    if (scene == nullptr) {
        std::cerr << "\x1b[31mFailed to initialize Assimp: \x1b[0m" << importer.GetErrorString() << std::endl;
        return false; // Возвращаем false, если произошла ошибка
    }
    return true; // Возвращаем true, если инициализация прошла успешно
}

// Проверка GLM
bool checkGLMInitialization() {
    // Проверяем, можем ли мы создать простой вектор и матрицу
    glm::vec3 testVector(1.0f, 2.0f, 3.0f);
    glm::mat4 testMatrix = glm::mat4(1.0f);

    // Проверка значений вектора и матрицы (можно добавить более сложные проверки)
    if (testVector.x != 1.0f || testMatrix[0][0] != 1.0f) {
        std::cerr << "\x1b[31mFailed to initialize GLM\x1b[0m" << std::endl;
        return false; // Возвращаем false, если произошла ошибка
    }
    return true; // Возвращаем true, если инициализация прошла успешно
}

// Проверка всех библиотек
bool EngineCheckInitializations() {
    return checkOpenGLInitialization() && checkAssimpInitialization() && checkGLMInitialization();
}

// Функция получения разрешения экрана для создания окна на разных мониторах
void getScreenResolution(int& width, int& height) {
    // Инициализация GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return;
    }

    // Получение видеодрайвера
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    if (mode) {
        width = mode->width;
        height = mode->height;
    }
    else {
        std::cerr << "Failed to get video mode" << std::endl;
    }

}


//============================================================================
// Реализация методов и прочего класов OpenGLWindow и Engine==================
//============================================================================

//=================================OpenGLWindow===============================
OpenGLWindow::OpenGLWindow(int width, int height, const char* title = nullptr)
    : windowWidth(width), windowHeight(height), windowTitle(title ? title : ""), window(nullptr)
{
    if (!initialize())
    {
        std::cerr << "Failed to initialize OpenGLWindow." << std::endl;
        exit(EXIT_FAILURE);
    }
    setWindowPosition(); // Устанавливаем позицию окна после инициализации
}
OpenGLWindow::~OpenGLWindow()
{
    glfwTerminate();
}

bool OpenGLWindow::initialize()
{
    // Инициализация GLFW
    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW." << std::endl;
        return false;
    }

    // Устанавливаем параметры GLFW
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_DECORATED, GLFW_FALSE); // Убираем рамку и кнопки управления

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // Для macOS
#endif

    // Создание окна
    window = glfwCreateWindow(windowWidth, windowHeight, windowTitle, nullptr, nullptr);
    if (window == nullptr)
    {
        std::cerr << "Failed to create GLFW window." << std::endl;
        glfwTerminate();
        return false;
    }

    // Устанавливаем контекст окна
    glfwMakeContextCurrent(window);

    // Инициализация GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cerr << "Failed to initialize GLAD." << std::endl;
        return false;
    }

    // Задаем размеры области просмотра
    glViewport(0, 0, windowWidth, windowHeight);

    return true;
}
void OpenGLWindow::setWindowPosition()
{
    // Получаем размер экрана
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());

    if (mode)
    {
        // Рассчитываем координаты для правого нижнего угла
        int posX = mode->width - windowWidth;
        int posY = mode->height - windowHeight - 40; // Учитываем высоту панели задач (около 40 пикселей)

        // Устанавливаем позицию окна
        glfwSetWindowPos(window, posX, posY);
    }
}
void OpenGLWindow::processInput()
{
    // Закрытие окна при нажатии ESC
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
    {
        glfwSetWindowShouldClose(window, true);
    }
}
void OpenGLWindow::render()
{
    // Очищаем экран (меняем цвет фона)
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);


}
void OpenGLWindow::run()
{
    // Основной цикл окна
    while (!glfwWindowShouldClose(window))
    {
        // Обработка ввода
        processInput();

        // Рендеринг кадра
        render();

        // Меняем буферы
        glfwSwapBuffers(window);

        // Обрабатываем события
        glfwPollEvents();
    }
}


//======================Engine=======================

// Конструктор Engine с инициализацией объектов Camera и OpenGLWindow
Engine::Engine(float cameraPosX, float cameraPosY, float cameraPosZ,
    float upX, float upY, float upZ, float yaw, float pitch,
    int windowWidth, int windowHeight)
    : camera(cameraPosX, cameraPosY, cameraPosZ, upX, upY, upZ, yaw, pitch),
    window(windowWidth, windowHeight) {
    // Здесь можно добавить дополнительную логику для Engine
}

//===================private часть===================
std::string Engine::loadShaderSource(const std::string& shaderName) {
    std::string path = "../ASCIIModel/" + shaderName;
    std::ifstream shaderFile(path);
    if (!shaderFile) {
        std::cerr << "\x1b[31mFailed to open shader file: \x1b[0m" << path << std::endl;
        return "";
    }

    // Чтение файла напрямую в строку
    std::string shaderSource((std::istreambuf_iterator<char>(shaderFile)),
        std::istreambuf_iterator<char>());
    return shaderSource;
}

bool Engine::compileShader(const std::string& vertexSource, const std::string& fragmentSource) 
{
    return false;
}
void Engine::runWindow() {
    window.run();
    render();
}
bool Engine::loadTextures() {
    return false;
}
void Engine::loadModelData(const std::string& filepath) {

}
void Engine::updateAnimation(float deltaTime) {

}
void Engine::applyTransformations() {

}
void Engine::updateLights() {

}
void Engine::renderScene() {

}
void Engine::applyShaderUniforms() {

}
void Engine::cleanup() {

}
void Engine::finalization() {

}
int  Engine::initialization() {
    return 1;
}

//===================public часть===================
bool Engine::loadModel(const std::string& filepath) {
    return false;
}
void Engine::setAnimation(const std::string& animationName) {

}
void Engine::update(float deltaTime) {

}
void Engine::render() {
    // Цикл рендеринга

    // Логическая часть работы со временем для каждого кадра


    // Рендеринг

    // Убеждаемся, что активировали шейдер прежде, чем настраивать uniform-переменные/объекты_рисования

    // Преобразования Вида/Проекции

    // Рендеринг загруженной модели



}
void Engine::shutdown() {

}



//======================Camera=======================

    // Конструктор, использующий векторы
Camera::Camera(glm::vec3 position, glm::vec3 up, float yaw, float pitch)
    : Front(glm::vec3(0.0f, 0.0f, -1.0f)),
    MovementSpeed(SPEED),
    MouseSensitivity(SENSITIVITY),
    Zoom(ZOOM)
{
    Position = position;
    WorldUp = up;
    Yaw = yaw;
    Pitch = pitch;
    updateCameraVectors();
}

// Конструктор, использующий скаляры
Camera::Camera(float posX, float posY, float posZ, float upX, float upY, float upZ, float yaw, float pitch)
    : Front(glm::vec3(0.0f, 0.0f, -1.0f)), MovementSpeed(SPEED), MouseSensitivity(SENSITIVITY), Zoom(ZOOM) {
    Position = glm::vec3(posX, posY, posZ);
    WorldUp = glm::vec3(upX, upY, upZ);
    Yaw = yaw;
    Pitch = pitch;
    updateCameraVectors();
}

//===================private часть===================

void Camera::updateCameraVectors()
{
    // Вычисляем новый вектор-прямо
    glm::vec3 front;
    front.x = cos(glm::radians(Yaw)) * cos(glm::radians(Pitch));
    front.y = sin(glm::radians(Pitch));
    front.z = sin(glm::radians(Yaw)) * cos(glm::radians(Pitch));
    Front = glm::normalize(front);

    // Также пересчитываем вектор-вправо и вектор-вверх
    Right = glm::normalize(glm::cross(Front, WorldUp)); // нормализуем векторы, потому что их длина стремится к 0 тем больше, чем больше вы смотрите вверх или вниз, что приводит к более медленному движению
    Up = glm::normalize(glm::cross(Right, Front));
}

//===================public часть===================
glm::mat4 Camera::GetViewMatrix()
{
    return glm::lookAt(Position, Position + Front, Up);
}


void Camera::ProcessKeyboard(Camera_Movement direction, float deltaTime)
{
    float velocity = MovementSpeed * deltaTime;
    if (direction == FORWARD)
        Position += Front * velocity;
    if (direction == BACKWARD)
        Position -= Front * velocity;
    if (direction == LEFT)
        Position -= Right * velocity;
    if (direction == RIGHT)
        Position += Right * velocity;
}





