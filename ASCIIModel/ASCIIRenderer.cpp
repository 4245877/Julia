#include "ASCIIRenderer.h"

//==================================================
// Функциии для проверки инициализации библиотек====
//==================================================

//Проверка OpenGL
bool checkOpenGLInitialization() {
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "\x1b[31mERROR:\x1b[0m Failed to initialize GLAD" << std::endl;
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
        std::cerr << "\x1b[31mERROR:\x1b[0m Failed to initialize Assimp" << importer.GetErrorString() << std::endl;
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
        std::cerr << "\x1b[31mERROR:\x1b[0m Failed to initialize GLM" << std::endl;
        return false; // Возвращаем false, если произошла ошибка
    }
    return true; // Возвращаем true, если инициализация прошла успешно
}

// Проверка всех библиотек
bool EngineCheckInitializations() {
    return checkOpenGLInitialization() && checkAssimpInitialization() && checkGLMInitialization();
}

// Функция получения разрешения экрана для создания окна на разных мониторах
void getScreenResolution(float& width, float& height) {
    // Инициализация GLFW
    if (!glfwInit()) {
        std::cerr << "\x1b[31mERORR:\x1b[0m Failed to initialize GLFW" << std::endl;
        return;
    }

    // Получение видеодрайвера
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    if (mode) {
        width = mode->width;
        height = mode->height;
    }
    else {
        std::cerr << "\x1b[31mERORR:\x1b[0m Failed to get video mode" << std::endl;
    }

}


//============================================================================
// Реализация методов и прочего класов OpenGLWindow и Engine==================
//============================================================================

//=================================OpenGLWindow===============================
OpenGLWindow::OpenGLWindow(float width, float height, const char* title = nullptr)
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
        std::cerr << "\x1b[31mERROR:\x1b[0m failed to initialize GLFW." << std::endl;
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
    float windowWidth, float windowHeight)
	: camera(cameraPosX, cameraPosY, cameraPosZ, upX, upY, upZ, yaw, pitch), window(windowWidth, windowHeight)
{}

//===================private часть===================
std::string Engine::loadShaderSource(const std::string& shaderName) {
   
    return "1";
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

void Engine::setAnimation(const std::string& animationName) {

}
void Engine::update(float deltaTime) {

}
void Engine::render() {
    static bool firstRun = true;
    static GLuint shaderProgram, VAO, VBO;

    if (firstRun) {
        // Шейдеры
        const char* vertexShaderSource = R"glsl(
            #version 330 core
            layout (location = 0) in vec3 aPos;
            uniform mat4 projection;
            uniform mat4 view;
            void main() {
                gl_Position = projection * view * vec4(aPos, 1.0);
            }
        )glsl";

        const char* fragmentShaderSource = R"glsl(
            #version 330 core
            out vec4 FragColor;
            void main() {
                FragColor = vec4(1.0, 0.0, 0.0, 1.0);
            }
        )glsl";

        // Компиляция шейдеров
        GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
        glCompileShader(vertexShader);

        GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
        glCompileShader(fragmentShader);

        // Линковка программы
        shaderProgram = glCreateProgram();
        glAttachShader(shaderProgram, vertexShader);
        glAttachShader(shaderProgram, fragmentShader);
        glLinkProgram(shaderProgram);

        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);

        // Вершины треугольника
        float vertices[] = {
            -0.5f, -0.5f, 0.0f,
             0.5f, -0.5f, 0.0f,
             0.0f,  0.5f, 0.0f
        };

        // Настройка VAO и VBO
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);

        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);

        firstRun = false;
    }

    // Получаем матрицы вида и проекции
    glm::mat4 view = camera.GetViewMatrix();
    glm::mat4 projection = glm::perspective(
        glm::radians(45.0f),
        (float)window.GetWindowWidth() / (float)window.GetWindowHeight(),
        0.1f, 100.0f
    );

    // Активируем шейдер и передаем uniform-переменные
    glUseProgram(shaderProgram);
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

    // Отрисовка треугольника
    glBindVertexArray(VAO);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindVertexArray(0);
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





