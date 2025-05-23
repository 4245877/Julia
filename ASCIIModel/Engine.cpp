#include "Engine.h"
#include "Camera.h"
// 
// Функциии для проверки инициализации библиотек 
// 

//Проверка OpenGL
bool checkOpenGLInitialization() {
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "\x1b[31mERROR:\x1b[0m Failed to initialize GLAD" << std::endl;
        return false; // Возвращаем false, если произошла ошибка
    }
    return true; // Возвращаем true, если инициализация прошла успешно 31mERORR
}

// Проверка Assimp
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
        std::cerr << "\x1b[31mERROR:\x1b[0m Failed to initialize GLFW" << std::endl;
        return;
    }

    // Получение видеодрайвера
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    if (mode) {
        width = mode->width;
        height = mode->height;
    }
    else {
        std::cerr << "\x1b[31mERROR:\x1b[0m Failed to get video mode" << std::endl;
    }

}


// 
// Реализация методов и прочего в класе Engine 
// 





// Конструктор Engine с инициализацией объектов Camera и OpenGLWindow
Engine::Engine(float cameraPosX, float cameraPosY, float cameraPosZ,
    float upX, float upY, float upZ, float yaw, float pitch,
    float windowWidth, float windowHeight)
    : camera(cameraPosX, cameraPosY, cameraPosZ, upX, upY, upZ, yaw, pitch),
    window(windowWidth, windowHeight),
    shaderProgram(0), VAO(0), VBO(0) // Инициализируем нулями для безопасности
{
    // Предполагаем, что window уже создала контекст OpenGL
    // Теперь можно инициализировать графические ресурсы
    initializeGraphics();
}


// private часть 

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

// public часть 

void Engine::setAnimation(const std::string& animationName) {

}
void Engine::update(float deltaTime) {

}

void Engine::render() {

    // Получаем матрицы вида и проекции
    glm::mat4 view = camera.GetViewMatrix();
    glm::mat4 projection = glm::perspective(
        glm::radians(45.0f), // Угол обзора
        (float)window.GetWindowWidth() / (float)window.GetWindowHeight(), // Соотношение сторон
        0.1f,  // Ближняя плоскость отсечения
        100.0f // Дальняя плоскость отсечения
    );

    // Активируем шейдерную программу
    glUseProgram(this->shaderProgram);

    // Передаем uniform-переменные (матрицы) в шейдер
    glUniformMatrix4fv(glGetUniformLocation(this->shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(glGetUniformLocation(this->shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

    // Отрисовка треугольника
    glBindVertexArray(this->VAO); // Активируем VAO
    glDrawArrays(GL_TRIANGLES, 0, 3); // Рисуем треугольник
    glBindVertexArray(0); // Отвязываем VAO
}
void Engine::shutdown() {

}


void Engine::initializeGraphics() {
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
            FragColor = vec4(1.0, 0.0, 0.0, 1.0); // Красный цвет
        }
    )glsl";

    // Компиляция вершинного шейдера
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);
    // Проверка на ошибки компиляции вершинного шейдера
    int success;
    char infoLog[512];
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
        std::cerr << "\x1b[31mERROR::SHADER::VERTEX::COMPILATION_FAILED\x1b[0m\n" << infoLog << std::endl;
    }

    // Компиляция фрагментного шейдера
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);
    // Проверка на ошибки компиляции фрагментного шейдера
    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
        std::cerr << "\x1b[31mERROR::SHADER::FRAGMENT::COMPILATION_FAILED\x1b[0m\n" << infoLog << std::endl;
    }

    // Линковка шейдерной программы
    this->shaderProgram = glCreateProgram();
    glAttachShader(this->shaderProgram, vertexShader);
    glAttachShader(this->shaderProgram, fragmentShader);
    glLinkProgram(this->shaderProgram);
    // Проверка на ошибки линковки
    glGetProgramiv(this->shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(this->shaderProgram, 512, NULL, infoLog);
        std::cerr << "\x1b[31mERROR::SHADER::PROGRAM::LINKING_FAILED\x1b[0m\n" << infoLog << std::endl;
    }

    // После линковки шейдеры нам больше не нужны
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    // Вершины треугольника
    float vertices[] = {
        -0.5f, -0.5f, 0.0f, // левая  
         0.5f, -0.5f, 0.0f, // правая 
         0.0f,  0.5f, 0.0f  // верхняя
    };

    // Настройка VAO и VBO
    glGenVertexArrays(1, &this->VAO);
    glGenBuffers(1, &this->VBO);

    glBindVertexArray(this->VAO);

    glBindBuffer(GL_ARRAY_BUFFER, this->VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    // Указываем OpenGL, как интерпретировать вершинные данные
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0); // Включаем атрибут вершины с location = 0

    // Отвязываем VBO (VAO все еще активен и хранит ссылку на этот VBO)
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    // Отвязываем VAO
    glBindVertexArray(0);
}


