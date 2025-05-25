#include "Engine.h"
#include "Camera.h"

// Инициализация статического члена
bool Engine::glfwInitialized = false;


bool Engine::InitializeGLFW() {
    if (!glfwInitialized) {
        if (!glfwInit()) {
            std::cerr << "\x1b[31mERROR:\x1b[0m Failed to initialize GLFW" << std::endl;
            return false;
        }
        glfwInitialized = true;
        std::cout << "GLFW initialized successfully." << std::endl;
        // Здесь можно добавить базовые GLFW window hints, если они глобальны для приложения
        // glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        // glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        // glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        // #ifdef __APPLE__
        // glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
        // #endif
    }
    return true;
}

// Статический метод для проверки, инициализирован ли GLFW
bool Engine::IsGLFWInitialized() {
    return glfwInitialized;
}

// Статический метод для завершения работы GLFW
void Engine::TerminateGLFW() {
    if (glfwInitialized) {
        glfwTerminate();
        glfwInitialized = false;
        std::cout << "GLFW terminated successfully." << std::endl;
    }
}


// 
// Функциии для проверки инициализации библиотек 
// 

//Проверка OpenGL
bool checkOpenGLInitialization() {
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "\x1b[31mERROR:\x1b[0m Failed to initialize GLAD" << std::endl;
        return false; // Возвращаем false, если произошла ошибка
    }
    return true; // Возвращаем true, если инициализация прошла успешно 
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
    // Важно: Эта функция теперь ожидает, что Engine::InitializeGLFW() уже был вызван!
    if (!Engine::IsGLFWInitialized()) {
        std::cerr << "\x1b[33mWARNING:\x1b[0m getScreenResolution called before GLFW was initialized by Engine! Attempting to initialize now..." << std::endl;
        if (!Engine::InitializeGLFW()) { // Попытка инициализировать, если это не было сделано
            std::cerr << "\x1b[31mERROR:\x1b[0m Failed to initialize GLFW in getScreenResolution. Using default resolution." << std::endl;
            width = 800.0f; // Значения по умолчанию
            height = 600.0f;
            return;
        }
    }

    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    if (mode) {
        width = static_cast<float>(mode->width);
        height = static_cast<float>(mode->height);
    }
    else {
        std::cerr << "\x1b[31mERROR:\x1b[0m Failed to get video mode. Using default resolution." << std::endl;
        width = 800.0f; // Значения по умолчанию
        height = 600.0f;
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
    window(windowWidth, windowHeight), // OpenGLWindow создается здесь
    shaderProgram(0), VAO(0), VBO(0)
{
 
    // Инициализация графики после того, как окно и контекст созданы
    // и callback установлен (хотя установка callback не зависит от initializeGraphics)
    initializeGraphics();
}


// private часть 

std::string Engine::loadShaderSource(const std::string& filePath) {
    std::ifstream shaderFile;
    // Устанавливаем исключения для ifstream, чтобы он выбрасывал их при ошибках
    shaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    try {
        // Открываем файл
        shaderFile.open(filePath);
        std::stringstream shaderStream;
        // Читаем содержимое файла в строковый поток
        shaderStream << shaderFile.rdbuf();
        // Закрываем файл
        shaderFile.close();
        // Преобразуем строковый поток в std::string
        return shaderStream.str();
    }
    catch (const std::ifstream::failure& e) {
        // Используем \x1b[31m для красного цвета ошибки в консоли, если она поддерживает ANSI-последовательности
        std::cerr << "\x1b[31mERROR::SHADER::FILE_NOT_SUCCESSFULLY_READ\x1b[0m: " << filePath << "\n" << e.what() << std::endl;
        return ""; // Возвращаем пустую строку в случае ошибки
    }
}

bool Engine::compileShader(const std::string& vertexSource, const std::string& fragmentSource) 
{
    return false;
}
void Engine::runWindow() {
    // Устанавливаем коллбэк перед запуском цикла окна
    window.setRenderCallback([this]() { this->render(); }); // << РАСКОММЕНТИРОВАТЬ И ИСПОЛЬЗОВАТЬ
    window.run(); // Теперь window.run() будет вызывать Engine::render() в каждой итерации
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
    // Освобождаем ресурсы VAO, VBO и шейдерной программы
    glDeleteVertexArrays(1, &this->VAO);
    glDeleteBuffers(1, &this->VBO);
    glDeleteProgram(this->shaderProgram);

    // Здесь также может быть другая логика очистки, если необходимо
    std::cout << "Engine cleanup: VAO, VBO, Shader Program deleted." << std::endl;
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
    // Устанавливаем цвет для очистки экрана (например, темно-бирюзовый)
    // Это можно делать один раз после создания контекста, но для теста можно и здесь
    glClearColor(0.1f, 0.2f, 0.2f, 1.0f);
    // Очищаем буфер цвета (и буфер глубины, если он используется)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Если используешь тест глубины

    // Получаем матрицы вида и проекции
    glm::mat4 view = camera.GetViewMatrix();
    glm::mat4 projection = glm::perspective(
        glm::radians(45.0f), // Угол обзора
        (float)window.GetWindowWidth() / (float)window.GetWindowHeight(), // Соотношение сторон
        0.1f,  // Ближняя плоскость отсечения
        1000.0f // Дальняя плоскость отсечения
    );

    // Проверка валидности шейдерной программы (для отладки)
    if (this->shaderProgram == 0) {
        std::cerr << "\x1b[31mERROR::ENGINE::RENDER\x1b[0m: Используется невалидная шейдерная программа (ID 0). Рендеринг прерван." << std::endl;
        return; // Выход, если шейдеры не скомпилированы/не слинкованы
    }

    // Активируем шейдерную программу
    glUseProgram(this->shaderProgram);

    // Передаем uniform-переменные (матрицы) в шейдер
    // Желательно проверять результат glGetUniformLocation
    GLint viewLoc = glGetUniformLocation(this->shaderProgram, "view");
    GLint projLoc = glGetUniformLocation(this->shaderProgram, "projection");

    if (viewLoc == -1 || projLoc == -1) {
        std::cerr << "\x1b[31mERROR::ENGINE::RENDER\x1b[0m: Не удалось найти uniform-переменные 'view' или 'projection'." << std::endl;
        // Можно добавить больше информации, например, ID программы
    }

    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));

    // Отрисовка треугольника
    glBindVertexArray(this->VAO); // Активируем VAO
    glDrawArrays(GL_TRIANGLES, 0, 3); // Рисуем треугольник
    glBindVertexArray(0); // Отвязываем VAO

    // ВАЖНО: glfwSwapBuffers(windowObject) должен вызываться в конце каждой итерации
    // главного цикла, обычно в методе run() вашего класса окна (OpenGLWindow).
    // Если его там нет, вы не увидите результат рендеринга.
}
void Engine::shutdown() {
    cleanup(); // Вызываем cleanup для освобождения ресурсов   
    Engine::TerminateGLFW(); // Завершение работы GLFW
    std::cout << "Engine shutdown complete." << std::endl;
}



void Engine::initializeGraphics() {
    // 1. Загрузка исходного кода шейдеров из файлов
    std::string vertexShaderCode = loadShaderSource("vertex_shader.glsl");
    std::string fragmentShaderCode = loadShaderSource("fragment_shader.glsl");

    // Проверяем, успешно ли загружены шейдеры
    if (vertexShaderCode.empty() || fragmentShaderCode.empty()) {
        std::cerr << "\x1b[31mERROR::ENGINE::INITIALIZE_GRAPHICS\x1b[0m: Failed to load one or more shader sources. Graphics initialization aborted." << std::endl;
        // Здесь можно предпринять более серьезные действия, например, выбросить исключение
        // или установить флаг ошибки в движке, чтобы предотвратить дальнейшие операции рендеринга.
        return; // Прерываем инициализацию, если шейдеры не загружены
    }

    // Преобразуем std::string в const char* для OpenGL функций
    const char* cVertexShaderSource = vertexShaderCode.c_str();
    const char* cFragmentShaderSource = fragmentShaderCode.c_str();

    // 2. Компиляция шейдеров
    GLuint vertexShader, fragmentShader;
    int success;
    char infoLog[512];

    // Вершинный шейдер
    vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &cVertexShaderSource, NULL);
    glCompileShader(vertexShader);
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
        std::cerr << "\x1b[31mERROR::SHADER::VERTEX::COMPILATION_FAILED\x1b[0m\n" << infoLog << std::endl;
    }

    // Фрагментный шейдер
    fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &cFragmentShaderSource, NULL);
    glCompileShader(fragmentShader);
    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
        std::cerr << "\x1b[31mERROR::SHADER::FRAGMENT::COMPILATION_FAILED\x1b[0m\n" << infoLog << std::endl;
    }

    // 3. Линковка шейдерной программы
    this->shaderProgram = glCreateProgram();
    glAttachShader(this->shaderProgram, vertexShader);
    glAttachShader(this->shaderProgram, fragmentShader);
    glLinkProgram(this->shaderProgram);
    glGetProgramiv(this->shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(this->shaderProgram, 512, NULL, infoLog);
        std::cerr << "\x1b[31mERROR::SHADER::PROGRAM::LINKING_FAILED\x1b[0m\n" << infoLog << std::endl;
    }

    // Удаляем объекты шейдеров после линковки, они больше не нужны
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    // 4. Настройка вершинных данных, VAO и VBO (остается как было)
    float vertices[] = {
        -0.5f, -0.5f, 0.0f, // левая  
         0.5f, -0.5f, 0.0f, // правая 
         0.0f,  0.5f, 0.0f  // верхняя
    };

    glGenVertexArrays(1, &this->VAO);
    glGenBuffers(1, &this->VBO);

    glBindVertexArray(this->VAO);

    glBindBuffer(GL_ARRAY_BUFFER, this->VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}



