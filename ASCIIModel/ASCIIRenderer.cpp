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
// Реализация методов и прочего Класов OpenGLWindow и Engine==================
//============================================================================


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


//==================================================
//======================Engine======================
//==================================================


Engine::Engine(int width, int height)
    : window(width, height) {}
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
void Engine::runWindow() {
    window.run();
}