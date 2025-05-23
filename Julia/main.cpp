#include <glad/glad.h>
#include "LoadWindow.h"
#include "Engine.h"

#include "Logger.h"

// Константы
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

// Камера
Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

// Тайминги
float deltaTime = 0.0f;
float lastFrame = 0.0f;

// Освещение
glm::vec3 lightPos(1.2f, 1.0f, 2.0f);


int main() {
    ConsoleCapture consoleCapture("log.txt");
    EngineCheckInitializations();


    try {
        LoadWindow customWindow;
        customWindow.run();

    }
    catch (const std::exception& e) {
        std::cerr << "\x1b[31mERROR:\x1b[0m " << e.what() << std::endl;
        return 1;
    }
    

    float screenWidth, screenHeight;
    getScreenResolution(screenWidth, screenHeight);

    screenWidth = screenWidth * 0.25;
	screenHeight = screenHeight * 0.43;

    Engine engine(0.0f, 0.0f, 3.0f, 0.0f, 1.0f, 0.0f, -90.0f, 0.0f, screenWidth, screenHeight);
    engine.runWindow();

    return 0;
}