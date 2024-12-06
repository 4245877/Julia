#include <glad/glad.h>
#include "LoadWindow.h"
#include "ASCIIRenderer.h"

// Тайминги
float deltaTime = 0.0f;
float lastFrame = 0.0f;

// Освещение
glm::vec3 lightPos(1.2f, 1.0f, 2.0f);


int main() {
    EngineCheckInitializations();
    try {
        LoadWindow customWindow;
        customWindow.run();
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }


    int screenWidth, screenHeight;
    getScreenResolution(screenWidth, screenHeight);

    Engine engine(0.0f, 0.0f, 3.0f, 0.0f, 1.0f, 0.0f, -90.0f, 0.0f, screenWidth * 0.25, screenHeight * 0.43);
    engine.runWindow();
    engine.render();



    return 0;
}


