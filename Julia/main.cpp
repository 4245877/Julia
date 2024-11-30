#include <glad/glad.h>
#include "LoadWindow.h"
#include "ASCIIRenderer.h"


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

    Engine engine(0.0f, 0.0f, 3.0f, 0.0f, 1.0f, 0.0f, -90.0f, 0.0f, screenWidth * 0.15, screenHeight * 0.3, "");
    engine.runWindow();


    return 0;
}


