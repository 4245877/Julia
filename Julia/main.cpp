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

    Engine engine(screenWidth * 0.15, screenHeight * 0.3);
    engine.runWindow();


    return 0;
}


