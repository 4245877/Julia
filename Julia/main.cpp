#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <iostream>
#include <vector>
#include "LoadWindow.h"
#include "ASCIIRenderer.h"


int main() {
    
    try {
        LoadWindow customWindow;
        customWindow.run();
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    OpenGLWindow window(800, 600, "OpenGL Window");
    window.run();


    return 0;
}

