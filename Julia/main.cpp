#include "LoadWindow.h"
#include <iostream>

int main()
{
    try {
        CustomWindow customWindow;
        customWindow.run();
    }
    catch (const std::exception& e) {
        // Обробка помилок
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
