#include <iostream>
#include <string>
#include "AICore.h"

int main() {
    AICore ai;

    std::string input;
    while (true) {
        std::cout << "> ";
        std::getline(std::cin, input);
        if (input == "выход") break;

        std::cout << ai.processInput(input) << std::endl;
    }

    return 0;
}
