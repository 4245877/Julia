#pragma once
#include <iostream>
#include <fstream>
#include <streambuf>

class ConsoleInterceptor {
private:
    std::ofstream outFile;
    std::streambuf* coutBuf;
    std::streambuf* cerrBuf;

public:
    // Конструктор, перенаправляющий std::cout и std::cerr в файл
    ConsoleInterceptor(const std::string& outputFile) {
        outFile.open(outputFile, std::ios::out | std::ios::app);
        if (!outFile.is_open()) {
            std::cerr << "Не удалось открыть файл для записи" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        // Сохраняем стандартные буферы и перенаправляем их в файл
        coutBuf = std::cout.rdbuf();
        cerrBuf = std::cerr.rdbuf();
        std::cout.rdbuf(outFile.rdbuf());
        std::cerr.rdbuf(outFile.rdbuf());
    }

    // Деструктор, восстанавливающий std::cout и std::cerr
    ~ConsoleInterceptor() {
        if (outFile.is_open()) {
            // Восстанавливаем буферы
            std::cout.rdbuf(coutBuf);
            std::cerr.rdbuf(cerrBuf);
            outFile.close();
        }
    }
};
