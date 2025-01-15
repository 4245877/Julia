#pragma once
#include <iostream>
#include <fstream>
#include <streambuf>
#include <string>

class ConsoleLogger {
private:
    std::ofstream logFile;
    std::streambuf* oldCoutBuffer;
    std::streambuf* oldCerrBuffer;

public:
    // Конструктор: открывает файл и перенаправляет потоки
    ConsoleLogger(const std::string& filename) {
        // Открытие файла
        logFile.open(filename, std::ios::out | std::ios::app);
        if (!logFile.is_open()) {
            throw std::ios_base::failure("Не удалось открыть файл для записи.");
        }

        // Сохраняем текущие буферы
        oldCoutBuffer = std::cout.rdbuf();
        oldCerrBuffer = std::cerr.rdbuf();

        // Перенаправляем std::cout и std::cerr в файл
        std::cout.rdbuf(logFile.rdbuf());
        std::cerr.rdbuf(logFile.rdbuf());
    }

    // Деструктор: восстанавливает потоки и закрывает файл
    ~ConsoleLogger() {
        // Восстанавливаем старые буферы
        std::cout.rdbuf(oldCoutBuffer);
        std::cerr.rdbuf(oldCerrBuffer);

        // Закрываем файл
        if (logFile.is_open()) {
            logFile.close();
        }
    }
};