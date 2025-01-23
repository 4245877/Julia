#pragma once
#include <iostream>
#include <fstream>
#include <streambuf>

class ConsoleCapture {
private:
    std::ofstream logFile;
    std::streambuf* originalCout;
    std::streambuf* originalCerr;

    class BufferRedirect : public std::streambuf {
    private:
        std::streambuf* originalBuffer;
        std::ostream& targetStream;
        std::ofstream& outputFile;

    public:
        BufferRedirect(std::streambuf* origBuffer, std::ostream& stream, std::ofstream& file)
            : originalBuffer(origBuffer), targetStream(stream), outputFile(file) {
        }

        // Перегрузка для записи данных
        virtual int overflow(int c) override {
            if (c != EOF) {
                // Пишем символы в файл
                outputFile.put(static_cast<char>(c));
                outputFile.flush();
                // Передаём символы в оригинальный поток
                originalBuffer->sputc(static_cast<char>(c));
            }
            return c;
        }
    };

    BufferRedirect* redirectCout;
    BufferRedirect* redirectCerr;

public:
    // Конструктор
    explicit ConsoleCapture(const std::string& filePath) {
        // Открываем лог-файл
        logFile.open(filePath, std::ios::out | std::ios::trunc);
        if (!logFile.is_open()) {
            throw std::runtime_error("Failed to open log file");
        }

        // Сохраняем оригинальные буферы
        originalCout = std::cout.rdbuf();
        originalCerr = std::cerr.rdbuf();

        // Устанавливаем наши кастомные буферы
        redirectCout = new BufferRedirect(originalCout, std::cout, logFile);
        redirectCerr = new BufferRedirect(originalCerr, std::cerr, logFile);

        std::cout.rdbuf(redirectCout);
        std::cerr.rdbuf(redirectCerr);
    }

    // Деструктор
    ~ConsoleCapture() {
        // Восстанавливаем оригинальные буферы
        std::cout.rdbuf(originalCout);
        std::cerr.rdbuf(originalCerr);

        // Удаляем кастомные буферы
        delete redirectCout;
        delete redirectCerr;

        // Закрываем лог-файл
        if (logFile.is_open()) {
            logFile.close();
        }
    }
};
