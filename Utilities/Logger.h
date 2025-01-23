#pragma once
#include <iostream>
#include <fstream>
#include <streambuf>
#include <regex>
#include <string>

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

        // Удаление ANSI последовательностей
        std::string stripAnsiCodes(const std::string& input) {
            static const std::regex ansiRegex("\x1B\\[[0-9;]*[a-zA-Z]");
            return std::regex_replace(input, ansiRegex, "");
        }

    public:
        BufferRedirect(std::streambuf* origBuffer, std::ostream& stream, std::ofstream& file)
            : originalBuffer(origBuffer), targetStream(stream), outputFile(file) {
        }

        // Перегрузка для записи данных
        virtual int overflow(int c) override {
            if (c != EOF) {
                char ch = static_cast<char>(c);
                // Пишем символы в файл без ANSI последовательностей
                std::string str(1, ch);
                std::string strippedStr = stripAnsiCodes(str);

                if (!strippedStr.empty()) {
                    outputFile << strippedStr;
                    outputFile.flush();
                }

                // Передаём символ в оригинальный поток
                originalBuffer->sputc(ch);
            }
            return c;
        }

        virtual std::streamsize xsputn(const char* s, std::streamsize n) override {
            // Пишем строку в файл без ANSI последовательностей
            std::string input(s, n);
            std::string strippedStr = stripAnsiCodes(input);

            if (!strippedStr.empty()) {
                outputFile << strippedStr;
                outputFile.flush();
            }

            // Передаём строку в оригинальный поток
            return originalBuffer->sputn(s, n);
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
