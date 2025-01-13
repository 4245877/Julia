#pragma once
#include <iostream>
#include <fstream>
#include <streambuf>
#include <string>

class TeeStreamBuffer : public std::streambuf {
private:
    std::streambuf* consoleBuffer;  // Буфер консоли
    std::streambuf* fileBuffer;     // Буфер файла

public:
    TeeStreamBuffer(std::streambuf* consoleBuf, std::streambuf* fileBuf)
        : consoleBuffer(consoleBuf), fileBuffer(fileBuf) {
    }

    // Метод для получения оригинального буфера
    std::streambuf* getConsoleBuffer() const {
        return consoleBuffer;
    }

protected:
    virtual int overflow(int c) override {
        if (c != EOF) {
            if (consoleBuffer) consoleBuffer->sputc(c); // Пишем в консоль
            if (fileBuffer) fileBuffer->sputc(c);       // Пишем в файл
        }
        return c;
    }

    virtual int sync() override {
        int result = 0;
        if (consoleBuffer) result = consoleBuffer->pubsync();
        if (fileBuffer) result = fileBuffer->pubsync();
        return result;
    }
};
