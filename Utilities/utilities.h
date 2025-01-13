#pragma once
#include <iostream>
#include <fstream>
#include <streambuf>
#include <string>

class TeeStreamBuffer : public std::streambuf {
private:
    std::streambuf* consoleBuffer;  // ����� �������
    std::streambuf* fileBuffer;     // ����� �����

public:
    TeeStreamBuffer(std::streambuf* consoleBuf, std::streambuf* fileBuf)
        : consoleBuffer(consoleBuf), fileBuffer(fileBuf) {
    }

    // ����� ��� ��������� ������������� ������
    std::streambuf* getConsoleBuffer() const {
        return consoleBuffer;
    }

protected:
    virtual int overflow(int c) override {
        if (c != EOF) {
            if (consoleBuffer) consoleBuffer->sputc(c); // ����� � �������
            if (fileBuffer) fileBuffer->sputc(c);       // ����� � ����
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
