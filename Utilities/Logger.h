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

        // ���������� ��� ������ ������
        virtual int overflow(int c) override {
            if (c != EOF) {
                // ����� ������� � ����
                outputFile.put(static_cast<char>(c));
                outputFile.flush();
                // ������� ������� � ������������ �����
                originalBuffer->sputc(static_cast<char>(c));
            }
            return c;
        }
    };

    BufferRedirect* redirectCout;
    BufferRedirect* redirectCerr;

public:
    // �����������
    explicit ConsoleCapture(const std::string& filePath) {
        // ��������� ���-����
        logFile.open(filePath, std::ios::out | std::ios::trunc);
        if (!logFile.is_open()) {
            throw std::runtime_error("Failed to open log file");
        }

        // ��������� ������������ ������
        originalCout = std::cout.rdbuf();
        originalCerr = std::cerr.rdbuf();

        // ������������� ���� ��������� ������
        redirectCout = new BufferRedirect(originalCout, std::cout, logFile);
        redirectCerr = new BufferRedirect(originalCerr, std::cerr, logFile);

        std::cout.rdbuf(redirectCout);
        std::cerr.rdbuf(redirectCerr);
    }

    // ����������
    ~ConsoleCapture() {
        // ��������������� ������������ ������
        std::cout.rdbuf(originalCout);
        std::cerr.rdbuf(originalCerr);

        // ������� ��������� ������
        delete redirectCout;
        delete redirectCerr;

        // ��������� ���-����
        if (logFile.is_open()) {
            logFile.close();
        }
    }
};
