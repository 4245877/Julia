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

        // �������� ANSI �������������������
        std::string stripAnsiCodes(const std::string& input) {
            static const std::regex ansiRegex("\x1B\\[[0-9;]*[a-zA-Z]");
            return std::regex_replace(input, ansiRegex, "");
        }

    public:
        BufferRedirect(std::streambuf* origBuffer, std::ostream& stream, std::ofstream& file)
            : originalBuffer(origBuffer), targetStream(stream), outputFile(file) {
        }

        // ���������� ��� ������ ������
        virtual int overflow(int c) override {
            if (c != EOF) {
                char ch = static_cast<char>(c);
                // ����� ������� � ���� ��� ANSI �������������������
                std::string str(1, ch);
                std::string strippedStr = stripAnsiCodes(str);

                if (!strippedStr.empty()) {
                    outputFile << strippedStr;
                    outputFile.flush();
                }

                // ������� ������ � ������������ �����
                originalBuffer->sputc(ch);
            }
            return c;
        }

        virtual std::streamsize xsputn(const char* s, std::streamsize n) override {
            // ����� ������ � ���� ��� ANSI �������������������
            std::string input(s, n);
            std::string strippedStr = stripAnsiCodes(input);

            if (!strippedStr.empty()) {
                outputFile << strippedStr;
                outputFile.flush();
            }

            // ������� ������ � ������������ �����
            return originalBuffer->sputn(s, n);
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
