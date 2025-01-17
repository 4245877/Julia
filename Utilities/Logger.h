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
    // �����������, ���������������� std::cout � std::cerr � ����
    ConsoleInterceptor(const std::string& outputFile) {
        outFile.open(outputFile, std::ios::out | std::ios::app);
        if (!outFile.is_open()) {
            std::cerr << "�� ������� ������� ���� ��� ������" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        // ��������� ����������� ������ � �������������� �� � ����
        coutBuf = std::cout.rdbuf();
        cerrBuf = std::cerr.rdbuf();
        std::cout.rdbuf(outFile.rdbuf());
        std::cerr.rdbuf(outFile.rdbuf());
    }

    // ����������, ����������������� std::cout � std::cerr
    ~ConsoleInterceptor() {
        if (outFile.is_open()) {
            // ��������������� ������
            std::cout.rdbuf(coutBuf);
            std::cerr.rdbuf(cerrBuf);
            outFile.close();
        }
    }
};
