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
    // �����������: ��������� ���� � �������������� ������
    ConsoleLogger(const std::string& filename) {
        // �������� �����
        logFile.open(filename, std::ios::out | std::ios::app);
        if (!logFile.is_open()) {
            throw std::ios_base::failure("�� ������� ������� ���� ��� ������.");
        }

        // ��������� ������� ������
        oldCoutBuffer = std::cout.rdbuf();
        oldCerrBuffer = std::cerr.rdbuf();

        // �������������� std::cout � std::cerr � ����
        std::cout.rdbuf(logFile.rdbuf());
        std::cerr.rdbuf(logFile.rdbuf());
    }

    // ����������: ��������������� ������ � ��������� ����
    ~ConsoleLogger() {
        // ��������������� ������ ������
        std::cout.rdbuf(oldCoutBuffer);
        std::cerr.rdbuf(oldCerrBuffer);

        // ��������� ����
        if (logFile.is_open()) {
            logFile.close();
        }
    }
};