// ASCIIRenderer.h
#pragma once

#include "ModelData.h"
#include <chrono>
#include <thread>

class ASCIIRenderer {
public:
    ASCIIRenderer();

    void animate(int durationInSeconds) const;

private:
    ModelData modelData;
};

// ASCIIRenderer.cpp
#include "ASCIIRenderer.h"
#include <iostream>

ASCIIRenderer::ASCIIRenderer() : modelData() {}

void ASCIIRenderer::animate(int durationInSeconds) const {
    auto startTime = std::chrono::steady_clock::now();
    int frameNumber = 0;

    while (true) {
        auto currentTime = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count() > durationInSeconds) {
            break;
        }

        auto frame = modelData.getFrame(frameNumber);
        for (const auto& line : frame) {
            std::cout << line << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500)); // задержка между кадрами
        std::cout << "\x1B[3J\x1B[H\x1B[2J"; // очищаем консоль

        frameNumber = (frameNumber + 1) % 2; // переключаемся между кадрами
    }
}
