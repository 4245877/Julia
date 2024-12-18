#ifndef CUSTOMWINDOW_H
#define CUSTOMWINDOW_H

#include <SFML/Graphics.hpp>
#include <SFML/Audio.hpp>
#include <iostream>
#include <cstdlib>
#include <ctime>
class LoadWindow{
private:
    sf::RenderWindow window;
    sf::Font font;
    sf::Text text;
    sf::VertexArray gradient;
    std::vector<std::string> loadingStages = {
        "Инициализация приложения...",
        "Загрузка ресурсов...",
        "Подключение к базе данных...",
        "Настройка интерфейса..."
    };
    int currentStage = 0;

    void initializeWindow();
    void initializeText();
    void initializeGradient();
    void handleEvents();
    void draw();

public:
    LoadWindow();
    ~LoadWindow();

    void run();
    void LoadSound();
    void updateLoadingStage();
};
#endif // CUSTOMWINDOW_H
