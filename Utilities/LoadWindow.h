#ifndef CUSTOMWINDOW_H
#define CUSTOMWINDOW_H

#include <SFML/Graphics.hpp>
#include <SFML/Audio.hpp>
#include <iostream>
#include <cstdlib>
#include <ctime>
class LoadWindow
{
public:
    LoadWindow();
    ~LoadWindow();
    void run();
    void LoadSound();

private:
    sf::RenderWindow window;
    sf::Font font;
    sf::Text text;
    sf::VertexArray gradient;

    void initializeWindow();
    void initializeText();
    void initializeGradient();
    void handleEvents();
    void draw();
};

#endif // CUSTOMWINDOW_H
