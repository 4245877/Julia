#ifndef CUSTOMWINDOW_H
#define CUSTOMWINDOW_H

#include <SFML/Graphics.hpp>

class CustomWindow
{
public:
    CustomWindow();
    void run();

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
