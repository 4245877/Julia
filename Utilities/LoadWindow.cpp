#include "LoadWindow.h"

CustomWindow::CustomWindow() :
    gradient(sf::TrianglesStrip, 4)
{
    initializeWindow();
    initializeText();
    initializeGradient();
}

void CustomWindow::initializeWindow()
{
    // �������� ������ ������
    sf::VideoMode desktop = sf::VideoMode::getDesktopMode();
    int screenWidth = desktop.width;
    int screenHeight = desktop.height;

    // ���������� ������ ���� (35% �� ����� ������)
    int windowWidth = static_cast<int>(screenWidth * 0.35f);
    int windowHeight = static_cast<int>(screenHeight * 0.35f);

    // ��������� ���� ��� ����� � ���������
    window.create(sf::VideoMode(windowWidth, windowHeight), "", sf::Style::None);

    // �������� ���� �� ������� ������
    window.setPosition(sf::Vector2i((screenWidth - windowWidth) / 2, (screenHeight - windowHeight) / 2));
}

void CustomWindow::initializeText()
{
    // ����������� ����� Bebas Neue � ����� Fonts
    if (!font.loadFromFile("../Utilities/Fonts/BebasNeue-Regular.ttf")) {
        throw std::runtime_error("Failed to load font");
    }

    // ��������� ����� "Julia"
    text.setFont(font);
    text.setString("Julia");
    text.setCharacterSize(100);

    // ������� ������ � ����� ����
    sf::FloatRect textRect = text.getLocalBounds();
    text.setOrigin(textRect.left + textRect.width / 2.0f, textRect.top + textRect.height / 2.0f);
    text.setPosition(window.getSize().x / 2.0f, window.getSize().y / 2.0f);
}

void CustomWindow::initializeGradient()
{
    // ��������� ���䳺�� �������
    gradient[0].position = sf::Vector2f(0, 0);
    gradient[0].color = sf::Color(0xff, 0x8e, 0x44);

    gradient[1].position = sf::Vector2f(window.getSize().x, 0);
    gradient[1].color = sf::Color(0xf9, 0x13, 0x62);

    gradient[2].position = sf::Vector2f(0, window.getSize().y);
    gradient[2].color = sf::Color(0x35, 0x12, 0x6a);

    gradient[3].position = sf::Vector2f(window.getSize().x, window.getSize().y);
    gradient[3].color = sf::Color(0x35, 0x12, 0x6a);
}

void CustomWindow::handleEvents()
{
    sf::Event event;
    while (window.pollEvent(event))
    {
        // �������� ���� ��� ��������� �� ������ ESC ��� ����� ���� ��������
        if (event.type == sf::Event::Closed ||
            (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Escape))
            window.close();
    }
}

void CustomWindow::draw()
{
    // ������� ���� ���� ��������
    window.clear(sf::Color::White);

    // ������� ���䳺��
    window.draw(gradient);

    // ������� �����
    window.draw(text);

    // ³��������� ���� ����
    window.display();
}

void CustomWindow::run()
{
    while (window.isOpen())
    {
        handleEvents();
        draw();
    }
}
