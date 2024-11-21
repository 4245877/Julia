#include <SFML/Graphics.hpp>
#include <cstdlib>
#include <ctime>
#include <string>
#include <memory>
#include <thread>
#include "LoadWindow.h"


LoadWindow::LoadWindow() :
    gradient(sf::TrianglesStrip, 4)
{
    initializeWindow();
    initializeText();
    initializeGradient();
}
// ���������� ����� ��������!!!
LoadWindow:: ~LoadWindow()
{
    LoadSound();
}
void LoadWindow::initializeWindow()
{
    // �������� ������ ������
    sf::VideoMode desktop = sf::VideoMode::getDesktopMode();
    int screenWidth = desktop.width;
    int screenHeight = desktop.height;

    // ���������� ������ ���� (35% �� ����� ������)
    int windowWidth = static_cast<int>(screenWidth * 0.45f);
    int windowHeight = static_cast<int>(screenHeight * 0.45f);

    // ��������� ���� ��� ����� � ���������
    window.create(sf::VideoMode(windowWidth, windowHeight), "", sf::Style::None);

    // �������� ���� �� ������� ������
    window.setPosition(sf::Vector2i((screenWidth - windowWidth) / 2, (screenHeight - windowHeight) / 2));
}
void LoadWindow::initializeText()
{
    // �������� ������ ������
    sf::VideoMode desktop = sf::VideoMode::getDesktopMode();
    short screenWidth = desktop.width;
    short FontSize = static_cast<short>(screenWidth * 0.145f);


    std::string arrFonts[] = {"oldtimer-GOPpg", "bebasNeue-Regular", "reading Regular", "mattoa Demo", "quan-BlackItalic", "sacramento", "Neonderthaw-Regular", "FleurDeLeah-Regular", "Futuristic-Regular", "Ninja District"};
    srand(static_cast<unsigned>(time(0) + 1));
    short FontChoice = rand() % 10;
    std::string FontName = "../Utilities/Fonts/" + arrFonts[FontChoice] + ".ttf";
    if (!font.loadFromFile(FontName)) {
        throw std::runtime_error("Failed to load font");
    }

    //�����
    text.setFont(font);
    text.setString("Julia");

    text.setCharacterSize(FontSize);

    // ������� ������ � ����� ����
    sf::FloatRect textRect = text.getLocalBounds();
    text.setOrigin(textRect.left + textRect.width / 2.0f, textRect.top + textRect.height / 2.0f);
    text.setPosition(window.getSize().x / 2.0f, window.getSize().y / 2.0f);
}
void LoadWindow::initializeGradient()
{
    // ������������� ���������� ��������� �����
    srand(static_cast<unsigned>(time(0)));

    int gradientChoice = rand() % 5;

    // �������� ������ ����
    sf::Vector2f windowSize(window.getSize().x, window.getSize().y);

    // � ����������� �� ���������� �������� ����� ��������
    if (gradientChoice == 0)
    {
        // �������� 1: #f6bf75, #d77185, #8766ac, #4150b1
        gradient[0].position = sf::Vector2f(0, 0);
        gradient[0].color = sf::Color(0xf6, 0xbf, 0x75); // ������-���������

        gradient[1].position = sf::Vector2f(windowSize.x, 0);
        gradient[1].color = sf::Color(0xd7, 0x71, 0x85); // ������-�������

        gradient[2].position = sf::Vector2f(0, windowSize.y);
        gradient[2].color = sf::Color(0x87, 0x66, 0xac); // ����������

        gradient[3].position = sf::Vector2f(windowSize.x, windowSize.y);
        gradient[3].color = sf::Color(0x41, 0x50, 0xb1); // �����
    }
    else if (gradientChoice == 1)
    {
        // �������� 2: #4facfe, #00f2fe
        gradient[0].position = sf::Vector2f(0, 0);
        gradient[0].color = sf::Color(0x4f, 0xac, 0xfe); // �������

        gradient[1].position = sf::Vector2f(windowSize.x, 0);
        gradient[1].color = sf::Color(0x00, 0xf2, 0xfe); // ���������

        gradient[2].position = sf::Vector2f(0, windowSize.y);
        gradient[2].color = sf::Color(0x4f, 0xac, 0xfe); // ������ ��������

        gradient[3].position = sf::Vector2f(windowSize.x, windowSize.y);
        gradient[3].color = sf::Color(0x00, 0xf2, 0xfe); // ������ ����������
    }
    else if (gradientChoice == 2)
    {
        // �������� 2: #b3b6eb, #e98a98
        gradient[0].position = sf::Vector2f(0, 0);
        gradient[0].color = sf::Color(0xb3, 0xb6, 0xeb); // ������-����������

        gradient[1].position = sf::Vector2f(windowSize.x, 0);
        gradient[1].color = sf::Color(0xe9, 0x8a, 0x98); // �������

        gradient[2].position = sf::Vector2f(0, windowSize.y);
        gradient[2].color = sf::Color(0xb3, 0xb6, 0xeb); // ������ ������-�����������

        gradient[3].position = sf::Vector2f(windowSize.x, windowSize.y);
        gradient[3].color = sf::Color(0xe9, 0x8a, 0x98); // ������ ��������
    }

    else if (gradientChoice == 3)
    {
        // �������� 2: #f6bf9f, #db8ade
        gradient[0].position = sf::Vector2f(0, 0);
        gradient[0].color = sf::Color(0xf6, 0xbf, 0x9f); // ������-���������

        gradient[1].position = sf::Vector2f(windowSize.x, 0);
        gradient[1].color = sf::Color(0xdb, 0x8a, 0xde); // ������-����������

        gradient[2].position = sf::Vector2f(0, windowSize.y);
        gradient[2].color = sf::Color(0xf6, 0xbf, 0x9f); // ������ ������-����������

        gradient[3].position = sf::Vector2f(windowSize.x, windowSize.y);
        gradient[3].color = sf::Color(0xdb, 0x8a, 0xde); // ������ ������-�����������
    }
    else if (gradientChoice == 4)
    {
        // �������� 2: #0c2442, #4f7492
        gradient[0].position = sf::Vector2f(0, 0);
        gradient[0].color = sf::Color(0x0c, 0x24, 0x42); // ����-�����

        gradient[1].position = sf::Vector2f(windowSize.x, 0);
        gradient[1].color = sf::Color(0x4f, 0x74, 0x92); // ����������-�����

        gradient[2].position = sf::Vector2f(0, windowSize.y);
        gradient[2].color = sf::Color(0x0c, 0x24, 0x42); // ������ ����-������

        gradient[3].position = sf::Vector2f(windowSize.x, windowSize.y);
        gradient[3].color = sf::Color(0x4f, 0x74, 0x92); // ������ ����������-������
    }

    else
    {
        // �������� 3: #d70a84, #51127f
        gradient[0].position = sf::Vector2f(0, 0);
        gradient[0].color = sf::Color(0xd7, 0x0a, 0x84); // �������

        gradient[1].position = sf::Vector2f(windowSize.x, 0);
        gradient[1].color = sf::Color(0x51, 0x12, 0x7f); // �����-����������

        gradient[2].position = sf::Vector2f(0, windowSize.y);
        gradient[2].color = sf::Color(0xd7, 0x0a, 0x84); // ������ ��������

        gradient[3].position = sf::Vector2f(windowSize.x, windowSize.y);
        gradient[3].color = sf::Color(0x51, 0x12, 0x7f); // ������ �����-�����������
    }
}
void LoadWindow::handleEvents()
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
void LoadWindow::draw()
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
void LoadWindow::run()
{
    while (window.isOpen())
    {
        handleEvents();
        draw();
    }
}


void LoadWindow::LoadSound() {
    std::string filepath = "D:/Julia/Utilities/Sound/1.mp3";
    // ���������� ����� ��������� ��� ���������� �������� sf::SoundBuffer
    auto buffer = std::make_shared<sf::SoundBuffer>();
    if (!buffer->loadFromFile(filepath)) {
        std::cerr << "������: �� ������� ��������� ���������!" << std::endl;
        return;
    }

    // ������� ���� � ����������� ��� �����
    sf::Sound sound;
    sound.setBuffer(*buffer);

    // ������� ��� ��������������� ����� � ��������� ������
    std::thread soundThread([sound]() mutable {
        sound.play();

        // ������� ���������� ��������������� �����
        while (sound.getStatus() == sf::Sound::Playing) {
            sf::sleep(sf::milliseconds(100)); // ������� ��������, ����� �� ��������� ���������
        }
        });

    soundThread.detach(); // ����������� �����, ����� �� ���������� ��� ����� ���������������
}