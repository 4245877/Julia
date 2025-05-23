#include "LoadWindow.h"
#include <SFML/Graphics.hpp>
#include <SFML/Audio.hpp>
#include <iostream>     // ��� std::cerr
#include <string>
#include <vector>
#include <memory>       // ��� std::make_shared, std::shared_ptr
#include <thread>       // ��� std::thread
#include <future>       // ��� std::promise, std::future
#include <random>       // ��� std::random_device, std::uniform_int_distribution
#include <array>        // ��� std::array (���� �����)
#include <chrono>       // ��� sf::sleep � ��������

// ����������� ����������� �������� �� LoadWindow.h
const std::string LoadWindow::INITIAL_TEXT_STRING = "Julia"; // ������
const std::string LoadWindow::FONT_DIRECTORY = "../Utilities/resources/Fonts/"; // �������� ���� �������������!
const std::string LoadWindow::SOUND_FILE_PATH = "D:/Julia/Utilities/resources/Sound/1.mp3"; // �������� ���� �������������!

// ������� �������
const std::vector<std::string> LoadWindow::FONT_FILES = {
    "oldtimer-GOPpg.ttf", "bebasNeue-Regular.ttf", "reading Regular.ttf",
    "mattoa Demo.ttf", "quan-BlackItalic.ttf", "sacramento.ttf",
    "Neonderthaw-Regular.ttf", "FleurDeLeah-Regular.ttf",
    "Futuristic-Regular.ttf", "Ninja District.ttf"
};

// ������� ����������
const std::vector<LoadWindow::GradientColors> LoadWindow::GRADIENT_PRESETS = {
    {sf::Color(0xf6, 0xbf, 0x75), sf::Color(0xd7, 0x71, 0x85), sf::Color(0x87, 0x66, 0xac), sf::Color(0x41, 0x50, 0xb1)}, // �������� 1
    {sf::Color(0x4f, 0xac, 0xfe), sf::Color(0x00, 0xf2, 0xfe), sf::Color(0x4f, 0xac, 0xfe), sf::Color(0x00, 0xf2, 0xfe)}, // �������� 2
    {sf::Color(0xb3, 0xb6, 0xeb), sf::Color(0xe9, 0x8a, 0x98), sf::Color(0xb3, 0xb6, 0xeb), sf::Color(0xe9, 0x8a, 0x98)}, // �������� 3
    {sf::Color(0xf6, 0xbf, 0x9f), sf::Color(0xdb, 0x8a, 0xde), sf::Color(0xf6, 0xbf, 0x9f), sf::Color(0xdb, 0x8a, 0xde)}, // �������� 4
    {sf::Color(0x0c, 0x24, 0x42), sf::Color(0x4f, 0x74, 0x92), sf::Color(0x0c, 0x24, 0x42), sf::Color(0x4f, 0x74, 0x92)}, // �������� 5
    {sf::Color(0xd7, 0x0a, 0x84), sf::Color(0x51, 0x12, 0x7f), sf::Color(0xd7, 0x0a, 0x84), sf::Color(0x51, 0x12, 0x7f)}  // �������� 6 (�� ����� else)
};


LoadWindow::LoadWindow() :
    gradient(sf::TrianglesStrip, 4), // 4 ������� ��� ���������
    randomGenerator(std::random_device{}()) // ������������� ����
{
    // ������������� ���� ���� ��� (������������ - � ������ �������������)
    // randomGenerator.seed(static_cast<unsigned long>(std::time(nullptr))); // ���� random_device �� ���� ����������� ��������

    initializeWindow();
    initializeFont(); // �������� ������
    initializeInitialText(); // ������������� �������� ������ ("Julia")
    initializeLoadingStageText(); // ������������� ������ ������ �������� 
    initializeGradient();
}

LoadWindow::~LoadWindow() {
    // ���������� ������ ����.
    // ������� SFML (window, font, text, gradient) ������������� �������������.
    // ������� (loadingStages) ����� ��������� �������������.
}

void LoadWindow::initializeWindow() {
    sf::VideoMode desktop = sf::VideoMode::getDesktopMode();

    // ������ �������� ���� �� ������ ������ ������ � ����������� ������
    int windowHeight = static_cast<int>(desktop.height * WINDOW_HEIGHT_SCALE);
    int windowWidth = static_cast<int>(windowHeight * WINDOW_ASPECT_RATIO);

    // ��������, ����� ���� �� ���� ���� ������
    if (windowWidth > static_cast<int>(desktop.width)) {
        windowWidth = static_cast<int>(desktop.width * 0.85f); // ��������, 85% ������ ������
        windowHeight = static_cast<int>(windowWidth / WINDOW_ASPECT_RATIO);
    }

    window.create(sf::VideoMode(windowWidth, windowHeight), "Loading...", sf::Style::None);
    window.setPosition(sf::Vector2i((desktop.width - windowWidth) / 2, (desktop.height - windowHeight) / 2));
}

void LoadWindow::initializeFont() {
    if (FONT_FILES.empty()) {
        throw std::runtime_error("������ ������ ������� ����.");
    }
    std::uniform_int_distribution<size_t> dist(0, FONT_FILES.size() - 1);
    std::string fontFile = FONT_FILES[dist(randomGenerator)];

    if (!font.loadFromFile(FONT_DIRECTORY + fontFile)) {
        throw std::runtime_error("�� ������� ��������� �����: " + FONT_DIRECTORY + fontFile);
    }
}

void LoadWindow::initializeInitialText() {
    text.setFont(font);
    text.setString(INITIAL_TEXT_STRING);

    // ��������������� ������� ������ ������������ ��������� ������
    sf::VideoMode desktop = sf::VideoMode::getDesktopMode();
    float scaleFactor = static_cast<float>(desktop.width) / INITIAL_TEXT_CHAR_SIZE_REFERENCE_WIDTH;
    unsigned int charSize = static_cast<unsigned int>(INITIAL_TEXT_CHAR_SIZE_AT_REFERENCE * scaleFactor);
    // ����������� �� ����������� ������, ����� ����� �� ��� ������� ������ �� ��������� �������
    if (charSize < 20) charSize = 20;
    text.setCharacterSize(charSize);

    sf::FloatRect textRect = text.getLocalBounds();
    text.setOrigin(textRect.left + textRect.width / 2.0f, textRect.top + textRect.height / 2.0f);
    text.setPosition(window.getSize().x / 2.0f, window.getSize().y / 2.0f);
}

void LoadWindow::initializeLoadingStageText() {
    loadingStageText.setFont(font);
    // ��������� ����� ����� ���������� � update()
    unsigned int charSize = static_cast<unsigned int>(window.getSize().y * LOADING_TEXT_CHAR_SIZE_SCALE);
    if (charSize < 12) charSize = 12; // ����������� ������
    loadingStageText.setCharacterSize(charSize);
    loadingStageText.setFillColor(sf::Color::White); // ��� ������ ���������� ����
}


void LoadWindow::initializeGradient() {
    if (GRADIENT_PRESETS.empty()) {
        throw std::runtime_error("������ �������� ��������� ����.");
    }
    std::uniform_int_distribution<size_t> dist(0, GRADIENT_PRESETS.size() - 1);
    const auto& colors = GRADIENT_PRESETS[dist(randomGenerator)];

    sf::Vector2f windowSize(static_cast<float>(window.getSize().x), static_cast<float>(window.getSize().y));

    gradient[0].position = sf::Vector2f(0, 0);
    gradient[0].color = colors.topLeft;

    gradient[1].position = sf::Vector2f(windowSize.x, 0);
    gradient[1].color = colors.topRight;

    gradient[2].position = sf::Vector2f(0, windowSize.y);
    gradient[2].color = colors.bottomLeft;

    gradient[3].position = sf::Vector2f(windowSize.x, windowSize.y);
    gradient[3].color = colors.bottomRight;
}

void LoadWindow::handleEvents() {
    sf::Event event;
    while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed ||
            (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Escape)) {
            window.close();
        }
    }
}

void LoadWindow::update() {
    if (allStagesComplete) {
        return; // ������ ������ �� ���������, ���� �������� ����� �����
    }

    if (currentStage < loadingStages.size()) {
        loadingStageText.setString(loadingStages[currentStage]);

        sf::FloatRect textBounds = loadingStageText.getLocalBounds();
        // ������������� ����� � ����� ������ ���� ���� � ��������
        loadingStageText.setPosition(
            TEXT_PADDING,
            window.getSize().y - textBounds.height - TEXT_PADDING - textBounds.top
        );
        // textBounds.top ����� ���� ������������� ��� ��������� �������, ��� ����� ���������

        // �������� �������� ��� �������� ������
        // ��� ������������ ����� �������� �������� ����� ��������� � ���������� �����
        // sf::sleep(sf::milliseconds(500)); // ����� ������, �������� � run()

        currentStage++;
    }
    else {
        allStagesComplete = true; // ��� ����� ��������
        // ���� ����� ������ � run() ����� �����
    }
}


void LoadWindow::draw() {
    window.clear(sf::Color::White); // ��� ����� ������ ������� ���� �� ���������
    window.draw(gradient);
    window.draw(text); // ���������� ������� ����� "Julia"
    if (currentStage > 0 && currentStage <= loadingStages.size() && !loadingStages.empty()) {
        // ���������� ����� ����� ��������, ���� �� ����
        window.draw(loadingStageText);
    }
    window.display();
}

void LoadWindow::run() {
    sf::Clock clock; // ��� ���������� �������� ���������� ������
    sf::Time timePerStage = sf::seconds(0.75f); // ����� �� ������ ����

    while (window.isOpen()) {
        handleEvents();

        if (!allStagesComplete) {
            if (clock.getElapsedTime() >= timePerStage || currentStage == 0) { // �������� ����� ������ ����
                update(); // ��������� ������ (����� �����)
                clock.restart();
            }
        }
        else {
            // ����� ���������, ����������� ���� � ���������
            playCompletionSound();
            window.close(); // ��������� ���� ����� �����
        }

        draw(); // ������ ������
    }
}

void LoadWindow::playCompletionSound() {
    // ���������� ������������ ������ � �������, �� ������ ��� ���������� � ������ ������
    auto buffer = std::make_shared<sf::SoundBuffer>();
    if (!buffer->loadFromFile(SOUND_FILE_PATH)) {
        std::cerr << "������: �� ������� ��������� ���������: " << SOUND_FILE_PATH << std::endl;
        return;
    }

    auto sound = std::make_shared<sf::Sound>();
    sound->setBuffer(*buffer);

    std::promise<void> soundPromise;
    std::future<void> soundFuture = soundPromise.get_future();

    // ��������� ��������������� � ��������� ������, ����� �� ����������� �������� ���� �������,
    // �� ���������� future ��� �������� ���������� ����� ��������� ����.
    std::thread soundThread([sound, buffer, &soundPromise]() { // ����������� buffer, ����� �� ��� ���� �����
        sound->play();
        while (sound->getStatus() == sf::Sound::Playing) {
            sf::sleep(sf::milliseconds(100)); // ��������� ������ �� ������� �����
        }
        soundPromise.set_value(); // ������ � ���������� ���������������
        });

    soundFuture.wait(); // ������� ���������� ��������������� �����
    if (soundThread.joinable()) { // �������� ��� ����� ����� ������������
        soundThread.join(); // ��������� ��������� �����
    }
}