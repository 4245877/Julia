#include "LoadWindow.h"
#include <SFML/Graphics.hpp>
#include <SFML/Audio.hpp>
#include <iostream>     // Для std::cerr
#include <string>
#include <vector>
#include <memory>       // Для std::make_shared, std::shared_ptr
#include <thread>       // Для std::thread
#include <future>       // Для std::promise, std::future
#include <random>       // Для std::random_device, std::uniform_int_distribution
#include <array>        // Для std::array (если нужно)
#include <chrono>       // Для sf::sleep и задержек

// Определения статических констант из LoadWindow.h
const std::string LoadWindow::INITIAL_TEXT_STRING = "Julia"; // Пример
const std::string LoadWindow::FONT_DIRECTORY = "../Utilities/resources/Fonts/"; // Сделайте путь настраиваемым!
const std::string LoadWindow::SOUND_FILE_PATH = "D:/Julia/Utilities/resources/Sound/1.mp3"; // Сделайте путь настраиваемым!

// Пресеты шрифтов
const std::vector<std::string> LoadWindow::FONT_FILES = {
    "oldtimer-GOPpg.ttf", "bebasNeue-Regular.ttf", "reading Regular.ttf",
    "mattoa Demo.ttf", "quan-BlackItalic.ttf", "sacramento.ttf",
    "Neonderthaw-Regular.ttf", "FleurDeLeah-Regular.ttf",
    "Futuristic-Regular.ttf", "Ninja District.ttf"
};

// Пресеты градиентов
const std::vector<LoadWindow::GradientColors> LoadWindow::GRADIENT_PRESETS = {
    {sf::Color(0xf6, 0xbf, 0x75), sf::Color(0xd7, 0x71, 0x85), sf::Color(0x87, 0x66, 0xac), sf::Color(0x41, 0x50, 0xb1)}, // Градиент 1
    {sf::Color(0x4f, 0xac, 0xfe), sf::Color(0x00, 0xf2, 0xfe), sf::Color(0x4f, 0xac, 0xfe), sf::Color(0x00, 0xf2, 0xfe)}, // Градиент 2
    {sf::Color(0xb3, 0xb6, 0xeb), sf::Color(0xe9, 0x8a, 0x98), sf::Color(0xb3, 0xb6, 0xeb), sf::Color(0xe9, 0x8a, 0x98)}, // Градиент 3
    {sf::Color(0xf6, 0xbf, 0x9f), sf::Color(0xdb, 0x8a, 0xde), sf::Color(0xf6, 0xbf, 0x9f), sf::Color(0xdb, 0x8a, 0xde)}, // Градиент 4
    {sf::Color(0x0c, 0x24, 0x42), sf::Color(0x4f, 0x74, 0x92), sf::Color(0x0c, 0x24, 0x42), sf::Color(0x4f, 0x74, 0x92)}, // Градиент 5
    {sf::Color(0xd7, 0x0a, 0x84), sf::Color(0x51, 0x12, 0x7f), sf::Color(0xd7, 0x0a, 0x84), sf::Color(0x51, 0x12, 0x7f)}  // Градиент 6 (из блока else)
};


LoadWindow::LoadWindow() :
    gradient(sf::TrianglesStrip, 4), // 4 вершины для градиента
    randomGenerator(std::random_device{}()) // Инициализация ГПСЧ
{
    // Инициализация ГПСЧ один раз (альтернатива - в списке инициализации)
    // randomGenerator.seed(static_cast<unsigned long>(std::time(nullptr))); // Если random_device не дает достаточной энтропии

    initializeWindow();
    initializeFont(); // Загрузка шрифта
    initializeInitialText(); // Инициализация главного текста ("Julia")
    initializeLoadingStageText(); // Инициализация текста этапов загрузки 
    initializeGradient();
}

LoadWindow::~LoadWindow() {
    // Деструктор теперь чист.
    // Ресурсы SFML (window, font, text, gradient) освобождаются автоматически.
    // Векторы (loadingStages) также очищаются автоматически.
}

void LoadWindow::initializeWindow() {
    sf::VideoMode desktop = sf::VideoMode::getDesktopMode();

    // Расчет размеров окна на основе высоты экрана и соотношения сторон
    int windowHeight = static_cast<int>(desktop.height * WINDOW_HEIGHT_SCALE);
    int windowWidth = static_cast<int>(windowHeight * WINDOW_ASPECT_RATIO);

    // Проверка, чтобы окно не было шире экрана
    if (windowWidth > static_cast<int>(desktop.width)) {
        windowWidth = static_cast<int>(desktop.width * 0.85f); // Например, 85% ширины экрана
        windowHeight = static_cast<int>(windowWidth / WINDOW_ASPECT_RATIO);
    }

    window.create(sf::VideoMode(windowWidth, windowHeight), "Loading...", sf::Style::None);
    window.setPosition(sf::Vector2i((desktop.width - windowWidth) / 2, (desktop.height - windowHeight) / 2));
}

void LoadWindow::initializeFont() {
    if (FONT_FILES.empty()) {
        throw std::runtime_error("Список файлов шрифтов пуст.");
    }
    std::uniform_int_distribution<size_t> dist(0, FONT_FILES.size() - 1);
    std::string fontFile = FONT_FILES[dist(randomGenerator)];

    if (!font.loadFromFile(FONT_DIRECTORY + fontFile)) {
        throw std::runtime_error("Не удалось загрузить шрифт: " + FONT_DIRECTORY + fontFile);
    }
}

void LoadWindow::initializeInitialText() {
    text.setFont(font);
    text.setString(INITIAL_TEXT_STRING);

    // Масштабирование размера шрифта относительно эталонной ширины
    sf::VideoMode desktop = sf::VideoMode::getDesktopMode();
    float scaleFactor = static_cast<float>(desktop.width) / INITIAL_TEXT_CHAR_SIZE_REFERENCE_WIDTH;
    unsigned int charSize = static_cast<unsigned int>(INITIAL_TEXT_CHAR_SIZE_AT_REFERENCE * scaleFactor);
    // Ограничение на минимальный размер, чтобы текст не был слишком мелким на маленьких экранах
    if (charSize < 20) charSize = 20;
    text.setCharacterSize(charSize);

    sf::FloatRect textRect = text.getLocalBounds();
    text.setOrigin(textRect.left + textRect.width / 2.0f, textRect.top + textRect.height / 2.0f);
    text.setPosition(window.getSize().x / 2.0f, window.getSize().y / 2.0f);
}

void LoadWindow::initializeLoadingStageText() {
    loadingStageText.setFont(font);
    // Начальный текст будет установлен в update()
    unsigned int charSize = static_cast<unsigned int>(window.getSize().y * LOADING_TEXT_CHAR_SIZE_SCALE);
    if (charSize < 12) charSize = 12; // Минимальный размер
    loadingStageText.setCharacterSize(charSize);
    loadingStageText.setFillColor(sf::Color::White); // Или другой подходящий цвет
}


void LoadWindow::initializeGradient() {
    if (GRADIENT_PRESETS.empty()) {
        throw std::runtime_error("Список пресетов градиента пуст.");
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
        return; // Больше ничего не обновляем, ждем закрытия после звука
    }

    if (currentStage < loadingStages.size()) {
        loadingStageText.setString(loadingStages[currentStage]);

        sf::FloatRect textBounds = loadingStageText.getLocalBounds();
        // Устанавливаем текст в левый нижний угол окна с отступом
        loadingStageText.setPosition(
            TEXT_PADDING,
            window.getSize().y - textBounds.height - TEXT_PADDING - textBounds.top
        );
        // textBounds.top может быть отрицательным для некоторых шрифтов, это важно учитывать

        // Имитация задержки или реальной работы
        // Для демонстрации можно добавить задержку перед переходом к следующему этапу
        // sf::sleep(sf::milliseconds(500)); // Убрал отсюда, задержка в run()

        currentStage++;
    }
    else {
        allStagesComplete = true; // Все этапы пройдены
        // Звук будет вызван в run() после этого
    }
}


void LoadWindow::draw() {
    window.clear(sf::Color::White); // Или любой другой фоновый цвет по умолчанию
    window.draw(gradient);
    window.draw(text); // Отображаем главный текст "Julia"
    if (currentStage > 0 && currentStage <= loadingStages.size() && !loadingStages.empty()) {
        // Отображаем текст этапа загрузки, если он есть
        window.draw(loadingStageText);
    }
    window.display();
}

void LoadWindow::run() {
    sf::Clock clock; // Для управления частотой обновления этапов
    sf::Time timePerStage = sf::seconds(0.75f); // Время на каждый этап

    while (window.isOpen()) {
        handleEvents();

        if (!allStagesComplete) {
            if (clock.getElapsedTime() >= timePerStage || currentStage == 0) { // Обновить сразу первый этап
                update(); // Обновляем логику (смена этапа)
                clock.restart();
            }
        }
        else {
            // Этапы завершены, проигрываем звук и закрываем
            playCompletionSound();
            window.close(); // Закрываем окно после звука
        }

        draw(); // Рисуем всегда
    }
}

void LoadWindow::playCompletionSound() {
    // Используем существующую логику с потоком, но теперь она вызывается в нужный момент
    auto buffer = std::make_shared<sf::SoundBuffer>();
    if (!buffer->loadFromFile(SOUND_FILE_PATH)) {
        std::cerr << "Ошибка: Не удалось загрузить аудиофайл: " << SOUND_FILE_PATH << std::endl;
        return;
    }

    auto sound = std::make_shared<sf::Sound>();
    sound->setBuffer(*buffer);

    std::promise<void> soundPromise;
    std::future<void> soundFuture = soundPromise.get_future();

    // Запускаем воспроизведение в отдельном потоке, чтобы не блокировать основной цикл надолго,
    // но используем future для ожидания завершения перед закрытием окна.
    std::thread soundThread([sound, buffer, &soundPromise]() { // Захватываем buffer, чтобы он жил пока нужен
        sound->play();
        while (sound->getStatus() == sf::Sound::Playing) {
            sf::sleep(sf::milliseconds(100)); // Проверяем статус не слишком часто
        }
        soundPromise.set_value(); // Сигнал о завершении воспроизведения
        });

    soundFuture.wait(); // Ожидаем завершения воспроизведения звука
    if (soundThread.joinable()) { // Убедимся что поток можно присоединить
        soundThread.join(); // Корректно завершаем поток
    }
}