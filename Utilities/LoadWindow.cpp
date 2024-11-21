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
// Деструктор нужно доделать!!!
LoadWindow:: ~LoadWindow()
{
    LoadSound();
}
void LoadWindow::initializeWindow()
{
    // Отримуємо розміри екрану
    sf::VideoMode desktop = sf::VideoMode::getDesktopMode();
    int screenWidth = desktop.width;
    int screenHeight = desktop.height;

    // Обчислюємо розміри вікна (35% від площі екрану)
    int windowWidth = static_cast<int>(screenWidth * 0.45f);
    int windowHeight = static_cast<int>(screenHeight * 0.45f);

    // Створюємо вікно без рамки і заголовку
    window.create(sf::VideoMode(windowWidth, windowHeight), "", sf::Style::None);

    // Центруємо вікно по середині екрану
    window.setPosition(sf::Vector2i((screenWidth - windowWidth) / 2, (screenHeight - windowHeight) / 2));
}
void LoadWindow::initializeText()
{
    // Отримуємо розміри екрану
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

    //текст
    text.setFont(font);
    text.setString("Julia");

    text.setCharacterSize(FontSize);

    // Позиція тексту в центрі вікна
    sf::FloatRect textRect = text.getLocalBounds();
    text.setOrigin(textRect.left + textRect.width / 2.0f, textRect.top + textRect.height / 2.0f);
    text.setPosition(window.getSize().x / 2.0f, window.getSize().y / 2.0f);
}
void LoadWindow::initializeGradient()
{
    // Инициализация генератора случайных чисел
    srand(static_cast<unsigned>(time(0)));

    int gradientChoice = rand() % 5;

    // Получаем размер окна
    sf::Vector2f windowSize(window.getSize().x, window.getSize().y);

    // В зависимости от выбранного варианта задаём градиент
    if (gradientChoice == 0)
    {
        // Градиент 1: #f6bf75, #d77185, #8766ac, #4150b1
        gradient[0].position = sf::Vector2f(0, 0);
        gradient[0].color = sf::Color(0xf6, 0xbf, 0x75); // светло-оранжевый

        gradient[1].position = sf::Vector2f(windowSize.x, 0);
        gradient[1].color = sf::Color(0xd7, 0x71, 0x85); // розово-красный

        gradient[2].position = sf::Vector2f(0, windowSize.y);
        gradient[2].color = sf::Color(0x87, 0x66, 0xac); // фиолетовый

        gradient[3].position = sf::Vector2f(windowSize.x, windowSize.y);
        gradient[3].color = sf::Color(0x41, 0x50, 0xb1); // синий
    }
    else if (gradientChoice == 1)
    {
        // Градиент 2: #4facfe, #00f2fe
        gradient[0].position = sf::Vector2f(0, 0);
        gradient[0].color = sf::Color(0x4f, 0xac, 0xfe); // голубой

        gradient[1].position = sf::Vector2f(windowSize.x, 0);
        gradient[1].color = sf::Color(0x00, 0xf2, 0xfe); // бирюзовый

        gradient[2].position = sf::Vector2f(0, windowSize.y);
        gradient[2].color = sf::Color(0x4f, 0xac, 0xfe); // повтор голубого

        gradient[3].position = sf::Vector2f(windowSize.x, windowSize.y);
        gradient[3].color = sf::Color(0x00, 0xf2, 0xfe); // повтор бирюзового
    }
    else if (gradientChoice == 2)
    {
        // Градиент 2: #b3b6eb, #e98a98
        gradient[0].position = sf::Vector2f(0, 0);
        gradient[0].color = sf::Color(0xb3, 0xb6, 0xeb); // светло-фиолетовый

        gradient[1].position = sf::Vector2f(windowSize.x, 0);
        gradient[1].color = sf::Color(0xe9, 0x8a, 0x98); // розовый

        gradient[2].position = sf::Vector2f(0, windowSize.y);
        gradient[2].color = sf::Color(0xb3, 0xb6, 0xeb); // повтор светло-фиолетового

        gradient[3].position = sf::Vector2f(windowSize.x, windowSize.y);
        gradient[3].color = sf::Color(0xe9, 0x8a, 0x98); // повтор розового
    }

    else if (gradientChoice == 3)
    {
        // Градиент 2: #f6bf9f, #db8ade
        gradient[0].position = sf::Vector2f(0, 0);
        gradient[0].color = sf::Color(0xf6, 0xbf, 0x9f); // светло-оранжевый

        gradient[1].position = sf::Vector2f(windowSize.x, 0);
        gradient[1].color = sf::Color(0xdb, 0x8a, 0xde); // светло-фиолетовый

        gradient[2].position = sf::Vector2f(0, windowSize.y);
        gradient[2].color = sf::Color(0xf6, 0xbf, 0x9f); // повтор светло-оранжевого

        gradient[3].position = sf::Vector2f(windowSize.x, windowSize.y);
        gradient[3].color = sf::Color(0xdb, 0x8a, 0xde); // повтор светло-фиолетового
    }
    else if (gradientChoice == 4)
    {
        // Градиент 2: #0c2442, #4f7492
        gradient[0].position = sf::Vector2f(0, 0);
        gradient[0].color = sf::Color(0x0c, 0x24, 0x42); // тёмно-синий

        gradient[1].position = sf::Vector2f(windowSize.x, 0);
        gradient[1].color = sf::Color(0x4f, 0x74, 0x92); // голубовато-серый

        gradient[2].position = sf::Vector2f(0, windowSize.y);
        gradient[2].color = sf::Color(0x0c, 0x24, 0x42); // повтор тёмно-синего

        gradient[3].position = sf::Vector2f(windowSize.x, windowSize.y);
        gradient[3].color = sf::Color(0x4f, 0x74, 0x92); // повтор голубовато-серого
    }

    else
    {
        // Градиент 3: #d70a84, #51127f
        gradient[0].position = sf::Vector2f(0, 0);
        gradient[0].color = sf::Color(0xd7, 0x0a, 0x84); // розовый

        gradient[1].position = sf::Vector2f(windowSize.x, 0);
        gradient[1].color = sf::Color(0x51, 0x12, 0x7f); // темно-фиолетовый

        gradient[2].position = sf::Vector2f(0, windowSize.y);
        gradient[2].color = sf::Color(0xd7, 0x0a, 0x84); // повтор розового

        gradient[3].position = sf::Vector2f(windowSize.x, windowSize.y);
        gradient[3].color = sf::Color(0x51, 0x12, 0x7f); // повтор темно-фиолетового
    }
}
void LoadWindow::handleEvents()
{
    sf::Event event;
    while (window.pollEvent(event))
    {
        // Закриття вікна при натисканні на клавішу ESC або через подію закриття
        if (event.type == sf::Event::Closed ||
            (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Escape))
            window.close();
    }
}
void LoadWindow::draw()
{
    // Очищуємо вікно білим кольором
    window.clear(sf::Color::White);

    // Малюємо градієнт
    window.draw(gradient);

    // Малюємо текст
    window.draw(text);

    // Відображаємо вміст вікна
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
    // Используем умный указатель для управления объектом sf::SoundBuffer
    auto buffer = std::make_shared<sf::SoundBuffer>();
    if (!buffer->loadFromFile(filepath)) {
        std::cerr << "Ошибка: Не удалось загрузить аудиофайл!" << std::endl;
        return;
    }

    // Создаем звук и настраиваем его буфер
    sf::Sound sound;
    sound.setBuffer(*buffer);

    // Функция для воспроизведения звука в отдельном потоке
    std::thread soundThread([sound]() mutable {
        sound.play();

        // Ожидаем завершения воспроизведения звука
        while (sound.getStatus() == sf::Sound::Playing) {
            sf::sleep(sf::milliseconds(100)); // Немного подождем, чтобы не нагружать процессор
        }
        });

    soundThread.detach(); // Отсоединяем поток, чтобы он завершился сам после воспроизведения
}