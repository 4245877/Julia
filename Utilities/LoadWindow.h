#ifndef LOADWINDOW_H // Исправлен защитный макрос
#define LOADWINDOW_H

#include <SFML/Graphics.hpp>
#include <SFML/Audio.hpp>
#include <vector>
#include <string>
#include <random> // Для std::mt19937

class LoadWindow {
private:
    // Константы конфигурации
    static constexpr float WINDOW_ASPECT_RATIO = 16.0f / 9.0f; // Примерное соотношение сторон для окна
    static constexpr float WINDOW_HEIGHT_SCALE = 0.30f; // 30% высоты экрана
    static constexpr unsigned int INITIAL_TEXT_CHAR_SIZE_REFERENCE_WIDTH = 1920; // Эталонная ширина экрана для размера шрифта
    static constexpr unsigned int INITIAL_TEXT_CHAR_SIZE_AT_REFERENCE = 150;    // Размер шрифта при эталонной ширине
    static constexpr float LOADING_TEXT_CHAR_SIZE_SCALE = 0.03f; // Масштаб для шрифта текста загрузки от высоты окна
    static constexpr float TEXT_PADDING = 15.0f;
    static const std::string INITIAL_TEXT_STRING; // Определим в .cpp
    static const std::string FONT_DIRECTORY;      // Определим в .cpp
    static const std::string SOUND_FILE_PATH;     // Определим в .cpp

    struct GradientColors {
        sf::Color topLeft, topRight, bottomLeft, bottomRight;
    };
     
    sf::RenderWindow window;
    sf::Font font;
    sf::Text text;
    sf::Text loadingStageText; // Отдельный объект Text для сообщений о загрузке
    sf::VertexArray gradient;

    std::vector<std::string> loadingStages = {
        "Инициализация приложения...",
        "Загрузка основных ресурсов...",
        "Подключение к сервисам...",
        "Настройка пользовательского интерфейса...",
        "Последние штрихи..."
    };
    int currentStage = 0;
    bool allStagesComplete = false;

    std::mt19937 randomGenerator; // Генератор случайных чисел из <random>

    // Статические данные для шрифтов и градиентов (определяются в .cpp)
    static const std::vector<std::string> FONT_FILES;
    static const std::vector<GradientColors> GRADIENT_PRESETS;

    // Приватные методы инициализации
    void initializeWindow();
    void initializeFont(); // Объединенная инициализация шрифта
    void initializeInitialText();
    void initializeLoadingStageText();
    void initializeGradient();

    // Приватные методы для логики
    void handleEvents();
    void update();
    void draw();
    void playCompletionSound(); // Переименованная функция для звука

public:
    LoadWindow();
    ~LoadWindow(); // Деструктор теперь пустой или для специфичной очистки

    void run();
    // void updateLoadingStage(); // Логика перенесена в update() и связана с playCompletionSound
};

#endif // LOADWINDOW_H