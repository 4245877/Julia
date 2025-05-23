#ifndef LOADWINDOW_H // ��������� �������� ������
#define LOADWINDOW_H

#include <SFML/Graphics.hpp>
#include <SFML/Audio.hpp>
#include <vector>
#include <string>
#include <random> // ��� std::mt19937

class LoadWindow {
private:
    // ��������� ������������
    static constexpr float WINDOW_ASPECT_RATIO = 16.0f / 9.0f; // ��������� ����������� ������ ��� ����
    static constexpr float WINDOW_HEIGHT_SCALE = 0.30f; // 30% ������ ������
    static constexpr unsigned int INITIAL_TEXT_CHAR_SIZE_REFERENCE_WIDTH = 1920; // ��������� ������ ������ ��� ������� ������
    static constexpr unsigned int INITIAL_TEXT_CHAR_SIZE_AT_REFERENCE = 150;    // ������ ������ ��� ��������� ������
    static constexpr float LOADING_TEXT_CHAR_SIZE_SCALE = 0.03f; // ������� ��� ������ ������ �������� �� ������ ����
    static constexpr float TEXT_PADDING = 15.0f;
    static const std::string INITIAL_TEXT_STRING; // ��������� � .cpp
    static const std::string FONT_DIRECTORY;      // ��������� � .cpp
    static const std::string SOUND_FILE_PATH;     // ��������� � .cpp

    struct GradientColors {
        sf::Color topLeft, topRight, bottomLeft, bottomRight;
    };
     
    sf::RenderWindow window;
    sf::Font font;
    sf::Text text;
    sf::Text loadingStageText; // ��������� ������ Text ��� ��������� � ��������
    sf::VertexArray gradient;

    std::vector<std::string> loadingStages = {
        "������������� ����������...",
        "�������� �������� ��������...",
        "����������� � ��������...",
        "��������� ����������������� ����������...",
        "��������� ������..."
    };
    int currentStage = 0;
    bool allStagesComplete = false;

    std::mt19937 randomGenerator; // ��������� ��������� ����� �� <random>

    // ����������� ������ ��� ������� � ���������� (������������ � .cpp)
    static const std::vector<std::string> FONT_FILES;
    static const std::vector<GradientColors> GRADIENT_PRESETS;

    // ��������� ������ �������������
    void initializeWindow();
    void initializeFont(); // ������������ ������������� ������
    void initializeInitialText();
    void initializeLoadingStageText();
    void initializeGradient();

    // ��������� ������ ��� ������
    void handleEvents();
    void update();
    void draw();
    void playCompletionSound(); // ��������������� ������� ��� �����

public:
    LoadWindow();
    ~LoadWindow(); // ���������� ������ ������ ��� ��� ����������� �������

    void run();
    // void updateLoadingStage(); // ������ ���������� � update() � ������� � playCompletionSound
};

#endif // LOADWINDOW_H