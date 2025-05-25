// OpenGLWindow.h
#ifndef OPENGLWINDOW_H
#define OPENGLWINDOW_H

#include <glad/glad.h> // Для функций OpenGL
#include <GLFW/glfw3.h> // Для управления окном и вводом
#include <iostream>     // Для вывода ошибок
#include <functional>

class OpenGLWindow
{
private:
    int windowWidth;
    int windowHeight;
    const char* windowTitle;
    GLFWwindow* window; // Указатель на окно GLFW

    std::function<void()> renderCallback;

    bool initialize();        // Инициализация GLFW и GLAD
    void setWindowPosition(); // Установка позиции окна
    void processInput();      // Обработка ввода пользователя
    void renderClear();       // Очистка экрана (переименовано из render для ясности, т.к. основная логика рендера будет в Engine)

public:
    OpenGLWindow(int width, int height, const char* title = "OpenGL Window");
    ~OpenGLWindow();


    void run(); // Запуск основного цикла окна
    int GetWindowWidth() const { return windowWidth; }   // Получить ширину окна
    int GetWindowHeight() const { return windowHeight; } // Получить высоту окна
    GLFWwindow* getGLFWwindow() const { return window; } // Получить указатель на окно GLFW (может понадобиться для Engine)

    // Метод для установки callback-функции
    void setRenderCallback(std::function<void()> callback);

};

#endif // OPENGLWINDOW_H