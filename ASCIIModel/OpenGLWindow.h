// OpenGLWindow.h
#ifndef OPENGLWINDOW_H
#define OPENGLWINDOW_H

#include <glad/glad.h> // ��� ������� OpenGL
#include <GLFW/glfw3.h> // ��� ���������� ����� � ������
#include <iostream>     // ��� ������ ������
#include <functional>

class OpenGLWindow
{
private:
    int windowWidth;
    int windowHeight;
    const char* windowTitle;
    GLFWwindow* window; // ��������� �� ���� GLFW

    std::function<void()> renderCallback;

    bool initialize();        // ������������� GLFW � GLAD
    void setWindowPosition(); // ��������� ������� ����
    void processInput();      // ��������� ����� ������������
    void renderClear();       // ������� ������ (������������� �� render ��� �������, �.�. �������� ������ ������� ����� � Engine)

public:
    OpenGLWindow(int width, int height, const char* title = "OpenGL Window");
    ~OpenGLWindow();


    void run(); // ������ ��������� ����� ����
    int GetWindowWidth() const { return windowWidth; }   // �������� ������ ����
    int GetWindowHeight() const { return windowHeight; } // �������� ������ ����
    GLFWwindow* getGLFWwindow() const { return window; } // �������� ��������� �� ���� GLFW (����� ������������ ��� Engine)

    // ����� ��� ��������� callback-�������
    void setRenderCallback(std::function<void()> callback);

};

#endif // OPENGLWINDOW_H