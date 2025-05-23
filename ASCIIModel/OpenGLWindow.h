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
    float windowWidth;
    float windowHeight;
    const char* windowTitle;
    GLFWwindow* window; // ��������� �� ���� GLFW

    bool initialize();        // ������������� GLFW � GLAD
    void setWindowPosition(); // ��������� ������� ����
    void processInput();      // ��������� ����� ������������
    void renderClear();       // ������� ������ (������������� �� render ��� �������, �.�. �������� ������ ������� ����� � Engine)

public:
    OpenGLWindow(float width, float height, const char* title = "OpenGL Window");
    ~OpenGLWindow();

    void run(); // ������ ��������� ����� ����
    float GetWindowWidth() const { return windowWidth; }   // �������� ������ ����
    float GetWindowHeight() const { return windowHeight; } // �������� ������ ����
    GLFWwindow* getGLFWwindow() const { return window; } // �������� ��������� �� ���� GLFW (����� ������������ ��� Engine)

    // ����� ��� ��������� callback-�������
    void setRenderCallback(std::function<void()> callback);

};

#endif // OPENGLWINDOW_H