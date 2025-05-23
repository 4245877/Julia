// Camera.h
#pragma once // Защита от двойного включения

#include <glm.hpp> // Для glm::vec3, glm::mat4
#include <gtc/matrix_transform.hpp> // Для glm::lookAt

// Определяем несколько возможных вариантов движения камеры
// Рекомендуется использовать enum class для лучшей области видимости и типобезопасности
enum class CameraMovement {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
};

// Параметры камеры по умолчанию
// Лучше сделать их static constexpr внутри класса или в .cpp файле, если они нужны только там
// Либо, если они действительно глобальные константы для камеры, оставить здесь.
const float DEFAULT_YAW = -90.0f;
const float DEFAULT_PITCH = 0.0f;
const float DEFAULT_SPEED = 2.5f;
const float DEFAULT_SENSITIVITY = 0.1f;
const float DEFAULT_ZOOM = 45.0f;

class Camera {
public:
    // Атрибуты камеры
    glm::vec3 Position;
    glm::vec3 Front;
    glm::vec3 Up;
    glm::vec3 Right;
    glm::vec3 WorldUp;

    // Углы Эйлера
    float Yaw;
    float Pitch;

    // Настройки камеры
    float MovementSpeed;
    float MouseSensitivity;
    float Zoom;

    // Конструктор, использующий векторы
    Camera(glm::vec3 position = glm::vec3(0.0f, 0.0f, 0.0f),
        glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f),
        float yaw = DEFAULT_YAW,
        float pitch = DEFAULT_PITCH);

    // Конструктор, использующий скаляры
    Camera(float posX, float posY, float posZ,
        float upX, float upY, float upZ,
        float yaw, float pitch);

    // Возвращает матрицу вида
    glm::mat4 GetViewMatrix();

    // Обрабатывает ввод с клавиатуры
    void ProcessKeyboard(CameraMovement direction, float deltaTime);

    // Обрабатывает ввод с мыши для изменения углов камеры (пример)
    // void ProcessMouseMovement(float xoffset, float yoffset, bool constrainPitch = true);

    // Обрабатывает ввод колесика мыши для зума (пример)
    // void ProcessMouseScroll(float yoffset);

private:
    // Вычисляет векторы камеры на основе углов Эйлера
    void updateCameraVectors();
};