// Camera.cpp
#include "Camera.h" // Включаем объявление нашего класса
#include <vector>   // Не используется напрямую здесь, но часто нужен для glm если не включен транзитивно
#include <cmath>    // Для std::sin, std::cos, если не включены транзитивно через glm
#include <glm.hpp>
// Конструктор, использующий векторы
Camera::Camera(glm::vec3 position, glm::vec3 up, float yaw, float pitch)
    : Front(glm::vec3(0.0f, 0.0f, -1.0f)), // Начальное значение Front
    MovementSpeed(DEFAULT_SPEED),
    MouseSensitivity(DEFAULT_SENSITIVITY),
    Zoom(DEFAULT_ZOOM) {
    Position = position;
    WorldUp = up;
    Yaw = yaw;
    Pitch = pitch;
    updateCameraVectors();
}

// Конструктор, использующий скаляры
Camera::Camera(float posX, float posY, float posZ, float upX, float upY, float upZ, float yaw, float pitch)
    : Front(glm::vec3(0.0f, 0.0f, -1.0f)),
    MovementSpeed(DEFAULT_SPEED),
    MouseSensitivity(DEFAULT_SENSITIVITY),
    Zoom(DEFAULT_ZOOM) {
    Position = glm::vec3(posX, posY, posZ);
    WorldUp = glm::vec3(upX, upY, upZ);
    Yaw = yaw;
    Pitch = pitch;
    updateCameraVectors();
}

// Возвращает матрицу вида
glm::mat4 Camera::GetViewMatrix() {
    // Перед получением матрицы вида убедимся, что векторы камеры обновлены,
    // если Pitch или Yaw могли измениться извне без вызова updateCameraVectors.
    // Однако, если Pitch/Yaw меняются только через ProcessMouseMovement, 
    // который вызывает updateCameraVectors, этот вызов здесь не обязателен.
    // updateCameraVectors(); // Раскомментируйте, если необходимо
    return glm::lookAt(Position, Position + Front, Up);
}

// Обрабатывает ввод с клавиатуры
void Camera::ProcessKeyboard(CameraMovement direction, float deltaTime) {
    float velocity = MovementSpeed * deltaTime;
    if (direction == CameraMovement::FORWARD)
        Position += Front * velocity;
    if (direction == CameraMovement::BACKWARD)
        Position -= Front * velocity;
    if (direction == CameraMovement::LEFT)
        Position -= Right * velocity;
    if (direction == CameraMovement::RIGHT)
        Position += Right * velocity;
    // Для полета можно добавить обработку движения по WorldUp/WorldDown
    // Position.y = 0.0f; // Опционально: если камера должна оставаться на "земле" (FPS-стиль)
}

// Вычисляет векторы камеры на основе углов Эйлера
void Camera::updateCameraVectors() {
    // Вычисляем новый вектор Front
    glm::vec3 frontVec;
    frontVec.x = std::cos(glm::radians(Yaw)) * std::cos(glm::radians(Pitch));
    frontVec.y = std::sin(glm::radians(Pitch));
    frontVec.z = std::sin(glm::radians(Yaw)) * std::cos(glm::radians(Pitch));
    Front = glm::normalize(frontVec);

    // Также пересчитываем векторы Right и Up
    Right = glm::normalize(glm::cross(Front, WorldUp)); // Нормализуем, т.к. их длина может меняться из-за округления
    Up = glm::normalize(glm::cross(Right, Front));
}

/*
// Пример реализации ProcessMouseMovement (если решите добавить)
void Camera::ProcessMouseMovement(float xoffset, float yoffset, bool constrainPitch) {
    xoffset *= MouseSensitivity;
    yoffset *= MouseSensitivity;

    Yaw += xoffset;
    Pitch += yoffset;

    // Ограничиваем Pitch, чтобы избежать "переворота" экрана
    if (constrainPitch) {
        if (Pitch > 89.0f)
            Pitch = 89.0f;
        if (Pitch < -89.0f)
            Pitch = -89.0f;
    }
    // Обновляем векторы Front, Right и Up на основе обновленных углов Эйлера
    updateCameraVectors();
}

// Пример реализации ProcessMouseScroll (если решите добавить)
void Camera::ProcessMouseScroll(float yoffset) {
    Zoom -= (float)yoffset; // yoffset обычно представляет собой количество "шагов" колесика
    if (Zoom < 1.0f)
        Zoom = 1.0f;
    if (Zoom > 45.0f) // Или другое максимальное значение
        Zoom = 45.0f;
    // Zoom обычно используется для изменения поля зрения (FOV) в матрице проекции,
    // а не напрямую в GetViewMatrix, если это не камера с эффектом " dolly zoom".
}
*/