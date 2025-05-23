// Camera.cpp
#include "Camera.h" // �������� ���������� ������ ������
#include <vector>   // �� ������������ �������� �����, �� ����� ����� ��� glm ���� �� ������� �����������
#include <cmath>    // ��� std::sin, std::cos, ���� �� �������� ����������� ����� glm
#include <glm.hpp>
// �����������, ������������ �������
Camera::Camera(glm::vec3 position, glm::vec3 up, float yaw, float pitch)
    : Front(glm::vec3(0.0f, 0.0f, -1.0f)), // ��������� �������� Front
    MovementSpeed(DEFAULT_SPEED),
    MouseSensitivity(DEFAULT_SENSITIVITY),
    Zoom(DEFAULT_ZOOM) {
    Position = position;
    WorldUp = up;
    Yaw = yaw;
    Pitch = pitch;
    updateCameraVectors();
}

// �����������, ������������ �������
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

// ���������� ������� ����
glm::mat4 Camera::GetViewMatrix() {
    // ����� ���������� ������� ���� ��������, ��� ������� ������ ���������,
    // ���� Pitch ��� Yaw ����� ���������� ����� ��� ������ updateCameraVectors.
    // ������, ���� Pitch/Yaw �������� ������ ����� ProcessMouseMovement, 
    // ������� �������� updateCameraVectors, ���� ����� ����� �� ����������.
    // updateCameraVectors(); // ����������������, ���� ����������
    return glm::lookAt(Position, Position + Front, Up);
}

// ������������ ���� � ����������
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
    // ��� ������ ����� �������� ��������� �������� �� WorldUp/WorldDown
    // Position.y = 0.0f; // �����������: ���� ������ ������ ���������� �� "�����" (FPS-�����)
}

// ��������� ������� ������ �� ������ ����� ������
void Camera::updateCameraVectors() {
    // ��������� ����� ������ Front
    glm::vec3 frontVec;
    frontVec.x = std::cos(glm::radians(Yaw)) * std::cos(glm::radians(Pitch));
    frontVec.y = std::sin(glm::radians(Pitch));
    frontVec.z = std::sin(glm::radians(Yaw)) * std::cos(glm::radians(Pitch));
    Front = glm::normalize(frontVec);

    // ����� ������������� ������� Right � Up
    Right = glm::normalize(glm::cross(Front, WorldUp)); // �����������, �.�. �� ����� ����� �������� ��-�� ����������
    Up = glm::normalize(glm::cross(Right, Front));
}

/*
// ������ ���������� ProcessMouseMovement (���� ������ ��������)
void Camera::ProcessMouseMovement(float xoffset, float yoffset, bool constrainPitch) {
    xoffset *= MouseSensitivity;
    yoffset *= MouseSensitivity;

    Yaw += xoffset;
    Pitch += yoffset;

    // ������������ Pitch, ����� �������� "����������" ������
    if (constrainPitch) {
        if (Pitch > 89.0f)
            Pitch = 89.0f;
        if (Pitch < -89.0f)
            Pitch = -89.0f;
    }
    // ��������� ������� Front, Right � Up �� ������ ����������� ����� ������
    updateCameraVectors();
}

// ������ ���������� ProcessMouseScroll (���� ������ ��������)
void Camera::ProcessMouseScroll(float yoffset) {
    Zoom -= (float)yoffset; // yoffset ������ ������������ ����� ���������� "�����" ��������
    if (Zoom < 1.0f)
        Zoom = 1.0f;
    if (Zoom > 45.0f) // ��� ������ ������������ ��������
        Zoom = 45.0f;
    // Zoom ������ ������������ ��� ��������� ���� ������ (FOV) � ������� ��������,
    // � �� �������� � GetViewMatrix, ���� ��� �� ������ � �������� " dolly zoom".
}
*/