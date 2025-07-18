// Camera.h
#pragma once // ������ �� �������� ���������

#include <glm.hpp> // ��� glm::vec3, glm::mat4
#include <gtc/matrix_transform.hpp> // ��� glm::lookAt

// ���������� ��������� ��������� ��������� �������� ������
// ������������� ������������ enum class ��� ������ ������� ��������� � ����������������
enum class CameraMovement {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
};

// ��������� ������ �� ���������
// ����� ������� �� static constexpr ������ ������ ��� � .cpp �����, ���� ��� ����� ������ ���
// ����, ���� ��� ������������� ���������� ��������� ��� ������, �������� �����.
const float DEFAULT_YAW = -90.0f;
const float DEFAULT_PITCH = 0.0f;
const float DEFAULT_SPEED = 2.5f;
const float DEFAULT_SENSITIVITY = 0.1f;
const float DEFAULT_ZOOM = 45.0f;

class Camera {
public:
    // �������� ������
    glm::vec3 Position;
    glm::vec3 Front;
    glm::vec3 Up;
    glm::vec3 Right;
    glm::vec3 WorldUp;

    // ���� ������
    float Yaw;
    float Pitch;

    // ��������� ������
    float MovementSpeed;
    float MouseSensitivity;
    float Zoom;

    // �����������, ������������ �������
    Camera(glm::vec3 position = glm::vec3(0.0f, 0.0f, 0.0f),
        glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f),
        float yaw = DEFAULT_YAW,
        float pitch = DEFAULT_PITCH);

    // �����������, ������������ �������
    Camera(float posX, float posY, float posZ,
        float upX, float upY, float upZ,
        float yaw, float pitch);

    // ���������� ������� ����
    glm::mat4 GetViewMatrix();

    // ������������ ���� � ����������
    void ProcessKeyboard(CameraMovement direction, float deltaTime);

    // ������������ ���� � ���� ��� ��������� ����� ������ (������)
    // void ProcessMouseMovement(float xoffset, float yoffset, bool constrainPitch = true);

    // ������������ ���� �������� ���� ��� ���� (������)
    // void ProcessMouseScroll(float yoffset);

private:
    // ��������� ������� ������ �� ������ ����� ������
    void updateCameraVectors();
};