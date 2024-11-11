#pragma once
#include <glm.hpp>
#include <vec2.hpp>
#include <vec3.hpp>
#include <vec4.hpp>
#include <mat4x4.hpp>
#include <gtc/matrix_transform.hpp>
#include <gtc/type_ptr.hpp>
#include <iostream>
#include <string>



class SoundEngine {
public:
    void Init(); // ������������� �������� �������
    void LoadSound(const std::string& filePath); // �������� �����
    void PlaySound(int soundID); // ������������ �����
    void SetListenerPosition(const glm::vec3& position); // ������� ��������� (������)
    void SetSoundPosition(int soundID, const glm::vec3& position); // ������� ��������� �����


};
