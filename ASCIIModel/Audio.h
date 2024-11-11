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
    void Init(); // Инициализация звуковой системы
    void LoadSound(const std::string& filePath); // Загрузка звука
    void PlaySound(int soundID); // Проигрывание звука
    void SetListenerPosition(const glm::vec3& position); // Позиция слушателя (камера)
    void SetSoundPosition(int soundID, const glm::vec3& position); // Позиция источника звука


};
