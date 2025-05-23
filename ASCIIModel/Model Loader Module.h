// (Модуль загрузки моделей): загрузка моделей и анимаций с использованием библиотеки Assimp.

#pragma once
#include <glad/glad.h>
#include <glm.hpp>
#include <gtc/matrix_transform.hpp>
#include <vec2.hpp>
#include <vec3.hpp>
#include <vec4.hpp>
#include <mat4x4.hpp>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <vector>
#include <string>
#include <map>
#include <unordered_map>
#include <iostream>


class Bone {
    std::string name;
    glm::mat4 offsetMatrix;
    glm::mat4 finalTransformation;
    // Методы для работы с костями
};


class Animation {
public:
    std::string name;
    float duration;
    float ticksPerSecond;
    std::map<std::string, std::vector<aiVectorKey>> positionKeys;
    std::map<std::string, std::vector<aiQuatKey>> rotationKeys;
    std::map<std::string, std::vector<aiVectorKey>> scaleKeys;
    // Методы для работы с анимацией
};



class Model {
public:
    Model();
    ~Model();

    void loadModel(const std::string& path);
    void animate(const std::string& animationName, float timeInSeconds);
    void render(GLuint shaderProgram);

private:
    //std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    std::vector<Bone> bones;
    std::unordered_map<std::string, Animation> animations;
    std::string directory;

    GLuint VAO, VBO, EBO;
    static const int MAX_BONE_INFLUENCE = 4;




    void setupMesh();
    void processNode(aiNode* node, const aiScene* scene);
    void processMesh(aiMesh* mesh, const aiScene* scene);
    void loadAnimations(const aiScene* scene);
    void calculateBoneTransforms(float animationTime, const std::string& animationName);
    glm::vec3 interpolatePosition(float animationTime, const std::vector<aiVectorKey>& keys);
    void loadBones(aiMesh* mesh); // для обработки костей
};

