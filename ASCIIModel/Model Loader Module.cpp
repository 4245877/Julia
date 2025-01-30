#include "Model Loader Module.h"

//===================public часть===================

Model::Model() {
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);
}
Model::~Model() {
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
}

void Model::loadModel(const std::string& path) {
    Assimp::Importer import;
    const aiScene* scene = import.ReadFile(path, aiProcess_Triangulate | aiProcess_FlipUVs);

    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)
    {
        std::cerr << "ERROR::ASSIMP::" << import.GetErrorString() << std::endl;
        return;
    }
    directory = path.substr(0, path.find_last_of('/'));

    processNode(scene->mRootNode, scene);
}
void Model::animate(const std::string& animationName, float timeInSeconds) {
    if (animations.find(animationName) == animations.end()) return;

    auto& anim = animations[animationName];
    float timeInTicks = timeInSeconds * anim.ticksPerSecond;
    float animationTime = fmod(timeInTicks, anim.duration);

    calculateBoneTransforms(animationTime, animationName);
}
void Model::render(GLuint shaderProgram) {
    // Отправка матриц костей в шейдер
}


//===================private часть===================
void Model::setupMesh() {
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);

    // Vertex positions
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);

    // Vertex normals
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));

    // Texture coords
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, texCoords));

    // Bone weights
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, boneWeights));

    // Bone IDs
    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, boneIDs));

    glBindVertexArray(0);
}
void Model::processNode(aiNode* node, const aiScene* scene) {
    // Обработка всех мешей узла
    for (unsigned int i = 0; i < node->mNumMeshes; i++) {
        aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
        processMesh(mesh, scene);
    }

    // Рекурсивно обработать дочерние узлы
    for (unsigned int i = 0; i < node->mNumChildren; i++) {
        processNode(node->mChildren[i], scene);
    }
}
void Model::processMesh(aiMesh* mesh, const aiScene* scene)
{
    for (unsigned int i = 0; i < mesh->mNumVertices; i++) {
        Vertex vertex;

        // Позиция
        vertex.position.x = mesh->mVertices[i].x;
        vertex.position.y = mesh->mVertices[i].y;
        vertex.position.z = mesh->mVertices[i].z;

        // Нормали
        if (mesh->HasNormals()) {
            vertex.normal.x = mesh->mNormals[i].x;
            vertex.normal.y = mesh->mNormals[i].y;
            vertex.normal.z = mesh->mNormals[i].z;
        }

        // Текстурные координаты
        if (mesh->mTextureCoords[0]) {
            vertex.texCoords.x = mesh->mTextureCoords[0][i].x;
            vertex.texCoords.y = mesh->mTextureCoords[0][i].y;
        }

        vertices.push_back(vertex);
    }

}
void Model::loadBones(aiMesh* mesh) {

}
void Model::loadAnimations(const aiScene* scene) {
    for (unsigned int i = 0; i < scene->mNumAnimations; i++) {
        aiAnimation* anim = scene->mAnimations[i];
        Animation animation;
        animation.name = anim->mName.C_Str();
        animation.duration = anim->mDuration;
        animation.ticksPerSecond = anim->mTicksPerSecond != 0 ? anim->mTicksPerSecond : 25.0f;

        for (unsigned int j = 0; j < anim->mNumChannels; j++) {
            aiNodeAnim* channel = anim->mChannels[j];
            std::string nodeName = channel->mNodeName.C_Str();

            // Сохранение ключевых кадров
            for (unsigned int k = 0; k < channel->mNumPositionKeys; k++) {
                animation.positionKeys[nodeName].push_back(channel->mPositionKeys[k]);
            }
            for (unsigned int k = 0; k < channel->mNumRotationKeys; k++) {
                animation.rotationKeys[nodeName].push_back(channel->mRotationKeys[k]);
            }
            for (unsigned int k = 0; k < channel->mNumScalingKeys; k++) {
                animation.scaleKeys[nodeName].push_back(channel->mScalingKeys[k]);
            }
        }

        animations[animation.name] = animation;
    }
}
void Model::calculateBoneTransforms(float animationTime, const std::string& animationName) {
    
}
glm::vec3 Model::interpolatePosition(float animationTime, const std::vector<aiVectorKey>& keys) {
    if (keys.empty()) return glm::vec3(0.0f);
    if (keys.size() == 1) return glm::vec3(keys[0].mValue.x, keys[0].mValue.y, keys[0].mValue.z);

    // Найти текущий и следующий ключевой кадр
    unsigned int nextFrame = 0;
    for (unsigned int i = 0; i < keys.size() - 1; i++) {
        if (animationTime < keys[i + 1].mTime) {
            nextFrame = i + 1;
            break;
        }
    }
    unsigned int currentFrame = nextFrame - 1;

    // Вычислить фактор интерполяции
    float delta = keys[nextFrame].mTime - keys[currentFrame].mTime;
    float factor = (animationTime - keys[currentFrame].mTime) / delta;

    const aiVector3D& start = keys[currentFrame].mValue;
    const aiVector3D& end = keys[nextFrame].mValue;

    // Линейная интерполяция
    return glm::vec3(
        start.x + factor * (end.x - start.x),
        start.y + factor * (end.y - start.y),
        start.z + factor * (end.z - start.z)
    );
}
