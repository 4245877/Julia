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
    

}
void Model::loadBones(aiMesh* mesh) {

}
void Model::loadAnimations(const aiScene* scene) {
   
}
void Model::calculateBoneTransforms(float animationTime, const std::string& animationName) {}
glm::vec3 Model::interpolatePosition(float animationTime, const std::vector<aiVectorKey>& keys){
	return glm::vec3();
}
