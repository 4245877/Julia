#include "julia/render_opengl/GLModelLoader.hpp"

#include <assimp/Importer.hpp>
#include <assimp/material.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <glm/glm.hpp>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace julia::render_opengl
{
    namespace
    {
        struct LoadContext
        {
            GLModel model;
            std::unordered_map<std::string, std::uint32_t> boneNameToIndex;
        };

        glm::mat4 toGlm(const aiMatrix4x4& matrix)
        {
            glm::mat4 result{1.0f};

            result[0][0] = matrix.a1;
            result[1][0] = matrix.a2;
            result[2][0] = matrix.a3;
            result[3][0] = matrix.a4;

            result[0][1] = matrix.b1;
            result[1][1] = matrix.b2;
            result[2][1] = matrix.b3;
            result[3][1] = matrix.b4;

            result[0][2] = matrix.c1;
            result[1][2] = matrix.c2;
            result[2][2] = matrix.c3;
            result[3][2] = matrix.c4;

            result[0][3] = matrix.d1;
            result[1][3] = matrix.d2;
            result[2][3] = matrix.d3;
            result[3][3] = matrix.d4;

            return result;
        }

        void addBoneInfluence(GLVertex& vertex, int boneId, float weight)
        {
            for (int i = 0; i < 4; ++i)
            {
                if (vertex.boneIds[i] < 0)
                {
                    vertex.boneIds[i] = boneId;
                    vertex.boneWeights[i] = weight;
                    return;
                }
            }

            // Если влияний больше 4, оставляем самые сильные.
            int weakestIndex = 0;
            float weakestWeight = vertex.boneWeights[0];

            for (int i = 1; i < 4; ++i)
            {
                if (vertex.boneWeights[i] < weakestWeight)
                {
                    weakestWeight = vertex.boneWeights[i];
                    weakestIndex = i;
                }
            }

            if (weight > weakestWeight)
            {
                vertex.boneIds[weakestIndex] = boneId;
                vertex.boneWeights[weakestIndex] = weight;
            }
        }

        void normalizeBoneWeights(GLVertex& vertex)
        {
            const float sum =
                vertex.boneWeights.x +
                vertex.boneWeights.y +
                vertex.boneWeights.z +
                vertex.boneWeights.w;

            if (sum > 0.0001f)
            {
                vertex.boneWeights /= sum;
            }
        }

        GLMeshMaterial loadMaterial(const aiScene* scene, const aiMesh* mesh)
        {
            GLMeshMaterial result{};

            if (mesh->mMaterialIndex >= scene->mNumMaterials)
            {
                return result;
            }

            const aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];

            aiColor3D diffuseColor{0.8f, 0.8f, 0.8f};
            if (material->Get(AI_MATKEY_COLOR_DIFFUSE, diffuseColor) == AI_SUCCESS)
            {
                result.diffuseColor = glm::vec3{
                    diffuseColor.r,
                    diffuseColor.g,
                    diffuseColor.b
                };
            }

            return result;
        }

        GLMesh processMesh(
            const aiScene* scene,
            const aiMesh* mesh,
            const glm::mat4& globalTransform,
            LoadContext& context
        )
        {
            std::vector<GLVertex> vertices;
            std::vector<std::uint32_t> indices;

            vertices.resize(mesh->mNumVertices);

            const glm::mat3 normalTransform =
                glm::mat3(glm::transpose(glm::inverse(globalTransform)));

            for (unsigned int i = 0; i < mesh->mNumVertices; ++i)
            {
                GLVertex vertex{};

                glm::vec3 localPosition{
                    mesh->mVertices[i].x,
                    mesh->mVertices[i].y,
                    mesh->mVertices[i].z
                };

                vertex.position = glm::vec3(
                    globalTransform * glm::vec4(localPosition, 1.0f)
                );

                if (mesh->HasNormals())
                {
                    glm::vec3 localNormal{
                        mesh->mNormals[i].x,
                        mesh->mNormals[i].y,
                        mesh->mNormals[i].z
                    };

                    vertex.normal = glm::normalize(normalTransform * localNormal);
                }

                if (mesh->HasTextureCoords(0))
                {
                    vertex.texCoord = glm::vec2{
                        mesh->mTextureCoords[0][i].x,
                        mesh->mTextureCoords[0][i].y
                    };
                }

                vertices[i] = vertex;
            }

            for (unsigned int faceIndex = 0; faceIndex < mesh->mNumFaces; ++faceIndex)
            {
                const aiFace& face = mesh->mFaces[faceIndex];

                for (unsigned int indexIndex = 0; indexIndex < face.mNumIndices; ++indexIndex)
                {
                    indices.push_back(static_cast<std::uint32_t>(face.mIndices[indexIndex]));
                }
            }

            for (unsigned int boneIndex = 0; boneIndex < mesh->mNumBones; ++boneIndex)
            {
                const aiBone* bone = mesh->mBones[boneIndex];
                const std::string boneName = bone->mName.C_Str();

                std::uint32_t finalBoneIndex = 0;

                const auto existingBone = context.boneNameToIndex.find(boneName);
                if (existingBone != context.boneNameToIndex.end())
                {
                    finalBoneIndex = existingBone->second;
                }
                else
                {
                    GLBoneInfo boneInfo{};
                    boneInfo.name = boneName;
                    boneInfo.offsetMatrix = toGlm(bone->mOffsetMatrix);

                    finalBoneIndex = context.model.addBone(std::move(boneInfo));
                    context.boneNameToIndex.emplace(boneName, finalBoneIndex);
                }

                for (unsigned int weightIndex = 0; weightIndex < bone->mNumWeights; ++weightIndex)
                {
                    const aiVertexWeight& weight = bone->mWeights[weightIndex];

                    if (weight.mVertexId >= vertices.size())
                    {
                        continue;
                    }

                    addBoneInfluence(
                        vertices[weight.mVertexId],
                        static_cast<int>(finalBoneIndex),
                        weight.mWeight
                    );
                }
            }

            for (auto& vertex : vertices)
            {
                normalizeBoneWeights(vertex);
            }

            return GLMesh{
                std::move(vertices),
                std::move(indices),
                loadMaterial(scene, mesh)
            };
        }

        void processNode(
            const aiScene* scene,
            const aiNode* node,
            const glm::mat4& parentTransform,
            LoadContext& context
        )
        {
            const glm::mat4 nodeTransform =
                parentTransform * toGlm(node->mTransformation);

            for (unsigned int i = 0; i < node->mNumMeshes; ++i)
            {
                const unsigned int meshIndex = node->mMeshes[i];
                const aiMesh* mesh = scene->mMeshes[meshIndex];

                context.model.addMesh(
                    processMesh(scene, mesh, nodeTransform, context)
                );
            }

            for (unsigned int i = 0; i < node->mNumChildren; ++i)
            {
                processNode(
                    scene,
                    node->mChildren[i],
                    nodeTransform,
                    context
                );
            }
        }
    }

    GLModel GLModelLoader::loadFromFile(const std::filesystem::path& path)
    {
        if (!std::filesystem::exists(path))
        {
            throw std::runtime_error("Model file does not exist: " + path.string());
        }

        Assimp::Importer importer;

        const unsigned int flags =
            aiProcess_Triangulate |
            aiProcess_GenSmoothNormals |
            aiProcess_JoinIdenticalVertices |
            aiProcess_LimitBoneWeights |
            aiProcess_CalcTangentSpace |
            aiProcess_GlobalScale |
            aiProcess_FlipUVs;

        const aiScene* scene = importer.ReadFile(path.string(), flags);

        if (!scene || !scene->mRootNode || (scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE))
        {
            throw std::runtime_error(
                "Assimp failed to load model: " +
                path.string() +
                "\n" +
                importer.GetErrorString()
            );
        }

        LoadContext context{};
        processNode(scene, scene->mRootNode, glm::mat4{1.0f}, context);

        if (context.model.empty())
        {
            throw std::runtime_error("Model contains no meshes: " + path.string());
        }

        return std::move(context.model);
    }
}