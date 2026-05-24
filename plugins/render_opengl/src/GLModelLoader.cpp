#include "julia/render_opengl/GLModelLoader.hpp"

#include <assimp/Importer.hpp>
#include <assimp/config.h>
#include <assimp/material.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
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

        struct NormalKey
        {
            int x = 0;
            int y = 0;
            int z = 0;

            bool operator==(const NormalKey& other) const
            {
                return x == other.x && y == other.y && z == other.z;
            }
        };

        struct NormalKeyHash
        {
            std::size_t operator()(const NormalKey& key) const noexcept
            {
                const std::size_t h1 = std::hash<int>{}(key.x);
                const std::size_t h2 = std::hash<int>{}(key.y);
                const std::size_t h3 = std::hash<int>{}(key.z);

                return h1 ^ (h2 << 1u) ^ (h3 << 2u);
            }
        };

        int quantizePosition(float value)
        {
            constexpr float Epsilon = 0.0001f;
            return static_cast<int>(std::round(value / Epsilon));
        }

        NormalKey makeNormalKey(const glm::vec3& position)
        {
            return NormalKey{
                quantizePosition(position.x),
                quantizePosition(position.y),
                quantizePosition(position.z)
            };
        }

        void rebuildSmoothNormals(
            std::vector<GLVertex>& vertices,
            const std::vector<std::uint32_t>& indices
        )
        {
            if (vertices.empty() || indices.size() < 3)
            {
                return;
            }

            std::vector<NormalKey> keys;
            keys.resize(vertices.size());

            std::unordered_map<NormalKey, glm::vec3, NormalKeyHash> normalSums;
            normalSums.reserve(vertices.size());

            for (std::size_t i = 0; i < vertices.size(); ++i)
            {
                keys[i] = makeNormalKey(vertices[i].position);
                normalSums.try_emplace(keys[i], glm::vec3{0.0f});
            }

            for (std::size_t i = 0; i + 2 < indices.size(); i += 3)
            {
                const std::uint32_t i0 = indices[i + 0];
                const std::uint32_t i1 = indices[i + 1];
                const std::uint32_t i2 = indices[i + 2];

                if (
                    i0 >= vertices.size() ||
                    i1 >= vertices.size() ||
                    i2 >= vertices.size()
                )
                {
                    continue;
                }

                const glm::vec3 p0 = vertices[i0].position;
                const glm::vec3 p1 = vertices[i1].position;
                const glm::vec3 p2 = vertices[i2].position;

                glm::vec3 faceNormal = glm::cross(p1 - p0, p2 - p0);

                if (glm::dot(faceNormal, faceNormal) < 0.000000000001f)
                {
                    continue;
                }

                // Если у FBX уже были нормали, используем их только как подсказку
                // для направления, чтобы случайно не перевернуть освещение.
                const glm::vec3 referenceNormal =
                    vertices[i0].normal +
                    vertices[i1].normal +
                    vertices[i2].normal;

                if (
                    glm::dot(referenceNormal, referenceNormal) > 0.000001f &&
                    glm::dot(faceNormal, referenceNormal) < 0.0f
                )
                {
                    faceNormal = -faceNormal;
                }

                normalSums[keys[i0]] += faceNormal;
                normalSums[keys[i1]] += faceNormal;
                normalSums[keys[i2]] += faceNormal;
            }

            for (std::size_t i = 0; i < vertices.size(); ++i)
            {
                const auto found = normalSums.find(keys[i]);

                if (
                    found != normalSums.end() &&
                    glm::dot(found->second, found->second) > 0.00000001f
                )
                {
                    vertices[i].normal = glm::normalize(found->second);
                }
                else if (glm::dot(vertices[i].normal, vertices[i].normal) > 0.00000001f)
                {
                    vertices[i].normal = glm::normalize(vertices[i].normal);
                }
                else
                {
                    vertices[i].normal = glm::vec3{0.0f, 1.0f, 0.0f};
                }
            }
        }

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

            rebuildSmoothNormals(vertices, indices);

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
            aiProcess_DropNormals |
            aiProcess_GenSmoothNormals |
            aiProcess_JoinIdenticalVertices |
            aiProcess_LimitBoneWeights |
            aiProcess_CalcTangentSpace |
            aiProcess_GlobalScale |
            aiProcess_FlipUVs |
            aiProcess_FindInvalidData |
            aiProcess_FindDegenerates |
            aiProcess_ImproveCacheLocality;

        importer.SetPropertyFloat(AI_CONFIG_PP_GSN_MAX_SMOOTHING_ANGLE, 175.0f);

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