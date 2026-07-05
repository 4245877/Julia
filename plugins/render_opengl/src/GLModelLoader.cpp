#include "julia/render_opengl/GLModelLoader.hpp"
#include "julia/render_opengl/GLTexture.hpp"

#include <assimp/Importer.hpp>
#include <assimp/config.h>
#include <assimp/material.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <glm/glm.hpp>

#include <algorithm>
#include <cstdint>
#include <cctype>
#include <filesystem>
#include <iostream>
#include <memory>
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

            std::filesystem::path modelDirectory;
            std::filesystem::path texturesDirectory;

            std::unordered_map<std::string, std::shared_ptr<GLTexture>> textureCache;
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

        std::string toLower(std::string value)
        {
            std::transform(
                value.begin(),
                value.end(),
                value.begin(),
                [](unsigned char c)
                {
                    return static_cast<char>(std::tolower(c));
                }
            );

            return value;
        }

        std::filesystem::path normalizeTexturePath(const aiString& value)
        {
            std::string raw = value.C_Str();

            std::replace(raw.begin(), raw.end(), '\\', '/');

            return std::filesystem::path{raw};
        }

        std::string materialMeshName(const aiMaterial* material, const aiMesh* mesh)
        {
            std::string name;

            aiString materialName;
            if (material->Get(AI_MATKEY_NAME, materialName) == AI_SUCCESS)
            {
                name += materialName.C_Str();
                name += " ";
            }

            if (mesh && mesh->mName.length > 0)
            {
                name += mesh->mName.C_Str();
            }

            return toLower(name);
        }

        void applyMaterialLightingProfile(
            GLMeshMaterial& material,
            const std::string& name
        )
        {
            if (
                name.find("shield") != std::string::npos ||
                name.find("armor") != std::string::npos ||
                name.find("metal") != std::string::npos
            )
            {
                material.lightResponse = 0.90f;
                material.specularStrength = 0.018f;
                material.rimStrength = 0.038f;
            }
            else if (name.find("hair") != std::string::npos)
            {
                material.lightResponse = 0.84f;
                material.specularStrength = 0.006f;
                material.rimStrength = 0.042f;
            }
            else if (
                name.find("skin") != std::string::npos ||
                name.find("face") != std::string::npos ||
                name.find("ear") != std::string::npos
            )
            {
                material.lightResponse = 0.78f;
                material.specularStrength = 0.003f;
                material.rimStrength = 0.030f;
            }
            else if (
                name.find("shirt") != std::string::npos ||
                name.find("cloth") != std::string::npos ||
                name.find("glove") != std::string::npos ||
                name.find("sock") != std::string::npos ||
                name.find("underwear") != std::string::npos ||
                name.find("body") != std::string::npos
            )
            {
                material.lightResponse = 0.74f;
                material.specularStrength = 0.002f;
                material.rimStrength = 0.026f;
            }
            else if (
                name.find("belt") != std::string::npos ||
                name.find("shoe") != std::string::npos ||
                name.find("leather") != std::string::npos ||
                name.find("cuff") != std::string::npos ||
                name.find("bow") != std::string::npos
            )
            {
                material.lightResponse = 0.78f;
                material.specularStrength = 0.005f;
                material.rimStrength = 0.030f;
            }
            else if (
                name.find("eye") != std::string::npos ||
                name.find("highlight") != std::string::npos ||
                // "hightlight" is a misspelling baked into the source asset's
                // material names; matched intentionally, not a typo here.
                name.find("hightlight") != std::string::npos
            )
            {
                material.lightResponse = 0.88f;
                material.specularStrength = 0.026f;
                material.rimStrength = 0.015f;
            }
        }

        std::filesystem::path resolveTexturePath(
            const std::filesystem::path& rawTexturePath,
            const LoadContext& context
        )
        {
            if (rawTexturePath.empty())
            {
                return {};
            }

            std::vector<std::filesystem::path> candidates;

            if (rawTexturePath.is_absolute())
            {
                candidates.push_back(rawTexturePath);
            }
            else
            {
                candidates.push_back(context.modelDirectory / rawTexturePath);
                candidates.push_back(context.modelDirectory / rawTexturePath.filename());

                candidates.push_back(context.texturesDirectory / rawTexturePath);
                candidates.push_back(context.texturesDirectory / rawTexturePath.filename());
            }

            for (const auto& candidate : candidates)
            {
                if (std::filesystem::exists(candidate))
                {
                    return candidate;
                }
            }

            return {};
        }

        std::filesystem::path fallbackTexturePathForMaterial(
            const aiMaterial* material,
            const aiMesh* mesh,
            const LoadContext& context
        )
        {
            const std::string name = materialMeshName(material, mesh);
            std::filesystem::path fileName;

            if (name.find("hair") != std::string::npos)
            {
                fileName = "hair_color.png";
            }
            else if (
                name.find("body") != std::string::npos ||
                name.find("skin") != std::string::npos ||
                name.find("face") != std::string::npos
            )
            {
                fileName = "body_color.png";
            }
            else if (name.find("shield") != std::string::npos)
            {
                fileName = "shield_color.png";
            }

            if (fileName.empty())
            {
                return {};
            }

            const std::filesystem::path candidate = context.texturesDirectory / fileName;

            if (std::filesystem::exists(candidate))
            {
                return candidate;
            }

            return {};
        }

        std::shared_ptr<GLTexture> loadTextureCached(
            const std::filesystem::path& path,
            LoadContext& context
        )
        {
            if (path.empty())
            {
                return nullptr;
            }

            const std::string key =
                std::filesystem::absolute(path).lexically_normal().string();

            const auto found = context.textureCache.find(key);
            if (found != context.textureCache.end())
            {
                return found->second;
            }

            auto texture = std::make_shared<GLTexture>(
                GLTexture::loadFromFile(path)
            );

            context.textureCache.emplace(key, texture);

            std::cout << "Loaded texture: " << path.string() << "\n";

            return texture;
        }

        GLMeshMaterial loadMaterial(
            const aiScene* scene,
            const aiMesh* mesh,
            LoadContext& context
        )
        {
            GLMeshMaterial result{};

            if (mesh->mMaterialIndex >= scene->mNumMaterials)
            {
                return result;
            }

            const aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];
            const std::string materialName = materialMeshName(material, mesh);

            applyMaterialLightingProfile(result, materialName);

            aiColor3D diffuseColor{0.8f, 0.8f, 0.8f};
            if (material->Get(AI_MATKEY_COLOR_DIFFUSE, diffuseColor) == AI_SUCCESS)
            {
                result.diffuseColor = glm::vec3{
                    diffuseColor.r,
                    diffuseColor.g,
                    diffuseColor.b
                };
            }

            std::filesystem::path texturePath;

            aiString diffuseTextureName;
            if (material->GetTexture(aiTextureType_DIFFUSE, 0, &diffuseTextureName) == AI_SUCCESS)
            {
                texturePath = resolveTexturePath(
                    normalizeTexturePath(diffuseTextureName),
                    context
                );
            }

            if (texturePath.empty())
            {
                texturePath = fallbackTexturePathForMaterial(material, mesh, context);
            }

            if (!texturePath.empty())
            {
                result.diffuseTexture = loadTextureCached(texturePath, context);
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
            const bool hasNormals = mesh->HasNormals();

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

                if (hasNormals)
                {
                    glm::vec3 localNormal{
                        mesh->mNormals[i].x,
                        mesh->mNormals[i].y,
                        mesh->mNormals[i].z
                    };

                    const glm::vec3 transformedNormal = normalTransform * localNormal;

                    if (glm::dot(transformedNormal, transformedNormal) > 0.00000001f)
                    {
                        vertex.normal = glm::normalize(transformedNormal);
                    }
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

                // Draw calls use GL_TRIANGLES, so a non-triangle face (a line or
                // point left behind by aiProcess_FindDegenerates) would shift the
                // whole index stream and corrupt the rest of the mesh. Skip them;
                // aiProcess_SortByPType also strips such primitives at import.
                if (face.mNumIndices != 3)
                {
                    continue;
                }

                indices.push_back(static_cast<std::uint32_t>(face.mIndices[0]));
                indices.push_back(static_cast<std::uint32_t>(face.mIndices[1]));
                indices.push_back(static_cast<std::uint32_t>(face.mIndices[2]));
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

                    // We bake the node's globalTransform into vertex positions
                    // (see above), but the Assimp offset matrix expects vertices
                    // in the mesh's original local space. Compose it with the
                    // inverse transform so that later skinning,
                    //   boneMatrix * offset' * bakedPos
                    //     == boneMatrix * offset * (G^-1 * G * localPos)
                    //     == boneMatrix * offset * localPos,
                    // reproduces the correct result. (Assumes a bone shared
                    // across meshes uses a consistent node transform, which
                    // holds for typical rigged FBX exports.)
                    boneInfo.offsetMatrix =
                        toGlm(bone->mOffsetMatrix) * glm::inverse(globalTransform);

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
                loadMaterial(scene, mesh, context)
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
            aiProcess_FlipUVs |
            aiProcess_FindInvalidData |
            aiProcess_FindDegenerates |
            aiProcess_SortByPType |
            aiProcess_ImproveCacheLocality;

        importer.SetPropertyFloat(AI_CONFIG_PP_GSN_MAX_SMOOTHING_ANGLE, 75.0f);

        // FindDegenerates turns degenerate triangles into points/lines; ask
        // SortByPType to drop those primitives outright so only real triangles
        // reach the mesh processor.
        importer.SetPropertyInteger(
            AI_CONFIG_PP_SBP_REMOVE,
            aiPrimitiveType_POINT | aiPrimitiveType_LINE
        );

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
        context.modelDirectory = path.parent_path();
        context.texturesDirectory = path.parent_path().parent_path() / "textures";

        processNode(scene, scene->mRootNode, glm::mat4{1.0f}, context);

        if (context.model.empty())
        {
            throw std::runtime_error("Model contains no meshes: " + path.string());
        }

        if (context.model.boneCount() > MaxBones)
        {
            std::cerr
                << "Warning: model '" << path.string() << "' has "
                << context.model.boneCount() << " bones, but the skinning "
                << "palette holds only " << MaxBones
                << ". Influences beyond that index will not animate.\n";
        }

        return std::move(context.model);
    }
}
