#include "julia/render_opengl/GLModelLoader.hpp"
#include "julia/render_opengl/GLTexture.hpp"

#include <assimp/Importer.hpp>
#include <assimp/GltfMaterial.h>
#include <assimp/config.h>
#include <assimp/material.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <glm/glm.hpp>

#include <algorithm>
#include <cstdint>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <initializer_list>
#include <memory>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace julia::render_opengl
{
    namespace
    {
        struct LoadContext
        {
            GLModel model;

            std::unordered_map<std::string, std::vector<std::uint32_t>> boneNameToIndices;
            std::unordered_map<const aiNode*, std::uint32_t> nodeToIndex;
            std::unordered_map<std::string, std::vector<std::uint32_t>> nodeNameToIndices;

            std::filesystem::path modelDirectory;
            std::filesystem::path texturesDirectory;
            bool useLegacyMaterialTextureFallback = false;

            std::unordered_map<std::string, std::shared_ptr<GLTexture>> textureCache;
        };

        std::string utf8Path(const std::filesystem::path& path)
        {
            const auto value = path.u8string();
            return {value.begin(), value.end()};
        }

        bool finite(const glm::vec3& value)
        {
            return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
        }

        bool finite(const glm::mat4& matrix)
        {
            for (int column = 0; column < 4; ++column)
                for (int row = 0; row < 4; ++row)
                    if (!std::isfinite(matrix[column][row])) return false;
            return true;
        }

        bool equivalent(const glm::mat4& a, const glm::mat4& b)
        {
            for (int column = 0; column < 4; ++column)
                for (int row = 0; row < 4; ++row)
                    if (a[column][row] != b[column][row]) return false;
            return true;
        }

        GLenum textureWrap(aiTextureMapMode mode)
        {
            switch (mode)
            {
                case aiTextureMapMode_Wrap: return GL_REPEAT;
                case aiTextureMapMode_Clamp: return GL_CLAMP_TO_EDGE;
                case aiTextureMapMode_Mirror: return GL_MIRRORED_REPEAT;
                case aiTextureMapMode_Decal: return GL_CLAMP_TO_BORDER;
                default: throw std::runtime_error("Unsupported material texture wrap mode");
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
            if (!std::isfinite(weight) || weight < 0.0f)
                throw std::runtime_error("Model contains an invalid bone weight");
            if (weight == 0.0f) return;
            for (int i = 0; i < 4; ++i)
            {
                if (vertex.boneIds[i] == boneId)
                {
                    vertex.boneWeights[i] += weight;
                    if (!std::isfinite(vertex.boneWeights[i]))
                        throw std::runtime_error("Model bone weight sum overflows");
                    return;
                }
            }
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
            const double sum =
                static_cast<double>(vertex.boneWeights.x) +
                vertex.boneWeights.y +
                vertex.boneWeights.z +
                vertex.boneWeights.w;

            if (sum > 0.0)
            {
                for (int i = 0; i < 4; ++i)
                    vertex.boneWeights[i] = static_cast<float>(vertex.boneWeights[i] / sum);
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

            return std::filesystem::path{std::u8string{raw.begin(), raw.end()}};
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

        std::vector<std::string> materialWords(const std::string& name)
        {
            std::vector<std::string> words;
            std::string word;
            const auto flush = [&] {
                if (!word.empty()) words.push_back(std::exchange(word, {}));
            };
            for (std::size_t i = 0; i < name.size(); ++i)
            {
                const auto c = static_cast<unsigned char>(name[i]);
                if (!std::isalpha(c))
                {
                    flush();
                    continue;
                }
                // Exporters use underscores, camelCase and numeric suffixes.
                // Match complete words so "ear" does not capture underwear/pearl.
                if (i > 0 && std::isupper(c) &&
                    std::islower(static_cast<unsigned char>(name[i - 1])))
                    flush();
                word += static_cast<char>(std::tolower(c));
            }
            flush();
            return words;
        }

        bool hasMaterialWord(const std::vector<std::string>& words,
            std::initializer_list<const char*> names)
        {
            return std::any_of(words.begin(), words.end(), [&](const auto& word) {
                return std::any_of(names.begin(), names.end(), [&](const char* name) { return word == name; });
            });
        }

        ToonMaterialKind specificMaterialKind(const std::vector<std::string>& words)
        {
            if (hasMaterialWord(words, {"eyelash", "eyelashes", "eyebrow", "eyebrows", "brow", "brows", "lash", "lashes", "ink"}))
                return ToonMaterialKind::Ink;
            if (hasMaterialWord(words, {"eye", "eyes", "iris", "pupil", "cornea", "highlight", "hightlight"}))
                return ToonMaterialKind::Eye;
            if (hasMaterialWord(words, {"shield", "armor", "armour", "shoulderarmor", "metal", "sword", "blade"}))
                return ToonMaterialKind::Metal;
            // Accessories such as hair_band have fabric semantics despite "hair".
            if (hasMaterialWord(words, {"shirt", "cloth", "clothes", "clothing", "glove", "gloves",
                "sock", "socks", "stocking", "stockings", "underwear", "dress", "underdress",
                "coat", "overcoat", "cape", "ribbon", "band"}))
                return ToonMaterialKind::Fabric;
            if (hasMaterialWord(words, {"skin", "face", "ear", "ears", "earinsides"}))
                return ToonMaterialKind::Skin;
            if (hasMaterialWord(words, {"belt", "shoe", "shoes", "boot", "boots", "leather", "cuff",
                "bow", "scabbard", "scrabbard"}))
                return ToonMaterialKind::Leather;
            if (hasMaterialWord(words, {"hair", "facialhair", "strand", "strands", "tail", "fur"}))
                return ToonMaterialKind::Hair;
            return ToonMaterialKind::Generic;
        }

        void applyMaterialLightingProfile(GLMeshMaterial& material,
            const aiMaterial* source, const aiMesh* mesh)
        {
            const auto meshWords = materialWords(mesh ? mesh->mName.C_Str() : "");
            aiString sourceName;
            source->Get(AI_MATKEY_NAME, sourceName);
            const auto sourceWords = materialWords(sourceName.C_Str());
            // A single body atlas is often shared by eyes, skin and clothing.
            // Prefer the specific mesh semantics over a generic atlas name.
            auto kind = specificMaterialKind(meshWords);
            if (kind == ToonMaterialKind::Generic) kind = specificMaterialKind(sourceWords);
            if (kind == ToonMaterialKind::Generic &&
                (hasMaterialWord(meshWords, {"body"}) || hasMaterialWord(sourceWords, {"body"})))
                kind = ToonMaterialKind::Fabric;
            material.toonKind = kind;
            switch (kind)
            {
                case ToonMaterialKind::Skin:
                    material.lightResponse = 0.78f;
                    material.specularStrength = 0.035f;
                    material.rimStrength = 0.030f;
                    material.specularPower = 48.0f;
                    material.outlineScale = 0.5f;
                    break;
                case ToonMaterialKind::Hair:
                    material.lightResponse = 0.84f;
                    material.specularStrength = 0.16f;
                    material.rimStrength = 0.042f;
                    material.specularPower = 64.0f;
                    material.outlineScale = 0.85f;
                    break;
                case ToonMaterialKind::Eye:
                    material.lightResponse = 0.88f;
                    material.specularStrength = 0.65f;
                    material.rimStrength = 0.015f;
                    material.specularPower = 180.0f;
                    material.outlineScale = 0.0f;
                    break;
                case ToonMaterialKind::Metal:
                    material.lightResponse = 0.90f;
                    material.specularStrength = 0.50f;
                    material.rimStrength = 0.038f;
                    material.specularPower = 100.0f;
                    break;
                case ToonMaterialKind::Fabric:
                    material.lightResponse = 0.74f;
                    material.specularStrength = 0.015f;
                    material.rimStrength = 0.026f;
                    material.specularPower = 32.0f;
                    break;
                case ToonMaterialKind::Leather:
                    material.lightResponse = 0.78f;
                    material.specularStrength = 0.10f;
                    material.rimStrength = 0.030f;
                    material.specularPower = 64.0f;
                    break;
                case ToonMaterialKind::Ink:
                    material.specularStrength = 0.0f;
                    material.rimStrength = 0.0f;
                    material.outlineScale = 0.0f;
                    break;
                case ToonMaterialKind::Generic:
                    break;
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
                candidates.push_back(context.texturesDirectory / rawTexturePath);
            }
            // Exporters often preserve an artist's absolute source path. After
            // relocation, the same file may live beside the model or in the
            // adjacent textures directory.
            candidates.push_back(context.modelDirectory / rawTexturePath.filename());
            candidates.push_back(context.texturesDirectory / rawTexturePath.filename());

            for (const auto& candidate : candidates)
            {
                if (std::filesystem::is_regular_file(candidate))
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
                utf8Path(std::filesystem::absolute(path).lexically_normal());

            const auto found = context.textureCache.find(key);
            if (found != context.textureCache.end())
            {
                return found->second;
            }

            auto texture = std::make_shared<GLTexture>(
                GLTexture::loadFromFile(path)
            );

            context.textureCache.emplace(key, texture);

            std::cout << "Loaded texture: " << utf8Path(path) << "\n";

            return texture;
        }

        std::shared_ptr<GLTexture> loadEmbeddedTextureCached(
            const aiTexture& embedded, const std::string& name, LoadContext& context)
        {
            const std::string key = "embedded:" + name;
            const auto found = context.textureCache.find(key);
            if (found != context.textureCache.end()) return found->second;
            if (!embedded.pcData || embedded.mWidth == 0)
                throw std::runtime_error("Invalid embedded texture: " + name);

            GLTexture texture;
            if (embedded.mHeight == 0)
            {
                texture = GLTexture::loadFromMemory(
                    reinterpret_cast<const unsigned char*>(embedded.pcData), embedded.mWidth);
            }
            else
            {
                if (embedded.mWidth > static_cast<unsigned int>(std::numeric_limits<int>::max()) ||
                    embedded.mHeight > static_cast<unsigned int>(std::numeric_limits<int>::max()) ||
                    static_cast<std::size_t>(embedded.mWidth) >
                        std::numeric_limits<std::size_t>::max() / embedded.mHeight / 4)
                    throw std::runtime_error("Embedded texture dimensions are too large: " + name);
                const std::size_t pixelCount = static_cast<std::size_t>(embedded.mWidth) * embedded.mHeight;
                std::vector<unsigned char> rgba(pixelCount * 4);
                for (std::size_t i = 0; i < pixelCount; ++i)
                {
                    rgba[i * 4] = embedded.pcData[i].r;
                    rgba[i * 4 + 1] = embedded.pcData[i].g;
                    rgba[i * 4 + 2] = embedded.pcData[i].b;
                    rgba[i * 4 + 3] = embedded.pcData[i].a;
                }
                texture = GLTexture::loadFromPixels(rgba.data(), static_cast<int>(embedded.mWidth),
                    static_cast<int>(embedded.mHeight), 4);
            }
            auto result = std::make_shared<GLTexture>(std::move(texture));
            context.textureCache.emplace(key, result);
            return result;
        }

        GLMeshMaterial loadMaterial(
            const aiScene* scene,
            const aiMesh* mesh,
            LoadContext& context,
            unsigned int& textureCoordinateChannel
        )
        {
            GLMeshMaterial result{};

            if (mesh->mMaterialIndex >= scene->mNumMaterials)
            {
                return result;
            }

            const aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];
            const std::string materialName = materialMeshName(material, mesh);

            applyMaterialLightingProfile(result, material, mesh);

            aiColor4D baseColor;
            const bool hasBaseColor = material->Get(AI_MATKEY_BASE_COLOR, baseColor) == AI_SUCCESS;
            if (hasBaseColor)
            {
                result.diffuseColor = glm::vec3{baseColor.r, baseColor.g, baseColor.b};
                result.opacity = baseColor.a;
            }
            else
            {
                aiColor3D diffuseColor;
                if (material->Get(AI_MATKEY_COLOR_DIFFUSE, diffuseColor) == AI_SUCCESS)
                    result.diffuseColor = glm::vec3{diffuseColor.r, diffuseColor.g, diffuseColor.b};
                material->Get(AI_MATKEY_OPACITY, result.opacity);
            }
            if (!finite(result.diffuseColor) || !std::isfinite(result.opacity))
                throw std::runtime_error("Material has non-finite color/opacity: " + materialName);
            result.diffuseColor = glm::max(result.diffuseColor, glm::vec3{0.0f});
            result.opacity = std::clamp(result.opacity, 0.0f, 1.0f);

            aiString alphaMode;
            const bool explicitAlphaMode = material->Get(AI_MATKEY_GLTF_ALPHAMODE, alphaMode) == AI_SUCCESS;
            if (explicitAlphaMode)
            {
                const std::string mode = alphaMode.C_Str();
                if (mode == "MASK") result.alphaMode = AlphaMode::Mask;
                else if (mode == "BLEND") result.alphaMode = AlphaMode::Blend;
                else if (mode != "OPAQUE")
                    throw std::runtime_error("Unknown material alpha mode: " + mode);
            }
            else if (result.opacity < 1.0f)
                result.alphaMode = AlphaMode::Blend;
            material->Get(AI_MATKEY_GLTF_ALPHACUTOFF, result.alphaCutoff);
            if (!std::isfinite(result.alphaCutoff))
                throw std::runtime_error("Material has non-finite alpha cutoff: " + materialName);
            result.alphaCutoff = std::clamp(result.alphaCutoff, 0.0f, 1.0f);

            std::filesystem::path texturePath;

            aiString diffuseTextureName;
            aiTextureMapping mapping = aiTextureMapping_UV;
            aiTextureMapMode mapModes[3]{aiTextureMapMode_Wrap, aiTextureMapMode_Wrap, aiTextureMapMode_Wrap};
            aiTextureType textureType = aiTextureType_BASE_COLOR;
            bool hasTexture = material->GetTexture(textureType, 0, &diffuseTextureName, &mapping,
                &textureCoordinateChannel, nullptr, nullptr, mapModes) == AI_SUCCESS;
            if (!hasTexture)
            {
                textureType = aiTextureType_DIFFUSE;
                hasTexture = material->GetTexture(textureType, 0, &diffuseTextureName, &mapping,
                    &textureCoordinateChannel, nullptr, nullptr, mapModes) == AI_SUCCESS;
            }
            if (hasTexture)
            {
                result.wrapS = textureWrap(mapModes[0]);
                result.wrapT = textureWrap(mapModes[1]);
                int minFilter = static_cast<int>(result.minFilter);
                int magFilter = static_cast<int>(result.magFilter);
                material->Get(AI_MATKEY_GLTF_MAPPINGFILTER_MIN(textureType, 0), minFilter);
                material->Get(AI_MATKEY_GLTF_MAPPINGFILTER_MAG(textureType, 0), magFilter);
                if (minFilter != GL_NEAREST && minFilter != GL_LINEAR &&
                    minFilter != GL_NEAREST_MIPMAP_NEAREST && minFilter != GL_LINEAR_MIPMAP_NEAREST &&
                    minFilter != GL_NEAREST_MIPMAP_LINEAR && minFilter != GL_LINEAR_MIPMAP_LINEAR)
                    throw std::runtime_error("Material has an invalid minification filter: " + materialName);
                if (magFilter != GL_NEAREST && magFilter != GL_LINEAR)
                    throw std::runtime_error("Material has an invalid magnification filter: " + materialName);
                result.minFilter = static_cast<GLenum>(minFilter);
                result.magFilter = static_cast<GLenum>(magFilter);
                if (mapping != aiTextureMapping_UV ||
                    textureCoordinateChannel >= AI_MAX_NUMBER_OF_TEXTURECOORDS ||
                    !mesh->HasTextureCoords(textureCoordinateChannel))
                    throw std::runtime_error("Material references unavailable UV coordinates: " + materialName);
                if (const auto* embedded = scene->GetEmbeddedTexture(diffuseTextureName.C_Str()))
                    result.diffuseTexture = loadEmbeddedTextureCached(*embedded, diffuseTextureName.C_Str(), context);
                else
                    texturePath = resolveTexturePath(normalizeTexturePath(diffuseTextureName), context);
            }

            if (!result.diffuseTexture && texturePath.empty())
            {
                if (context.useLegacyMaterialTextureFallback)
                    texturePath = fallbackTexturePathForMaterial(material, mesh, context);
                if (hasTexture && texturePath.empty())
                    std::cerr << "Warning: diffuse texture not found: " << diffuseTextureName.C_Str() << "\n";
            }

            if (!texturePath.empty())
            {
                result.diffuseTexture = loadTextureCached(texturePath, context);
            }

            if (!explicitAlphaMode && result.alphaMode != AlphaMode::Blend &&
                result.diffuseTexture && result.diffuseTexture->hasTransparency())
                result.alphaMode = AlphaMode::Mask;

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

            if (!mesh || !mesh->HasPositions())
                throw std::runtime_error("Model mesh contains no positions");
            const double determinant = glm::determinant(glm::dmat3{globalTransform});
            const glm::mat4 inverseTransform{glm::inverse(glm::dmat4{globalTransform})};
            if (!finite(globalTransform) || !std::isfinite(determinant) || determinant == 0.0f ||
                !finite(inverseTransform))
                throw std::runtime_error("Mesh has a singular or non-finite node transform: " + std::string{mesh->mName.C_Str()});
            unsigned int textureCoordinateChannel = 0;
            auto material = loadMaterial(scene, mesh, context, textureCoordinateChannel);

            vertices.resize(mesh->mNumVertices);

            const glm::mat3 normalTransform =
                glm::mat3(glm::transpose(inverseTransform));
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
                if (!finite(vertex.position))
                    throw std::runtime_error("Model contains a non-finite position");

                if (hasNormals)
                {
                    glm::vec3 localNormal{
                        mesh->mNormals[i].x,
                        mesh->mNormals[i].y,
                        mesh->mNormals[i].z
                    };

                    const glm::vec3 transformedNormal = normalTransform * localNormal;

                    if (!finite(transformedNormal))
                        throw std::runtime_error("Model contains a non-finite normal");

                    const float normalScale = std::max({std::abs(transformedNormal.x),
                        std::abs(transformedNormal.y), std::abs(transformedNormal.z)});
                    if (normalScale > 0.0f)
                    {
                        vertex.normal = glm::normalize(transformedNormal / normalScale);
                    }
                }

                if (mesh->HasTextureCoords(textureCoordinateChannel))
                {
                    vertex.texCoord = glm::vec2{
                        mesh->mTextureCoords[textureCoordinateChannel][i].x,
                        mesh->mTextureCoords[textureCoordinateChannel][i].y
                    };
                    if (!std::isfinite(vertex.texCoord.x) || !std::isfinite(vertex.texCoord.y))
                        throw std::runtime_error("Model contains non-finite texture coordinates");
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

                for (unsigned int i = 0; i < 3; ++i)
                    if (face.mIndices[i] >= vertices.size())
                        throw std::runtime_error("Model face references an invalid vertex");

                indices.push_back(static_cast<std::uint32_t>(face.mIndices[0]));
                // Baking a reflection reverses triangle orientation. Keep the
                // original front faces aligned with inverse-transposed normals.
                indices.push_back(static_cast<std::uint32_t>(face.mIndices[determinant < 0.0f ? 2 : 1]));
                indices.push_back(static_cast<std::uint32_t>(face.mIndices[determinant < 0.0f ? 1 : 2]));
            }

            for (unsigned int boneIndex = 0; boneIndex < mesh->mNumBones; ++boneIndex)
            {
                const aiBone* bone = mesh->mBones[boneIndex];
                const std::string boneName = bone->mName.C_Str();

                std::uint32_t finalBoneIndex = 0;

                std::uint32_t boneNode = NoModelNode;
#ifndef ASSIMP_BUILD_NO_ARMATUREPOPULATE_PROCESS
                if (bone->mNode)
                {
                    const auto node = context.nodeToIndex.find(bone->mNode);
                    if (node != context.nodeToIndex.end()) boneNode = node->second;
                }
#endif
                if (boneNode == NoModelNode)
                {
                    const auto nodes = context.nodeNameToIndices.find(boneName);
                    if (nodes == context.nodeNameToIndices.end() || nodes->second.size() != 1)
                        throw std::runtime_error("Bone has no unique corresponding model node: " + boneName);
                    boneNode = nodes->second.front();
                }
                const glm::mat4 adjustedOffset = toGlm(bone->mOffsetMatrix) * inverseTransform;
                if (!finite(adjustedOffset))
                    throw std::runtime_error("Bone has a non-finite offset matrix: " + boneName);
                auto& candidates = context.boneNameToIndices[boneName];
                const auto existingBone = std::find_if(candidates.begin(), candidates.end(), [&](std::uint32_t index) {
                    const auto& existing = context.model.bones()[index];
                    return existing.nodeIndex == boneNode && equivalent(existing.offsetMatrix, adjustedOffset);
                });
                if (existingBone != candidates.end()) finalBoneIndex = *existingBone;
                else
                {
                    GLBoneInfo boneInfo{};
                    boneInfo.name = boneName;
                    boneInfo.nodeIndex = boneNode;

                    // We bake the node's globalTransform into vertex positions
                    // (see above), but the Assimp offset matrix expects vertices
                    // in the mesh's original local space. Compose it with the
                    // inverse transform so that later skinning,
                    //   boneMatrix * offset' * bakedPos
                    //     == boneMatrix * offset * (G^-1 * G * localPos)
                    //     == boneMatrix * offset * localPos,
                    // reproduces the correct result. One skeleton node may have
                    // multiple palette entries when mesh instances require
                    // different adjusted offsets; names alone are not a key.
                    boneInfo.offsetMatrix = adjustedOffset;

                    finalBoneIndex = context.model.addBone(std::move(boneInfo));
                    candidates.push_back(finalBoneIndex);
                }

                for (unsigned int weightIndex = 0; weightIndex < bone->mNumWeights; ++weightIndex)
                {
                    const aiVertexWeight& weight = bone->mWeights[weightIndex];

                    if (weight.mVertexId >= vertices.size())
                    {
                        throw std::runtime_error("Bone weight references an invalid vertex: " + boneName);
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
                std::move(material)
            };
        }

        void registerNodes(const aiNode* node, std::uint32_t parent, LoadContext& context)
        {
            GLModelNode imported;
            imported.name = node->mName.C_Str();
            imported.parent = parent;
            imported.localTransform = toGlm(node->mTransformation);
            const auto index = context.model.addNode(std::move(imported));
            context.nodeToIndex.emplace(node, index);
            context.nodeNameToIndices[node->mName.C_Str()].push_back(index);
            for (unsigned int i = 0; i < node->mNumChildren; ++i)
                registerNodes(node->mChildren[i], index, context);
        }

        void processNode(const aiScene* scene, const aiNode* node, LoadContext& context)
        {
            const glm::mat4& nodeTransform =
                context.model.nodes()[context.nodeToIndex.at(node)].globalBindTransform;

            for (unsigned int i = 0; i < node->mNumMeshes; ++i)
            {
                const unsigned int meshIndex = node->mMeshes[i];
                if (meshIndex >= scene->mNumMeshes)
                    throw std::runtime_error("Model node references an invalid mesh");
                const aiMesh* mesh = scene->mMeshes[meshIndex];

                if ((mesh->mPrimitiveTypes & aiPrimitiveType_TRIANGLE) == 0 || mesh->mNumFaces == 0)
                    continue;

                context.model.addMesh(
                    processMesh(scene, mesh, nodeTransform, context)
                );
            }

            for (unsigned int i = 0; i < node->mNumChildren; ++i)
            {
                processNode(
                    scene,
                    node->mChildren[i],
                    context
                );
            }
        }

        void copyAnimations(const aiScene& scene, LoadContext& context)
        {
            for (unsigned int a = 0; a < scene.mNumAnimations; ++a)
            {
                const auto& source = *scene.mAnimations[a];
                AnimationClip clip;
                clip.name = source.mName.C_Str();
                clip.durationTicks = source.mDuration;
                if (!std::isfinite(source.mTicksPerSecond) || source.mTicksPerSecond < 0.0)
                    throw std::runtime_error("Invalid animation tick rate: " + clip.name);
                clip.usedDefaultTicksPerSecond = source.mTicksPerSecond == 0.0;
                clip.ticksPerSecond = clip.usedDefaultTicksPerSecond ? 25.0 : source.mTicksPerSecond;
                clip.channels.reserve(source.mNumChannels);
                for (unsigned int c = 0; c < source.mNumChannels; ++c)
                {
                    const auto& input = *source.mChannels[c];
                    const auto found = context.nodeNameToIndices.find(input.mNodeName.C_Str());
                    if (found == context.nodeNameToIndices.end() || found->second.size() != 1)
                        throw std::runtime_error("Animation channel has no unique node: " +
                            std::string{input.mNodeName.C_Str()});
                    AnimationChannel channel;
                    channel.nodeIndex = found->second.front();
                    const auto copyVectors = [](const aiVectorKey* keys, unsigned int count, auto& out) {
                        out.reserve(count);
                        for (unsigned int k = 0; k < count; ++k)
                            out.push_back({keys[k].mTime,
                                {keys[k].mValue.x, keys[k].mValue.y, keys[k].mValue.z}});
                    };
                    copyVectors(input.mPositionKeys, input.mNumPositionKeys, channel.translations);
                    copyVectors(input.mScalingKeys, input.mNumScalingKeys, channel.scales);
                    channel.rotations.reserve(input.mNumRotationKeys);
                    for (unsigned int k = 0; k < input.mNumRotationKeys; ++k)
                    {
                        const auto& q = input.mRotationKeys[k].mValue;
                        channel.rotations.push_back({input.mRotationKeys[k].mTime, {q.w, q.x, q.y, q.z}});
                    }
                    clip.channels.push_back(std::move(channel));
                }
                context.model.addAnimation(std::move(clip));
            }
        }
    }

    GLModel GLModelLoader::loadFromFile(const std::filesystem::path& path, GLModelLoadOptions options)
    {
        if (!std::filesystem::is_regular_file(path))
        {
            throw std::runtime_error("Model file does not exist: " + utf8Path(path));
        }

        Assimp::Importer importer;

        unsigned int flags =
            aiProcess_Triangulate |
            aiProcess_ValidateDataStructure |
            aiProcess_PopulateArmatureData |
            aiProcess_GenSmoothNormals |
            aiProcess_JoinIdenticalVertices |
            aiProcess_LimitBoneWeights |
            aiProcess_FlipUVs |
            aiProcess_GenUVCoords |
            aiProcess_TransformUVCoords |
            aiProcess_FindInvalidData |
            aiProcess_FindDegenerates |
            aiProcess_SortByPType |
            aiProcess_ImproveCacheLocality;

        // GenSmoothNormals alone keeps normals supplied by the asset, including
        // an accidentally faceted export. Force regeneration only on request.
        if (options.rebuildSmoothNormals) flags |= aiProcess_ForceGenNormals;
        importer.SetPropertyFloat(AI_CONFIG_PP_GSN_MAX_SMOOTHING_ANGLE,
            options.rebuildSmoothNormals ? 45.0f : 75.0f);
        importer.SetPropertyInteger(AI_CONFIG_PP_LBW_MAX_WEIGHTS, 4);
        // Asset units are arbitrary: an absolute 1e-6 area cutoff can erase
        // perfectly valid small triangles before model fitting scales them up.
        importer.SetPropertyBool(AI_CONFIG_PP_FD_CHECKAREA, false);
        // Constant UVs are valid (e.g. sampling a single atlas swatch). Assimp's
        // invalid-data pass otherwise deletes the channel and leaves materials
        // pointing at UV data that no longer exists. We validate finite UVs.
        importer.SetPropertyBool(AI_CONFIG_PP_FID_IGNORE_TEXTURECOORDS, true);

        // FindDegenerates turns degenerate triangles into points/lines; ask
        // SortByPType to drop those primitives outright so only real triangles
        // reach the mesh processor.
        importer.SetPropertyInteger(
            AI_CONFIG_PP_SBP_REMOVE,
            aiPrimitiveType_POINT | aiPrimitiveType_LINE
        );

        // Assimp's Windows IO system expects UTF-8 and converts to wide paths;
        // path.string() uses the active code page and breaks Unicode filenames.
        const aiScene* scene = importer.ReadFile(utf8Path(path), flags);

        if (!scene || !scene->mRootNode || (scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE))
        {
            throw std::runtime_error(
                "Assimp failed to load model: " +
                utf8Path(path) +
                "\n" +
                importer.GetErrorString()
            );
        }

        LoadContext context{};
        context.modelDirectory = path.parent_path();
        context.texturesDirectory = path.parent_path().parent_path() / "textures";
        context.useLegacyMaterialTextureFallback = options.useLegacyMaterialTextureFallback;

        registerNodes(scene->mRootNode, NoModelNode, context);
        processNode(scene, scene->mRootNode, context);
        copyAnimations(*scene, context);

        if (context.model.empty())
        {
            throw std::runtime_error("Model contains no meshes: " + utf8Path(path));
        }

        return std::move(context.model);
    }
}
