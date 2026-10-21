#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "julia/render_opengl/GLModelLoader.hpp"
#include "julia/render_opengl/GLShader.hpp"
#include "julia_test.hpp"

#include <assimp/Exporter.hpp>
#include <assimp/material.h>
#include <assimp/scene.h>

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace
{
    using namespace julia::render_opengl;

    bool close(const glm::mat4& a, const glm::mat4& b)
    {
        for (int column = 0; column < 4; ++column)
            for (int row = 0; row < 4; ++row)
                if (std::abs(a[column][row] - b[column][row]) > 0.0001f) return false;
        return true;
    }

    std::array<unsigned char, 4> texturePixel(const GLTexture& texture)
    {
        std::array<unsigned char, 4> pixel{};
        glBindTexture(GL_TEXTURE_2D, texture.id());
        GLint width = 0, height = 0;
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &width);
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &height);
        if (width <= 0 || height <= 0) throw std::runtime_error("Texture has no storage");
        std::vector<unsigned char> pixels(static_cast<std::size_t>(width) * height * 4);
        glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
        std::copy_n(pixels.begin(), 4, pixel.begin());
        glBindTexture(GL_TEXTURE_2D, 0);
        return pixel;
    }

    std::filesystem::path writeInstancedSkin(const std::filesystem::path& directory, bool singular = false,
        bool animated = false)
    {
        // Assbin preserves exact offset matrices, unlike exporters that may
        // reconstruct skeletons. The same aiMesh is referenced by two nodes.
        aiScene scene;
        scene.mRootNode = new aiNode{"root"};
        if (singular) scene.mRootNode->mTransformation.a1 = 0.0f;
        scene.mRootNode->mNumChildren = 3;
        scene.mRootNode->mChildren = new aiNode*[3]{new aiNode{"joint"}, new aiNode{"a"}, new aiNode{"b"}};
        for (unsigned int i = 0; i < 3; ++i)
            scene.mRootNode->mChildren[i]->mParent = scene.mRootNode;
        auto* joint = scene.mRootNode->mChildren[0];
        joint->mTransformation.b4 = 0.3f;
        for (unsigned int i = 1; i < 3; ++i)
        {
            auto* node = scene.mRootNode->mChildren[i];
            node->mTransformation.a4 = i == 1 ? -0.4f : 0.4f;
            node->mNumMeshes = 1;
            node->mMeshes = new unsigned int[1]{0};
        }

        scene.mNumMeshes = 1;
        scene.mMeshes = new aiMesh*[1]{new aiMesh};
        auto& mesh = *scene.mMeshes[0];
        mesh.mName = "instanced_triangle";
        mesh.mPrimitiveTypes = aiPrimitiveType_TRIANGLE;
        mesh.mNumVertices = 3;
        mesh.mVertices = new aiVector3D[3]{{-0.2f, -0.2f, 0.0f}, {0.2f, -0.2f, 0.0f}, {0.0f, 0.2f, 0.0f}};
        mesh.mNormals = new aiVector3D[3]{{0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 1.0f}};
        mesh.mTextureCoords[0] = new aiVector3D[3]{{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.5f, 1.0f, 0.0f}};
        mesh.mNumUVComponents[0] = 2;
        mesh.mNumFaces = 1;
        mesh.mFaces = new aiFace[1];
        mesh.mFaces[0].mNumIndices = 3;
        mesh.mFaces[0].mIndices = new unsigned int[3]{0, 1, 2};
        mesh.mNumBones = 1;
        mesh.mBones = new aiBone*[1]{new aiBone};
        auto& bone = *mesh.mBones[0];
        bone.mName = "joint";
        // jointBind^-1 * instance-A global transform.
        bone.mOffsetMatrix.a4 = -0.4f;
        bone.mOffsetMatrix.b4 = -0.3f;
        bone.mNumWeights = 3;
        bone.mWeights = new aiVertexWeight[3]{{0, 1.0f}, {1, 1.0f}, {2, 1.0f}};

        scene.mNumMaterials = 1;
        scene.mMaterials = new aiMaterial*[1]{new aiMaterial};
        const aiString textureName{"*0"};
        scene.mMaterials[0]->AddProperty(&textureName, AI_MATKEY_TEXTURE_DIFFUSE(0));
        scene.mNumTextures = 1;
        scene.mTextures = new aiTexture*[1]{new aiTexture};
        scene.mTextures[0]->mWidth = 1;
        scene.mTextures[0]->mHeight = 1;
        scene.mTextures[0]->pcData = new aiTexel[1];
        auto& texel = scene.mTextures[0]->pcData[0];
        texel.r = 200; texel.g = 100; texel.b = 50; texel.a = 128;

        if (animated)
        {
            scene.mNumAnimations = 1;
            scene.mAnimations = new aiAnimation*[1]{new aiAnimation};
            auto& clip = *scene.mAnimations[0];
            clip.mName = "owned keys, unspecified tick rate";
            clip.mDuration = 50;
            clip.mTicksPerSecond = 0;
            clip.mNumChannels = 1;
            clip.mChannels = new aiNodeAnim*[1]{new aiNodeAnim};
            auto& channel = *clip.mChannels[0];
            channel.mNodeName = "joint";
            channel.mNumPositionKeys = 2;
            channel.mPositionKeys = new aiVectorKey[2]{{0,{0,.3f,0}}, {50,{1,.3f,0}}};
            channel.mNumRotationKeys = 1;
            channel.mRotationKeys = new aiQuatKey[1]{{0,aiQuaternion{}}};
            channel.mNumScalingKeys = 1;
            channel.mScalingKeys = new aiVectorKey[1]{{0,{1,1,1}}};
        }
        const auto path = directory / (singular ? "singular.assbin" : animated ? "animated_skin.assbin" : "instanced_skin.assbin");
        Assimp::Exporter exporter;
        if (exporter.Export(&scene, "assbin", path.string()) != AI_SUCCESS)
            throw std::runtime_error(exporter.GetErrorString());
        return path;
    }

    std::filesystem::path writeGltf(const std::filesystem::path& directory)
    {
        std::ofstream binary(directory / "fixture.bin", std::ios::binary);
        const float positions[]{-0.5f, -0.5f, 0.0f, 0.5f, -0.5f, 0.0f, 0.0f, 0.5f, 0.0f};
        const float n = 1.0f / std::sqrt(3.0f);
        const float normals[]{n, n, n, n, n, n, n, n, n};
        const float uv0[]{0.1f, 0.2f, 0.1f, 0.2f, 0.1f, 0.2f};
        const float uv1[]{0.7f, 0.8f, 0.7f, 0.8f, 0.7f, 0.8f};
        const std::uint16_t indices[]{0, 1, 2};
        binary.write(reinterpret_cast<const char*>(positions), sizeof(positions));
        binary.write(reinterpret_cast<const char*>(normals), sizeof(normals));
        binary.write(reinterpret_cast<const char*>(uv0), sizeof(uv0));
        binary.write(reinterpret_cast<const char*>(uv1), sizeof(uv1));
        binary.write(reinterpret_cast<const char*>(indices), sizeof(indices));
        binary.close();

        const auto path = directory / "fixture.gltf";
        std::ofstream json(path);
        json << R"({
"asset":{"version":"2.0"},
"buffers":[{"uri":"fixture.bin","byteLength":126}],
"bufferViews":[
 {"buffer":0,"byteOffset":0,"byteLength":36,"target":34962},
 {"buffer":0,"byteOffset":36,"byteLength":36,"target":34962},
 {"buffer":0,"byteOffset":72,"byteLength":24,"target":34962},
 {"buffer":0,"byteOffset":96,"byteLength":24,"target":34962},
 {"buffer":0,"byteOffset":120,"byteLength":6,"target":34963}],
"accessors":[
 {"bufferView":0,"componentType":5126,"count":3,"type":"VEC3","min":[-0.5,-0.5,0],"max":[0.5,0.5,0]},
 {"bufferView":1,"componentType":5126,"count":3,"type":"VEC3"},
 {"bufferView":2,"componentType":5126,"count":3,"type":"VEC2"},
 {"bufferView":3,"componentType":5126,"count":3,"type":"VEC2"},
 {"bufferView":4,"componentType":5123,"count":3,"type":"SCALAR"}],
"images":[{"uri":"data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAIAAAABCAYAAAD0In+KAAAAEUlEQVR4nGM4kWLUwCUi9x8ADlADGpXP8icAAAAASUVORK5CYII="}],
"samplers":[
 {"wrapS":33071,"wrapT":33648,"minFilter":9728,"magFilter":9728},
 {"wrapS":10497,"wrapT":33071,"minFilter":9986,"magFilter":9729}],
"textures":[{"source":0,"sampler":0},{"source":0,"sampler":1}],
"materials":[
 {"pbrMetallicRoughness":{"baseColorFactor":[0.25,0.5,0.75,0.4],"baseColorTexture":{"index":0,"texCoord":1}},"alphaMode":"OPAQUE"},
 {"pbrMetallicRoughness":{"baseColorFactor":[0.25,0.5,0.75,0.4],"baseColorTexture":{"index":0,"texCoord":1}},"alphaMode":"MASK","alphaCutoff":0.35},
 {"pbrMetallicRoughness":{"baseColorFactor":[0.25,0.5,0.75,0.4],"baseColorTexture":{"index":1,"texCoord":1}},"alphaMode":"BLEND"}],
"meshes":[{"primitives":[
 {"attributes":{"POSITION":0,"NORMAL":1,"TEXCOORD_0":2,"TEXCOORD_1":3},"indices":4,"material":0},
 {"attributes":{"POSITION":0,"NORMAL":1,"TEXCOORD_0":2,"TEXCOORD_1":3},"indices":4,"material":1},
 {"attributes":{"POSITION":0,"NORMAL":1,"TEXCOORD_0":2,"TEXCOORD_1":3},"indices":4,"material":2}]}],
"nodes":[{"name":"mirrored_nonuniform","mesh":0,"scale":[-1,1.5,0.5]}],
"scenes":[{"nodes":[0]}],"scene":0
})";
        return path;
    }

    std::filesystem::path writeLegacyFallbackFixture(const std::filesystem::path& directory)
    {
        const auto models = directory / "legacy-models";
        const auto textures = directory / "textures";
        std::filesystem::create_directories(models);
        std::filesystem::create_directories(textures);
        // A valid 1x1 RGBA PNG. Its existence does not imply that unrelated UVs
        // or similarly named materials were authored for this atlas.
        constexpr unsigned char png[]{137,80,78,71,13,10,26,10,0,0,0,13,73,72,68,82,
            0,0,0,1,0,0,0,1,8,6,0,0,0,31,21,196,137,0,0,0,13,73,68,65,84,120,156,
            99,56,145,98,212,0,0,5,53,1,223,115,98,227,219,0,0,0,0,73,69,78,68,174,66,96,130};
        for (const char* name : {"body_color.png", "hair_color.png"})
        {
            std::ofstream image(textures / name, std::ios::binary);
            image.write(reinterpret_cast<const char*>(png), sizeof(png));
        }
        std::ofstream materials(models / "untextured.mtl");
        materials << "newmtl body\nKd 0.5 0.6 0.7\nnewmtl hair\nKd 0.2 0.3 0.4\n";
        materials.close();
        const auto path = models / "untextured.obj";
        std::ofstream model(path);
        model << "mtllib untextured.mtl\nv -0.5 -0.5 0\nv 0.5 -0.5 0\nv 0 0.5 0\n"
              << "vt 0 0\nvt 1 0\nvt 0.5 1\no body\nusemtl body\nf 1/1 2/2 3/3\n"
              << "o hair\nusemtl hair\nf 1/1 2/2 3/3\n";
        return path;
    }

    std::array<unsigned char, 3> drawPixel(const GLMesh& mesh, int mode, int x = 32)
    {
        GLShader shader;
        shader.createFromSource(R"(#version 330 core
layout(location=0) in vec3 aPosition;
layout(location=1) in vec3 aNormal;
layout(location=2) in vec2 aTexCoord;
out vec3 normal;
out vec2 uv;
void main(){gl_Position=vec4(aPosition,1);normal=aNormal;uv=aTexCoord;}
)", R"(#version 330 core
in vec3 normal;
in vec2 uv;
uniform int mode;
uniform sampler2D uDiffuseTexture;
out vec4 color;
void main(){color=vec4(mode==0?vec3(1):mode==1?normal*.5+.5:mode==2?vec3(uv,0):texture(uDiffuseTexture,vec2(1.25,.5)).rgb,1);}
)");
        shader.bind();
        shader.setInt("mode", mode);
        glViewport(0, 0, 64, 64);
        glDisable(GL_FRAMEBUFFER_SRGB);
        glDisable(GL_DITHER);
        glDisable(GL_BLEND);
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_CULL_FACE);
        glFrontFace(GL_CCW);
        glCullFace(GL_BACK);
        glClearColor(0, 0, 0, 1);
        glClear(GL_COLOR_BUFFER_BIT);
        mesh.draw(shader);
        std::array<unsigned char, 3> pixel{};
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(x, 32, 1, 1, GL_RGB, GL_UNSIGNED_BYTE, pixel.data());
        JULIA_CHECK(glGetError() == GL_NO_ERROR);
        return pixel;
    }

    void testNormalRepair(const std::filesystem::path& directory)
    {
        // A shallow fold and a right-angle crease, both with deliberately flat
        // authored normals and separate UVs at the shared geometric edge.
        for (const float angle : {20.0f, 90.0f})
        {
            const float s = std::sin(glm::radians(angle));
            const float c = std::cos(glm::radians(angle));
            const auto path = directory / (angle < 45.0f ? "fold.obj" : "crease.obj");
            std::ofstream file(path);
            file << "v -0.8 0 0\nv 0 -0.8 0\nv 0 0.8 0\nv "
                 << 0.8f * c << " 0 " << -0.8f * s << '\n'
                 << "vt 0 0\nvt 0.4 0\nvt 0.4 1\nvt 0.6 0\nvt 1 0\nvt 0.6 1\n"
                 << "vn 0 0 1\nvn " << s << " 0 " << c << '\n'
                 << "f 1/1/1 2/2/1 3/3/1\nf 2/4/2 4/5/2 3/6/2\n";
            file.close();

            const auto original = GLModelLoader::loadFromFile(path);
            const auto repaired = GLModelLoader::loadFromFile(path,
                {.rebuildSmoothNormals = true});
            JULIA_CHECK(original.meshCount() == 1 && repaired.meshCount() == 1);
            const auto& a = original.meshes().at(0);
            const auto& b = repaired.meshes().at(0);
            JULIA_CHECK(a.indexCount() == 6 && b.indexCount() == 6);
            JULIA_CHECK(original.bounds().min == repaired.bounds().min);
            JULIA_CHECK(original.bounds().max == repaired.bounds().max);
            JULIA_CHECK(drawPixel(a, 0, 16) == drawPixel(b, 0, 16));
            JULIA_CHECK(drawPixel(a, 2, 16) == drawPixel(b, 2, 16));
            const auto flat = drawPixel(a, 1, 16);
            const auto smooth = drawPixel(b, 1, 16);
            JULIA_CHECK(std::abs(static_cast<int>(flat[0]) - 128) <= 1);
            if (angle < 45.0f)
            {
                JULIA_CHECK(smooth[0] > flat[0] + 5);
                JULIA_CHECK(smooth[2] > 245);
            }
            else
                JULIA_CHECK(flat == smooth);
        }
    }

    void testToonMaterialProfiles(const std::filesystem::path& directory)
    {
        struct ProfileCase
        {
            const char* mesh;
            const char* material;
            ToonMaterialKind expected;
        };
        const ProfileCase cases[]{
            {"eyes_mesh", "body_material", ToonMaterialKind::Eye},
            {"skin_mesh", "body_material", ToonMaterialKind::Skin},
            {"sword_mesh", "body_material", ToonMaterialKind::Metal},
            {"shieldBase_mesh", "body_material", ToonMaterialKind::Metal},
            {"boots_mesh", "body_material", ToonMaterialKind::Leather},
            {"underwear_tex_underwear_0", "tex_underwear", ToonMaterialKind::Fabric},
            {"eyelashes_tex_eyelashes_0", "tex_eyelashes", ToonMaterialKind::Ink},
            {"eyebrows_mesh", "body_material", ToonMaterialKind::Ink},
            {"hair_band_tex_band_0", "tex_band", ToonMaterialKind::Fabric},
            {"hair1_mesh", "body_material", ToonMaterialKind::Hair},
            {"earInsides_mesh", "body_material", ToonMaterialKind::Skin},
            {"NurbsPath.001_tex_hair3_0", "tex_hair3", ToonMaterialKind::Hair},
            {"arbitrary_mesh", "eyes_material", ToonMaterialKind::Eye},
            {"clear_mesh", "default_material", ToonMaterialKind::Generic},
            {"teapot_mesh", "pearl_material", ToonMaterialKind::Generic},
            {"skinned_mesh", "default_material", ToonMaterialKind::Generic},
            {"body_mesh", "body_material", ToonMaterialKind::Fabric},
            {"facialHair_mesh", "body_material", ToonMaterialKind::Hair}};
        std::ofstream materials(directory / "toon_profiles.mtl");
        std::ofstream model(directory / "toon_profiles.obj");
        model << "mtllib toon_profiles.mtl\nv -0.5 -0.5 0\nv 0.5 -0.5 0\nv 0 0.5 0\n"
              << "vn 0 0 1\n";
        // Deliberately share one atlas material across skin, eyes, and clothing,
        // as in Raphtalia1. Mesh semantics must survive that shared assignment.
        std::vector<std::string> writtenMaterials;
        for (const auto& entry : cases)
        {
            if (std::find(writtenMaterials.begin(), writtenMaterials.end(), entry.material) == writtenMaterials.end())
            {
                materials << "newmtl " << entry.material << "\nKd 0.5 0.6 0.7\n";
                writtenMaterials.emplace_back(entry.material);
            }
            model << "o " << entry.mesh << "\nusemtl " << entry.material << "\nf 1//1 2//1 3//1\n";
        }
        model.close();
        materials.close();
        const auto imported = GLModelLoader::loadFromFile(directory / "toon_profiles.obj");
        JULIA_CHECK(imported.meshCount() == std::size(cases));
        for (std::size_t i = 0; i < std::min(imported.meshCount(), std::size(cases)); ++i)
        {
            const auto& material = imported.meshes()[i].material();
            if (material.toonKind != cases[i].expected)
                std::cerr << "Incorrect toon profile for " << cases[i].mesh << '\n';
            JULIA_CHECK(material.toonKind == cases[i].expected);
            JULIA_CHECK(material.diffuseColor == glm::vec3(0.5f, 0.6f, 0.7f));
            if (material.toonKind == ToonMaterialKind::Eye)
            {
                JULIA_CHECK(material.specularStrength >= 0.5f);
                JULIA_CHECK(material.specularPower >= 100.0f);
                JULIA_CHECK(material.outlineScale == 0.0f);
            }
            if (material.toonKind == ToonMaterialKind::Ink)
            {
                JULIA_CHECK(material.specularStrength == 0.0f);
                JULIA_CHECK(material.outlineScale == 0.0f);
            }
        }
    }

    void runTests(const std::filesystem::path& directory)
    {
        std::filesystem::create_directories(directory);
        testNormalRepair(directory);
        testToonMaterialProfiles(directory);
        // Both fixture scene and loader importer are destroyed before playback.
        const auto animated = GLModelLoader::loadFromFile(writeInstancedSkin(directory, false, true));
        JULIA_CHECK(animated.animations().size() == 1);
        if (animated.animations().size() == 1)
        {
            const auto& clip = animated.animations()[0];
            JULIA_CHECK(clip.name == "owned keys, unspecified tick rate");
            JULIA_CHECK(clip.usedDefaultTicksPerSecond && clip.ticksPerSecond == 25);
            JULIA_CHECK(clip.durationSeconds() == 2);
            AnimationPlayer player(animated);
            player.play(); player.update(1);
            const auto node = clip.channels[0].nodeIndex;
            JULIA_CHECK(std::abs(player.localTransforms()[node][3].x - .5f) < 1e-5f);
            JULIA_CHECK(std::abs(player.localTransforms()[node][3].y - .3f) < 1e-5f);
        }
        const auto skin = GLModelLoader::loadFromFile(writeInstancedSkin(directory));
        JULIA_CHECK(skin.meshCount() == 2);
        JULIA_CHECK(skin.boneCount() == 2);
        if (skin.boneCount() == 2)
        {
            JULIA_CHECK(skin.bones()[0].name == skin.bones()[1].name);
            JULIA_CHECK(skin.bones()[0].nodeIndex == skin.bones()[1].nodeIndex);
            const auto palette = skin.skinningMatrices();
            JULIA_CHECK(close(palette[0], glm::mat4{1.0f}));
            JULIA_CHECK(close(palette[1], glm::translate(glm::mat4{1.0f}, glm::vec3{-0.8f, 0.0f, 0.0f})));
        }
        JULIA_CHECK(std::abs(skin.bounds().min.x + 0.6f) < 0.0001f);
        JULIA_CHECK(std::abs(skin.bounds().max.x - 0.6f) < 0.0001f);
        if (!skin.meshes().empty())
        {
            const auto& material = skin.meshes()[0].material();
            JULIA_CHECK(material.alphaMode == AlphaMode::Mask);
            JULIA_CHECK(material.diffuseTexture && material.diffuseTexture->hasTransparency());
            if (material.diffuseTexture)
                JULIA_CHECK((texturePixel(*material.diffuseTexture) == std::array<unsigned char, 4>{200, 100, 50, 128}));
        }

        bool singularRejected = false;
        try { (void)GLModelLoader::loadFromFile(writeInstancedSkin(directory, true)); }
        catch (const std::exception&) { singularRejected = true; }
        JULIA_CHECK(singularRejected);

        const auto tinyPath = directory / "tiny.obj";
        {
            std::ofstream tiny(tinyPath);
            tiny << "v 0 0 0\nv 0.0001 0 0\nv 0 0.0001 0\nf 1 2 3\n";
        }
        const auto tiny = GLModelLoader::loadFromFile(tinyPath);
        JULIA_CHECK(tiny.meshCount() == 1);
        const auto fit = tiny.fitToHeightTransform(2.2f);
        const auto min = glm::vec3{fit * glm::vec4{tiny.bounds().min, 1.0f}};
        const auto max = glm::vec3{fit * glm::vec4{tiny.bounds().max, 1.0f}};
        JULIA_CHECK(std::abs(max.y - min.y - 2.2f) < 0.0001f);

        const auto gltf = GLModelLoader::loadFromFile(writeGltf(directory));
        JULIA_CHECK(gltf.meshCount() == 3);
        if (gltf.meshCount() == 3)
        {
            JULIA_CHECK(gltf.meshes()[0].material().alphaMode == AlphaMode::Opaque);
            JULIA_CHECK(gltf.meshes()[1].material().alphaMode == AlphaMode::Mask);
            JULIA_CHECK(gltf.meshes()[2].material().alphaMode == AlphaMode::Blend);
            JULIA_CHECK(std::abs(gltf.meshes()[1].material().alphaCutoff - 0.35f) < 0.0001f);
            for (const auto& mesh : gltf.meshes())
            {
                const auto& material = mesh.material();
                JULIA_CHECK(material.diffuseColor == glm::vec3(0.25f, 0.5f, 0.75f));
                JULIA_CHECK(std::abs(material.opacity - 0.4f) < 0.0001f);
                JULIA_CHECK(material.diffuseTexture && material.diffuseTexture->hasTransparency());
                if (material.diffuseTexture)
                    JULIA_CHECK((texturePixel(*material.diffuseTexture) == std::array<unsigned char, 4>{200, 100, 50, 128}));
            }
            JULIA_CHECK(gltf.meshes()[0].material().diffuseTexture == gltf.meshes()[2].material().diffuseTexture);
            const auto& firstSampler = gltf.meshes()[0].material();
            const auto& secondSampler = gltf.meshes()[2].material();
            JULIA_CHECK(firstSampler.wrapS == GL_CLAMP_TO_EDGE);
            JULIA_CHECK(firstSampler.wrapT == GL_MIRRORED_REPEAT);
            JULIA_CHECK(firstSampler.minFilter == GL_NEAREST);
            JULIA_CHECK(firstSampler.magFilter == GL_NEAREST);
            JULIA_CHECK(secondSampler.wrapS == GL_REPEAT);
            JULIA_CHECK(secondSampler.wrapT == GL_CLAMP_TO_EDGE);
            JULIA_CHECK(secondSampler.minFilter == GL_NEAREST_MIPMAP_LINEAR);
            JULIA_CHECK(secondSampler.magFilter == GL_LINEAR);
            // Two glTF texture entries share image storage but retain distinct
            // sampling state. U=1.25 clamps to the dark second pixel in the
            // first material and repeats to the bright first pixel in the last.
            const auto clampPixel = drawPixel(gltf.meshes()[0], 3);
            const auto repeatPixel = drawPixel(gltf.meshes()[2], 3);
            JULIA_CHECK(static_cast<int>(repeatPixel[0]) - clampPixel[0] > 100);
            // A negative node determinant must not make the front face vanish.
            JULIA_CHECK((drawPixel(gltf.meshes()[0], 0) == std::array<unsigned char, 3>{255, 255, 255}));
            const auto normalPixel = drawPixel(gltf.meshes()[0], 1);
            const auto expectedNormal = glm::normalize(glm::vec3{-1.0f, 1.0f / 1.5f, 2.0f});
            for (int channel = 0; channel < 3; ++channel)
                JULIA_CHECK(std::abs(normalPixel[channel] - (expectedNormal[channel] * 0.5f + 0.5f) * 255.0f) <= 1.0f);
            const auto uvPixel = drawPixel(gltf.meshes()[0], 2);
            JULIA_CHECK(std::abs(static_cast<int>(uvPixel[0]) - 179) <= 1);
            // glTF's top-left UV convention is converted once by Assimp and
            // once by FlipUVs, yielding the original glTF V for top-down images.
            JULIA_CHECK(std::abs(static_cast<int>(uvPixel[1]) - 204) <= 1);
        }

        const auto unicodeDirectory = directory / std::filesystem::path{u8"\u0456\u043c\u043f\u043e\u0440\u0442_\u0442\u0435\u0441\u0442"};
        std::filesystem::create_directories(unicodeDirectory);
        const auto unicodeModel = GLModelLoader::loadFromFile(writeGltf(unicodeDirectory));
        JULIA_CHECK(unicodeModel.meshCount() == 3);

        const auto legacyPath = writeLegacyFallbackFixture(directory);
        const auto untextured = GLModelLoader::loadFromFile(legacyPath);
        JULIA_CHECK(untextured.meshCount() == 2);
        for (const auto& mesh : untextured.meshes())
            JULIA_CHECK(!mesh.material().diffuseTexture);
        const auto legacy = GLModelLoader::loadFromFile(legacyPath,
            GLModelLoadOptions{.useLegacyMaterialTextureFallback = true});
        JULIA_CHECK(legacy.meshCount() == 2);
        for (const auto& mesh : legacy.meshes())
            JULIA_CHECK(mesh.material().diffuseTexture && mesh.material().diffuseTexture->valid());
    }
}

int main(int argc, char** argv)
{
    if (!glfwInit()) return 1;
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    auto* window = glfwCreateWindow(64, 64, "Model importer verification", nullptr, nullptr);
    if (!window) { glfwTerminate(); return 1; }
    glfwMakeContextCurrent(window);
    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)))
    { glfwDestroyWindow(window); glfwTerminate(); return 1; }
    try { runTests(argc > 1 ? std::filesystem::path{argv[1]} : std::filesystem::path{"model-import-fixtures"}); }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; JULIA_CHECK(false); }
    glfwDestroyWindow(window);
    glfwTerminate();
    return JULIA_TEST_RESULT();
}
