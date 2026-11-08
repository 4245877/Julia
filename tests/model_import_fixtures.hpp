#pragma once
#include <assimp/Exporter.hpp>
#include <assimp/material.h>
#include <assimp/scene.h>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <stdexcept>
namespace model_fixture
{
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

}
