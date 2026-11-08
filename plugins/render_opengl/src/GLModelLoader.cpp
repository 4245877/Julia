#include "julia/render_opengl/GLModelLoader.hpp"
#include <stdexcept>
#include <unordered_map>

namespace julia::render_opengl
{
    static_assert(model::sampler::Repeat == GL_REPEAT && model::sampler::ClampToEdge == GL_CLAMP_TO_EDGE &&
        model::sampler::MirroredRepeat == GL_MIRRORED_REPEAT && model::sampler::ClampToBorder == GL_CLAMP_TO_BORDER);
    static_assert(model::sampler::Nearest == GL_NEAREST && model::sampler::Linear == GL_LINEAR &&
        model::sampler::NearestMipmapNearest == GL_NEAREST_MIPMAP_NEAREST &&
        model::sampler::LinearMipmapNearest == GL_LINEAR_MIPMAP_NEAREST &&
        model::sampler::NearestMipmapLinear == GL_NEAREST_MIPMAP_LINEAR &&
        model::sampler::LinearMipmapLinear == GL_LINEAR_MIPMAP_LINEAR);

    GLModel GLModelLoader::upload(std::shared_ptr<const model::ModelAsset> asset)
    {
        if (!asset) throw std::invalid_argument("Cannot upload a null model asset");
        GLModel result(asset);
        std::unordered_map<const model::Image*, std::shared_ptr<GLTexture>> textures;
        for (const auto& mesh : asset->meshes())
        {
            GLMeshMaterial material;
            static_cast<model::MaterialProperties&>(material) = mesh.material;
            material.wrapS = mesh.material.wrapS; material.wrapT = mesh.material.wrapT;
            material.minFilter = mesh.material.minFilter; material.magFilter = mesh.material.magFilter;
            if (const auto& image = mesh.material.diffuseTexture)
            {
                auto& texture = textures[image.get()];
                if (!texture)
                    texture = std::make_shared<GLTexture>(GLTexture::loadFromPixels(
                        image->pixels().data(), image->width(), image->height(), 4));
                material.diffuseTexture = texture;
            }
            result.addMesh(GLMesh{mesh.vertices, mesh.indices, std::move(material)});
        }
        return result;
    }

    GLModel GLModelLoader::loadFromFile(const std::filesystem::path& path, GLModelLoadOptions options)
    {
        return upload(model_import::ModelImporter::loadFromFile(path, options));
    }
}
