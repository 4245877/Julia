#pragma once
#include "julia/embodiment/BodyInstance.hpp"
#include <algorithm>
#include <limits>
#include <string_view>

namespace hand_skin_audit
{
    inline constexpr std::array<std::string_view,6> regions{"index","middle","ring","pinky","thumb","palm"};
    struct Measurement
    {
        std::array<float,regions.size()> vertexGaps;
        bool intersects=false;
    };

    inline bool intersectsBox(const glm::vec3& a,const glm::vec3& b,const glm::vec3& c,const glm::vec3& halfExtent)
    {
        if (glm::any(glm::greaterThan(glm::min(a,glm::min(b,c)),halfExtent)) ||
            glm::any(glm::lessThan(glm::max(a,glm::max(b,c)),-halfExtent))) return false;

        const auto separates=[&](const glm::vec3& axis) {
            const float radius=glm::dot(glm::abs(axis),halfExtent);
            const float p=glm::dot(a,axis),q=glm::dot(b,axis),r=glm::dot(c,axis);
            return std::min({p,q,r})>radius || std::max({p,q,r}) < -radius;
        };
        const std::array edges{b-a,c-b,a-c};
        if (separates(glm::cross(edges[0],edges[1]))) return false;
        for (const auto& edge:edges)
            for (const auto& axis:{glm::vec3{1,0,0},glm::vec3{0,1,0},glm::vec3{0,0,1}})
                if (separates(glm::cross(edge,axis))) return false;
        return true;
    }

    inline bool intersectsSphere(const glm::vec3& a,const glm::vec3& b,const glm::vec3& c,float radius)
    {
        const auto closestBoxPoint=glm::max(glm::max(glm::min(a,glm::min(b,c)),-glm::max(a,glm::max(b,c))),glm::vec3{0});
        if (glm::dot(closestBoxPoint,closestBoxPoint)>radius*radius) return false;

        const auto normal=glm::cross(b-a,c-a);
        const float normalSquared=glm::dot(normal,normal);
        if (normalSquared>0 && glm::dot(glm::cross(b-a,-a),normal)>=0 &&
            glm::dot(glm::cross(c-b,-b),normal)>=0 && glm::dot(glm::cross(a-c,-c),normal)>=0) {
            const float plane=glm::dot(a,normal);
            return plane*plane<=radius*radius*normalSquared;
        }
        const auto crosses=[&](const glm::vec3& from,const glm::vec3& to) {
            const auto edge=to-from;
            const float lengthSquared=glm::dot(edge,edge);
            const float weight=lengthSquared>0 ? std::clamp(-glm::dot(from,edge)/lengthSquared,0.f,1.f):0.f;
            const auto p=from+edge*weight;
            return glm::dot(p,p)<=radius*radius;
        };
        return crosses(a,b) || crosses(b,c) || crosses(c,a);
    }

    class Surface
    {
    public:
        explicit Surface(const julia::model::ModelAsset& asset)
        {
            constexpr std::array<std::string_view,regions.size()> prefixes{
                "c_index","c_middle","c_ring","c_pinky","c_thumb","hand."};
            std::array<unsigned,regions.size()> counts{};
            std::vector<std::size_t> boneRegions;
            for (const auto& bone:asset.bones()) {
                const std::string_view name=bone.name;
                const auto region=std::find_if(prefixes.begin(),prefixes.end(),
                    [&](auto prefix) { return name.starts_with(prefix) && name.find(".l_")!=std::string_view::npos; });
                boneRegions.push_back(static_cast<std::size_t>(region-prefixes.begin()));
            }
            for (const auto& mesh:asset.meshes()) {
                if (mesh.material.toonKind!=julia::model::ToonMaterialKind::Skin) continue;
                std::vector<std::size_t> vertexRegions(mesh.vertices.size(),regions.size());
                for (std::size_t i=0;i<mesh.vertices.size();++i) {
                    const auto& v=mesh.vertices[i];
                    int strongest=0;
                    for (int k=1;k<4;++k) if (v.boneWeights[k]>v.boneWeights[strongest]) strongest=k;
                    if (v.boneIds[strongest]>=0) vertexRegions[i]=boneRegions[v.boneIds[strongest]];
                }
                constexpr auto unused=std::numeric_limits<std::size_t>::max();
                std::vector<std::size_t> remap(mesh.vertices.size(),unused);
                for (std::size_t i=0;i<mesh.indices.size();i+=3) {
                    const std::array face{mesh.indices[i],mesh.indices[i+1],mesh.indices[i+2]};
                    if (std::none_of(face.begin(),face.end(),[&](auto v) { return vertexRegions[v]<regions.size(); })) continue;
                    std::array<std::size_t,3> triangle;
                    for (std::size_t k=0;k<face.size();++k) {
                        const auto v=face[k];
                        if (remap[v]==unused) {
                            remap[v]=vertices_.size();
                            const auto region=vertexRegions[v];
                            vertices_.push_back({&mesh.vertices[v],region});
                            if (region<regions.size()) ++counts[region];
                        }
                        triangle[k]=remap[v];
                    }
                    triangles_.push_back(triangle);
                }
            }
            if (std::ranges::find(counts,0u)!=counts.end()) throw std::runtime_error("Missing indexed skin in a hand region");
            points_.resize(vertices_.size());
        }

        template<class Frame>
        Measurement measure(const julia::embodiment::BodyInstance& body,const Frame& frame)
        {
            using namespace julia;
            using namespace julia::embodiment;
            const auto fromResource=glm::inverse(rigidMatrix(frame.object->worldPose))*
                body.control()->binding().worldFromResource(rigidMatrix({frame.root.position,frame.root.rotation}));
            Measurement result;
            result.vertexGaps.fill(std::numeric_limits<float>::infinity());
            const auto* box=std::get_if<world::Box>(&frame.object->shape);
            const auto halfExtent=box ? glm::vec3{box->halfExtent}:glm::vec3{0};
            const float radius=box ? 0:static_cast<float>(std::get<world::Sphere>(frame.object->shape).radius);
            for (std::size_t k=0;k<vertices_.size();++k) {
                const auto& [source,region]=vertices_[k];
                const glm::vec4 position{source->position,1};
                glm::vec4 posed{0};
                float total=0;
                for (int i=0;i<4;++i) if (source->boneIds[i]>=0) {
                    posed+=(frame.pose.palette[source->boneIds[i]]*position)*source->boneWeights[i];
                    total+=source->boneWeights[i];
                }
                const auto point=glm::vec3{fromResource*(total>0 ? posed/total:position)};
                points_[k]=point;
                if (region==regions.size()) continue;
                const auto d=glm::abs(point)-halfExtent;
                const float gap=box ? glm::length(glm::max(d,glm::vec3{0}))+std::min(0.f,std::max({d.x,d.y,d.z})):
                    glm::length(point)-radius;
                result.vertexGaps[region]=std::min(result.vertexGaps[region],gap);
            }
            result.intersects=std::ranges::any_of(triangles_,[&](const auto& triangle) {
                const auto& [a,b,c]=triangle;
                return box ? intersectsBox(points_[a],points_[b],points_[c],halfExtent):
                    intersectsSphere(points_[a],points_[b],points_[c],radius);
            });
            return result;
        }

    private:
        struct Vertex
        {
            const julia::model::Vertex* source;
            std::size_t region;
        };
        std::vector<Vertex> vertices_;
        std::vector<std::array<std::size_t,3>> triangles_;
        std::vector<glm::vec3> points_;
    };
}
