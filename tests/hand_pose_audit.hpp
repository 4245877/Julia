#pragma once
// Bare-hand skin audit: linear blend skinning identical to model.vert, then
// triangle intersections between hand regions, folds inside one finger,
// local surface compression and clearance between neighbouring fingers.
#include "julia/model/ModelAsset.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>
#include <glm/glm.hpp>

namespace hand_pose_audit
{
    // Regions: 0..4 index, middle, ring, pinky, thumb; 5 palm (hand bone).
    inline constexpr std::array<std::string_view,6> regionNames{"index","middle","ring","pinky","thumb","palm"};
    inline constexpr unsigned Palm=5, NoRegion=255;

    struct PairCount
    {
        unsigned strict=0;   // all three vertices of both triangles dominated by their region
        unsigned majority=0; // region of at least two vertices, non-adjacent triangles
    };
    struct HandMetrics
    {
        // Unordered region pairs a<b, both on this hand.
        std::array<std::array<PairCount,6>,6> pairs{};
        unsigned fingerFolds=0;      // different phalanges of one finger, non-adjacent triangles
        float minAreaRatio=1;        // posed/bind triangle area, hand regions
        unsigned compressed=0;       // triangles below 25 % of the bind area
        std::array<float,4> neighbourClearance{}; // index-middle, middle-ring, ring-pinky, thumb-index (metres)
        unsigned interRegion() const
        {
            unsigned total=0;
            for (unsigned a=0;a<6;++a) for (unsigned b=a+1;b<6;++b) total+=pairs[a][b].majority;
            return total;
        }
        unsigned interRegionStrict() const
        {
            unsigned total=0;
            for (unsigned a=0;a<6;++a) for (unsigned b=a+1;b<6;++b) total+=pairs[a][b].strict;
            return total;
        }
    };

    class Skin
    {
    public:
        explicit Skin(const julia::model::ModelAsset& asset,float metresPerResource=.4f) : scale_(metresPerResource)
        {
            std::vector<int> boneSide(asset.bones().size(),-1), boneRegion(asset.bones().size(),NoRegion), bonePhalanx(asset.bones().size(),0);
            constexpr std::array<std::string_view,5> fingers{"index","middle","ring","pinky","thumb"};
            for (std::size_t i=0;i<asset.bones().size();++i) {
                const std::string_view name=asset.bones()[i].name;
                const int side=name.find(".l_")!=std::string_view::npos ? 0:name.find(".r_")!=std::string_view::npos ? 1:-1;
                if (side<0) continue;
                if (name.starts_with("hand.")) { boneSide[i]=side; boneRegion[i]=Palm; continue; }
                for (unsigned f=0;f<5;++f) for (int k=1;k<=3;++k)
                    if (name.starts_with(std::string{"c_"}+std::string{fingers[f]}+std::to_string(k)+".")) {
                        boneSide[i]=side; boneRegion[i]=static_cast<int>(f); bonePhalanx[i]=k;
                    }
            }
            std::map<std::array<std::int64_t,3>,std::uint32_t> weld;
            for (const auto& mesh:asset.meshes()) {
                if (mesh.material.toonKind!=julia::model::ToonMaterialKind::Skin) continue;
                std::vector<std::uint32_t> remap(mesh.vertices.size(),std::numeric_limits<std::uint32_t>::max());
                std::vector<char> hand(mesh.vertices.size(),0);
                for (std::size_t v=0;v<mesh.vertices.size();++v)
                    for (int k=0;k<4;++k) if (mesh.vertices[v].boneWeights[k]>0 && mesh.vertices[v].boneIds[k]>=0 &&
                        boneRegion[static_cast<std::size_t>(mesh.vertices[v].boneIds[k])]!=NoRegion) hand[v]=1;
                for (std::size_t i=0;i+2<mesh.indices.size();i+=3) {
                    const std::array face{mesh.indices[i],mesh.indices[i+1],mesh.indices[i+2]};
                    if (!hand[face[0]] && !hand[face[1]] && !hand[face[2]]) continue;
                    std::array<std::uint32_t,3> t{};
                    for (int k=0;k<3;++k) {
                        const auto v=face[static_cast<std::size_t>(k)];
                        if (remap[v]==std::numeric_limits<std::uint32_t>::max()) {
                            const auto& x=mesh.vertices[v];
                            const std::array<std::int64_t,3> key{std::llround(x.position.x*1e6),std::llround(x.position.y*1e6),std::llround(x.position.z*1e6)};
                            const auto [it,added]=weld.try_emplace(key,static_cast<std::uint32_t>(points_.size()));
                            if (added) {
                                Vertex vertex; vertex.bind=x.position;
                                float total=0;
                                for (int b=0;b<4;++b) if (x.boneIds[b]>=0 && x.boneWeights[b]>0 && std::isfinite(x.boneWeights[b])) total+=x.boneWeights[b];
                                int strongest=-1; float best=0;
                                for (int b=0;b<4;++b) if (x.boneIds[b]>=0 && x.boneWeights[b]>0 && std::isfinite(x.boneWeights[b])) {
                                    vertex.bones[static_cast<std::size_t>(vertex.count)]=x.boneIds[b];
                                    vertex.weights[static_cast<std::size_t>(vertex.count)]=x.boneWeights[b]/total; ++vertex.count;
                                    if (x.boneWeights[b]>best) { best=x.boneWeights[b]; strongest=x.boneIds[b]; }
                                }
                                if (strongest>=0) {
                                    const auto s=static_cast<std::size_t>(strongest);
                                    vertex.side=boneSide[s]; vertex.region=static_cast<unsigned>(boneRegion[s]); vertex.phalanx=bonePhalanx[s];
                                }
                                vertices_.push_back(vertex); points_.push_back(x.position);
                            }
                            remap[v]=it->second;
                        }
                        t[static_cast<std::size_t>(k)]=remap[v];
                    }
                    if (t[0]==t[1] || t[1]==t[2] || t[0]==t[2]) continue;
                    triangles_.push_back(t);
                }
            }
            // Triangle labels and bind areas.
            for (const auto& t:triangles_) {
                Triangle info;
                const auto& a=vertices_[t[0]]; const auto& b=vertices_[t[1]]; const auto& c=vertices_[t[2]];
                info.side=a.side;
                if (a.region==b.region || a.region==c.region) info.region=a.region; else if (b.region==c.region) info.region=b.region; else info.region=a.region;
                if (a.side!=b.side || a.side!=c.side) info.side=-1;
                info.strict=a.region==b.region && a.region==c.region && a.region!=NoRegion;
                info.phalanx=info.strict && a.phalanx==b.phalanx && a.phalanx==c.phalanx ? a.phalanx:0;
                info.bindArea=area(points_[t[0]],points_[t[1]],points_[t[2]]);
                labels_.push_back(info);
            }
            // One-ring adjacency on welded vertices for fold exclusion.
            neighbours_.resize(vertices_.size());
            for (const auto& t:triangles_) for (int k=0;k<3;++k) for (int m=0;m<3;++m) if (k!=m)
                neighbours_[t[static_cast<std::size_t>(k)]].push_back(t[static_cast<std::size_t>(m)]);
            for (auto& n:neighbours_) { std::sort(n.begin(),n.end()); n.erase(std::unique(n.begin(),n.end()),n.end()); }
            posed_=points_;
        }

        std::size_t vertexCount() const { return vertices_.size(); }
        std::size_t triangleCount() const { return triangles_.size(); }

        // Palette in resource space, as uploaded to model.vert.
        void pose(std::span<const glm::mat4> palette)
        {
            for (std::size_t i=0;i<vertices_.size();++i) {
                const auto& v=vertices_[i];
                if (!v.count) { posed_[i]=v.bind; continue; }
                glm::mat4 m{0};
                for (int k=0;k<v.count;++k) m+=palette[static_cast<std::size_t>(v.bones[static_cast<std::size_t>(k)])]*v.weights[static_cast<std::size_t>(k)];
                posed_[i]=glm::vec3{m*glm::vec4{v.bind,1}};
            }
        }
        std::span<const glm::vec3> posed() const { return posed_; }

        std::array<HandMetrics,2> measure() const
        {
            std::array<HandMetrics,2> result{};
            for (auto& r:result) r.neighbourClearance.fill(std::numeric_limits<float>::infinity());
            // Surface compression.
            for (std::size_t i=0;i<triangles_.size();++i) {
                const auto& l=labels_[i];
                if (l.side<0 || l.region==NoRegion || l.bindArea<=0) continue;
                const auto& t=triangles_[i];
                const float ratio=area(posed_[t[0]],posed_[t[1]],posed_[t[2]])/l.bindArea;
                auto& m=result[static_cast<std::size_t>(l.side)];
                m.minAreaRatio=std::min(m.minAreaRatio,ratio);
                if (ratio<.25f) ++m.compressed;
            }
            // Spatial hash of triangle bounds, 1.4 mm cells.
            const float cell=.0014f/scale_;
            std::unordered_map<std::uint64_t,std::vector<std::uint32_t>> grid;
            const auto key=[&](int x,int y,int z) {
                return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x+1048576))<<42) ^
                    (static_cast<std::uint64_t>(static_cast<std::uint32_t>(y+1048576))<<21) ^ static_cast<std::uint64_t>(static_cast<std::uint32_t>(z+1048576));
            };
            for (std::uint32_t i=0;i<triangles_.size();++i) {
                if (labels_[i].side<0 || labels_[i].region==NoRegion) continue;
                const auto& t=triangles_[i];
                const auto lo=glm::min(posed_[t[0]],glm::min(posed_[t[1]],posed_[t[2]]))/cell;
                const auto hi=glm::max(posed_[t[0]],glm::max(posed_[t[1]],posed_[t[2]]))/cell;
                for (int x=static_cast<int>(std::floor(lo.x));x<=static_cast<int>(std::floor(hi.x));++x)
                    for (int y=static_cast<int>(std::floor(lo.y));y<=static_cast<int>(std::floor(hi.y));++y)
                        for (int z=static_cast<int>(std::floor(lo.z));z<=static_cast<int>(std::floor(hi.z));++z)
                            grid[key(x,y,z)].push_back(i);
            }
            std::vector<std::uint64_t> seen;
            for (const auto& [k,list]:grid) for (std::size_t p=0;p<list.size();++p) for (std::size_t q=p+1;q<list.size();++q) {
                const auto i=std::min(list[p],list[q]), j=std::max(list[p],list[q]);
                const auto& a=labels_[i]; const auto& b=labels_[j];
                if (a.side!=b.side) continue;
                const bool sameRegion=a.region==b.region;
                if (sameRegion && (a.region>=4 || !a.phalanx || !b.phalanx || a.phalanx==b.phalanx)) continue;
                if (shareVertex(i,j) || (sameRegion && nearby(i,j))) continue;
                seen.push_back((static_cast<std::uint64_t>(i)<<32)|j);
            }
            std::sort(seen.begin(),seen.end()); seen.erase(std::unique(seen.begin(),seen.end()),seen.end());
            for (const auto pair:seen) {
                const auto i=static_cast<std::uint32_t>(pair>>32), j=static_cast<std::uint32_t>(pair&0xffffffffu);
                if (!intersects(triangles_[i],triangles_[j])) continue;
                const auto& a=labels_[i]; const auto& b=labels_[j];
                auto& m=result[static_cast<std::size_t>(a.side)];
                if (a.region==b.region) { ++m.fingerFolds; continue; }
                const auto lo=std::min(a.region,b.region), hi=std::max(a.region,b.region);
                ++m.pairs[lo][hi].majority;
                if (a.strict && b.strict) ++m.pairs[lo][hi].strict;
            }
            return result;
        }

        // Clearance between neighbouring fingers beyond the web: phalanges 2-3 of one
        // finger against all phalanges of the other (index-middle, middle-ring, ring-pinky, thumb-index).
        void clearance(std::array<HandMetrics,2>& result) const
        {
            if (byRegion_.empty()) {
                byRegion_.resize(2*5);
                for (std::uint32_t i=0;i<vertices_.size();++i) {
                    const auto& v=vertices_[i];
                    if (v.side>=0 && v.region<5) byRegion_[static_cast<std::size_t>(v.side)*5+v.region].push_back(i);
                }
            }
            constexpr std::array<std::pair<unsigned,unsigned>,4> neighbours{{{0,1},{1,2},{2,3},{4,0}}};
            for (std::size_t side=0;side<2;++side) for (std::size_t n=0;n<neighbours.size();++n) {
                float best=std::numeric_limits<float>::infinity();
                for (int pass=0;pass<2;++pass) {
                    const auto from=pass ? neighbours[n].second:neighbours[n].first, to=pass ? neighbours[n].first:neighbours[n].second;
                    for (const auto i:byRegion_[side*5+from]) {
                        if (vertices_[i].phalanx<2) continue;
                        for (const auto j:byRegion_[side*5+to]) {
                            const auto d=posed_[i]-posed_[j];
                            best=std::min(best,glm::dot(d,d));
                        }
                    }
                }
                result[side].neighbourClearance[n]=std::sqrt(best)*scale_;
            }
        }

    private:
        struct Vertex
        {
            glm::vec3 bind{0};
            std::array<int,4> bones{};
            std::array<float,4> weights{};
            int count=0, side=-1, phalanx=0;
            unsigned region=NoRegion;
        };
        struct Triangle { int side=-1; unsigned region=NoRegion; bool strict=false; int phalanx=0; float bindArea=0; };
        float scale_;
        std::vector<Vertex> vertices_;
        std::vector<glm::vec3> points_, posed_;
        std::vector<std::array<std::uint32_t,3>> triangles_;
        std::vector<Triangle> labels_;
        std::vector<std::vector<std::uint32_t>> neighbours_;
        mutable std::vector<std::vector<std::uint32_t>> byRegion_;

        static float area(const glm::vec3& a,const glm::vec3& b,const glm::vec3& c) { return .5f*glm::length(glm::cross(b-a,c-a)); }
        bool shareVertex(std::uint32_t i,std::uint32_t j) const
        {
            for (auto a:triangles_[i]) for (auto b:triangles_[j]) if (a==b) return true;
            return false;
        }
        // Two-ring proximity on the welded surface: crease neighbours, not folds.
        bool nearby(std::uint32_t i,std::uint32_t j) const
        {
            for (auto a:triangles_[i]) for (auto n:neighbours_[a]) for (auto b:triangles_[j])
                if (n==b || std::binary_search(neighbours_[n].begin(),neighbours_[n].end(),b)) return true;
            return false;
        }
        // Segment strictly crosses the plane and pierces the triangle interior.
        static bool pierces(const glm::vec3& p,const glm::vec3& q,const glm::vec3& a,const glm::vec3& b,const glm::vec3& c)
        {
            const auto n=glm::cross(b-a,c-a);
            const float len=glm::length(n);
            if (len<=0) return false;
            const float dp=glm::dot(p-a,n)/len, dq=glm::dot(q-a,n)/len;
            constexpr float eps=1e-7f;
            if ((dp>eps && dq>eps) || (dp< -eps && dq< -eps) || std::abs(dp-dq)<=eps) return false;
            if (std::abs(dp)<=eps || std::abs(dq)<=eps) return false;
            const auto x=p+(q-p)*(dp/(dp-dq));
            const float u=glm::dot(glm::cross(b-a,x-a),n), v=glm::dot(glm::cross(c-b,x-b),n), w=glm::dot(glm::cross(a-c,x-c),n);
            return u>0 && v>0 && w>0;
        }
        bool intersects(const std::array<std::uint32_t,3>& s,const std::array<std::uint32_t,3>& t) const
        {
            const auto& P=posed_;
            const auto lo1=glm::min(P[s[0]],glm::min(P[s[1]],P[s[2]])), hi1=glm::max(P[s[0]],glm::max(P[s[1]],P[s[2]]));
            const auto lo2=glm::min(P[t[0]],glm::min(P[t[1]],P[t[2]])), hi2=glm::max(P[t[0]],glm::max(P[t[1]],P[t[2]]));
            if (glm::any(glm::lessThan(hi1,lo2)) || glm::any(glm::lessThan(hi2,lo1))) return false;
            for (int k=0;k<3;++k) {
                if (pierces(P[s[static_cast<std::size_t>(k)]],P[s[static_cast<std::size_t>((k+1)%3)]],P[t[0]],P[t[1]],P[t[2]])) return true;
                if (pierces(P[t[static_cast<std::size_t>(k)]],P[t[static_cast<std::size_t>((k+1)%3)]],P[s[0]],P[s[1]],P[s[2]])) return true;
            }
            return false;
        }
    };
}
