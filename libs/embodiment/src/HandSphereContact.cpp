#include "HandSphereContact.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace julia::embodiment
{
    namespace
    {
        constexpr int Palm=5, Thumb=4;
        constexpr float PadClearance=.0005f, EngageGap=.00115f, ReleaseGap=.0018f;
        constexpr float MinimumFacing=.65f, MinimumOpposition=.55f;
        constexpr float Infinity=std::numeric_limits<float>::infinity();
        struct Bounds
        {
            glm::vec3 min{Infinity},max{-Infinity};
            void add(glm::vec3 p) { min=glm::min(min,p); max=glm::max(max,p); }
            bool overlaps(const Bounds& b) const
            { return glm::all(glm::lessThanEqual(min,b.max)) && glm::all(glm::lessThanEqual(b.min,max)); }
        };
        float squared(glm::vec3 v) { return glm::dot(v,v); }
        Bounds bounds(const std::array<glm::vec3,3>& p)
        {
            return {glm::min(p[0],glm::min(p[1],p[2])),glm::max(p[0],glm::max(p[1],p[2]))};
        }
        bool segmentPierces(glm::vec3 from,glm::vec3 to,const std::array<glm::vec3,3>& t)
        {
            const auto& [a,b,c]=t;
            auto n=glm::cross(b-a,c-a);
            const float area=glm::length(n);
            if (area==0) return false;
            n/=area;
            const float d=glm::dot(from-a,n), e=glm::dot(to-a,n);
            constexpr float slack=1e-7f;
            if (std::abs(d)<=slack || std::abs(e)<=slack || (d>0)==(e>0)) return false;
            const auto p=from+(to-from)*(d/(d-e));
            return glm::dot(glm::cross(b-a,p-a),n)>0 && glm::dot(glm::cross(c-b,p-b),n)>0 &&
                glm::dot(glm::cross(a-c,p-c),n)>0;
        }
        bool trianglesCross(const std::array<glm::vec3,3>& a,const std::array<glm::vec3,3>& b)
        {
            for (std::size_t k=0;k<3;++k) for (const auto& other:b)
                if (squared(a[k]-other)<1e-12f) return false; // Joined finger web, including UV seams.
            for (std::size_t k=0;k<3;++k)
                if (segmentPierces(a[k],a[(k+1)%3],b) || segmentPierces(b[k],b[(k+1)%3],a)) return true;
            return false;
        }
        template<class Closest>
        void nearestOnTriangle(glm::vec3 center,const std::array<glm::vec3,3>& t,const Bounds& box,Closest& answer)
        {
            if (squared(glm::max(box.min-center,glm::max(center-box.max,glm::vec3{0})))>=answer.squared) return;
            const auto& [a,b,c]=t;
            const auto n=glm::cross(b-a,c-a);
            const float n2=squared(n);
            const auto remember=[&](glm::vec3 p) {
                const float d=squared(p-center);
                if (d<answer.squared) { answer.squared=d; answer.point=p; }
            };
            if (n2>0) {
                const auto p=center-n*(glm::dot(center-a,n)/n2);
                if (glm::dot(glm::cross(b-a,p-a),n)>=0 && glm::dot(glm::cross(c-b,p-b),n)>=0 &&
                    glm::dot(glm::cross(a-c,p-c),n)>=0) { remember(p); return; }
            }
            for (std::size_t k=0;k<3;++k) {
                const auto v=t[k], edge=t[(k+1)%3]-v;
                const float len=squared(edge);
                remember(len>0 ? v+edge*std::clamp(glm::dot(center-v,edge)/len,0.f,1.f):v);
            }
        }
        HandPose sphereFlexionLimit()
        {
            // Local angle limits for the sweep around this sphere. The extra travel
            // gives each digit room to find the surface at the measured IK residual.
            const auto r=[](float deg) { return glm::radians(deg); };
            HandPose pose;
            pose.fingers={FingerAngles{r(20.1f),r(-6.7f),r(46.7f),r(26.7f)}, FingerAngles{r(22.6f),r(-17.3f),r(42.9f),r(24.2f)},
                FingerAngles{r(27.2f),r(-19.1f),r(49.8f),r(25.9f)}, FingerAngles{r(28.7f),r(-8.8f),r(43.3f),r(18.4f)}};
            pose.thumb={r(49.9f),r(-3.3f),r(61),r(4.9f),r(29.3f)};
            return pose;
        }
    }

    OpenHandSurface::OpenHandSurface(std::vector<std::array<glm::vec3,3>> triangles)
    {
        if (triangles.empty()) throw std::invalid_argument("No open left hand skin for sphere approach");
        faces_.reserve(triangles.size());
        for (const auto& t:triangles) {
            const auto box=bounds(t);
            faces_.push_back({t,box.min,box.max});
            for (const auto& p:t) radius_=std::max(radius_,glm::length(p));
        }
    }
    glm::vec3 OpenHandSurface::closestPoint(glm::vec3 center) const
    {
        struct { float squared=Infinity; glm::vec3 point{0}; } best;
        for (const auto& f:faces_) nearestOnTriangle(center,f.points,{f.min,f.max},best);
        return best.point;
    }

    HandSphereContact::HandSphereContact(const BodyRigBinding& binding,const HandLayer& hand)
        : binding_(binding),hand_(hand),pose_(binding.asset().skeleton())
    {
        const auto& asset=binding_.asset();
        const auto& anatomy=hand_.anatomy(0);
        const auto& body=binding_.profile().bodyFromResource;
        const auto wrist=binding_.jointNode(JointId::LeftWrist);
        std::vector<int> region(asset.nodes().size(),-1);
        region[wrist]=Palm;
        for (const auto& j:hand_.joints()) if (j.side==0) {
            region[j.node]=static_cast<int>(j.finger);
            if (j.phalanx!=2) continue;
            const auto f=body*asset.nodes()[j.node].globalBindTransform;
            const auto distal=glm::normalize(glm::vec3{f[1]});
            const auto pad=j.finger==Thumb ? -.70f*anatomy.radial+.71f*anatomy.palmar:
                glm::cross(anatomy.flexion[j.finger],distal);
            padAxes_[j.finger]=glm::inverse(glm::mat3{f})*glm::normalize(pad);
        }
        std::vector<bool> usedBones(asset.boneCount());
        constexpr auto unused=std::numeric_limits<std::size_t>::max();
        for (const auto& mesh:asset.meshes()) {
            if (mesh.material.toonKind!=model::ToonMaterialKind::Skin) continue;
            std::vector<std::size_t> remap(mesh.vertices.size(),unused);
            std::vector<int> regions(mesh.vertices.size(),-1);
            std::vector<bool> pads(mesh.vertices.size());
            for (std::size_t k=0;k<mesh.vertices.size();++k) {
                const auto& v=mesh.vertices[k];
                int slot=0;
                for (int i=1;i<4;++i) if (v.boneWeights[i]>v.boneWeights[slot]) slot=i;
                if (v.boneIds[slot]<0) continue;
                const auto node=asset.bones()[v.boneIds[slot]].nodeIndex;
                const int digit=regions[k]=region[node];
                if (digit<0 || digit>=Palm || hand_.joints()[digit*3+2].node!=node) continue;
                const auto f=body*asset.nodes()[node].globalBindTransform;
                const auto towardTip=glm::vec3{body*glm::vec4{v.position,1}-f[3]};
                const auto pad=glm::normalize(glm::mat3{f}*padAxes_[digit]);
                pads[k]=glm::dot(towardTip,glm::normalize(glm::vec3{f[1]}))>.002f &&
                    glm::dot(glm::normalize(glm::mat3{body}*v.normal),pad)>.28f;
            }
            for (std::size_t k=0;k<mesh.indices.size();k+=3) {
                const std::array ids{mesh.indices[k],mesh.indices[k+1],mesh.indices[k+2]};
                if (std::ranges::all_of(ids,[&](auto i) { return regions[i]<0; })) continue;
                Triangle t{{},-1,-1};
                for (std::size_t a=0;a<3;++a) {
                    const auto i=ids[a], next=ids[(a+1)%3];
                    if (regions[i]==regions[next] && regions[i]>=0) {
                        t.region=regions[i];
                        if (pads[i] && pads[next]) t.pad=t.region;
                    }
                    if (remap[i]==unused) {
                        remap[i]=vertices_.size();
                        vertices_.push_back(&mesh.vertices[i]);
                        for (int slot=0;slot<4;++slot)
                            if (const int b=mesh.vertices[i].boneIds[slot];b>=0) usedBones[b]=true;
                    }
                    t.vertices[a]=remap[i];
                }
                if (t.region>=0) regions_[t.region].push_back(triangles_.size());
                triangles_.push_back(t);
            }
        }
        for (std::size_t b=0;b<usedBones.size();++b) if (usedBones[b]) skinBones_.push_back(b);
        skin_.resize(asset.boneCount());
        points_.resize(vertices_.size());
        for (std::size_t d=0;d<digitVertices_.size();++d) {
            if (std::ranges::none_of(regions_[d],[&](auto i) { return triangles_[i].pad==static_cast<int>(d); }))
                throw std::invalid_argument("Missing left hand skin pad for sphere grasp");
            auto& vertices=digitVertices_[d];
            for (auto i:regions_[d]) {
                const auto& ids=triangles_[i].vertices;
                vertices.insert(vertices.end(),ids.begin(),ids.end());
            }
            std::ranges::sort(vertices);
            vertices.erase(std::unique(vertices.begin(),vertices.end()),vertices.end());
        }
        skin(openHandPose());
        std::vector<std::array<glm::vec3,3>> open;
        open.reserve(triangles_.size());
        for (const auto& t:triangles_)
            open.push_back({points_[t.vertices[0]],points_[t.vertices[1]],points_[t.vertices[2]]});
        openSurface_=std::make_shared<const OpenHandSurface>(std::move(open));
    }

    void HandSphereContact::skin(const HandPose& angles,std::optional<std::size_t> digit)
    {
        hand_.applyPose(pose_.locals,0,angles);
        animation::computeGlobalTransforms(binding_.asset().skeleton(),pose_.locals,pose_.globals);
        skin(pose_.globals,digit);
    }
    void HandSphereContact::skin(std::span<const glm::mat4> globals,std::optional<std::size_t> digit)
    {
        const auto wrist=binding_.jointFrame(JointId::LeftWrist,globals);
        const auto fromResource=glm::inverse(wrist)*binding_.profile().bodyFromResource;
        const auto& bones=binding_.asset().bones();
        for (const auto b:skinBones_) skin_[b]=fromResource*globals[bones[b].nodeIndex]*bones[b].offsetMatrix;
        const auto skinVertex=[&](std::size_t k) {
            const auto& v=*vertices_[k];
            glm::vec4 p{0};
            float sum=0;
            for (int i=0;i<4;++i) if (v.boneIds[i]>=0) {
                p+=(skin_[v.boneIds[i]]*glm::vec4{v.position,1})*v.boneWeights[i];
                sum+=v.boneWeights[i];
            }
            points_[k]=glm::vec3{sum>0 ? p/sum:fromResource*glm::vec4{v.position,1}};
        };
        if (digit) {
            for (auto k:digitVertices_[*digit]) skinVertex(k);
            return;
        }
        for (std::size_t k=0;k<vertices_.size();++k) skinVertex(k);
        for (std::size_t d=0;d<padNormals_.size();++d)
            padNormals_[d]=glm::normalize(glm::mat3{fromResource*globals[hand_.joints()[d*3+2].node]}*padAxes_[d]);
    }
    void HandSphereContact::checkTriangle(const Triangle& t,glm::vec3 center,Closest& nearest) const
    {
        const auto& [a,b,c]=t.vertices;
        const std::array sample{points_[a],points_[b],points_[c]};
        nearestOnTriangle(center,sample,bounds(sample),nearest);
    }
    SphereContacts HandSphereContact::contacts(glm::vec3 center,float radius,bool releasing) const
    {
        std::array<Closest,5> pads;
        pads.fill({Infinity});
        Closest skin{Infinity};
        for (const auto& t:triangles_) {
            checkTriangle(t,center,skin);
            if (t.pad>=0) checkTriangle(t,center,pads[t.pad]);
        }
        SphereContacts result;
        result.skinGap=std::sqrt(skin.squared)-radius;
        glm::vec3 opposing{0};
        unsigned count=0;
        for (std::size_t d=0;d<pads.size();++d) {
            auto& p=result.pads[d];
            p.point=pads[d].point;
            p.gap=std::sqrt(pads[d].squared)-radius;
            const auto direction=glm::normalize(center-p.point);
            p.facing=glm::dot(direction,padNormals_[d]);
            p.supported=p.gap>=0 && p.gap<=(releasing ? ReleaseGap:EngageGap) && p.facing>=MinimumFacing;
            if (d!=Thumb && p.supported) { ++count; opposing+=direction; }
        }
        if (squared(opposing)>0)
            result.thumbOpposition=glm::dot(glm::normalize(opposing),glm::normalize(center-result.pads[Thumb].point));
        result.supporting=result.skinGap>0 && result.pads[Thumb].supported && count>=3 &&
            result.thumbOpposition<=-MinimumOpposition;
        return result;
    }
    SphereContacts HandSphereContact::measure(std::span<const glm::mat4> globals,glm::vec3 center,float radius,bool releasing)
    {
        skin(globals);
        return contacts(center,radius,releasing);
    }

    bool HandSphereContact::fingersClear() const
    {
        struct Face { std::array<glm::vec3,3> p; Bounds box; };
        std::array<std::vector<Face>,6> faces;
        std::array<Bounds,6> limits;
        std::array<float,6> maxWidths{};
        const auto leftEdge=[](const auto& f) { return f.box.min.x; };
        for (std::size_t d=0;d<faces.size();++d) {
            auto& group=faces[d];
            group.reserve(regions_[d].size());
            for (const auto i:regions_[d]) {
                const auto& [a,b,c]=triangles_[i].vertices;
                const std::array p{points_[a],points_[b],points_[c]};
                const auto box=bounds(p);
                group.push_back({p,box});
                limits[d].add(box.min); limits[d].add(box.max);
                maxWidths[d]=std::max(maxWidths[d],box.max.x-box.min.x);
            }
            std::ranges::sort(group,{},leftEdge);
        }
        for (std::size_t a=0;a<faces.size();++a) for (std::size_t b=a+1;b<faces.size();++b) {
            if (!limits[a].overlaps(limits[b])) continue;
            for (const auto& left:faces[a]) {
                if (!left.box.overlaps(limits[b])) continue;
                const auto begin=std::ranges::lower_bound(faces[b],left.box.min.x-maxWidths[b]-1e-7f,{},leftEdge);
                for (auto right=begin;right!=faces[b].end() && right->box.min.x<=left.box.max.x;++right)
                    if (left.box.overlaps(right->box) && trianglesCross(left.p,right->p)) return false;
            }
        }
        return true;
    }
    void HandSphereContact::advanceDigits(Plan& plan)
    {
        const auto open=openHandPose(), limit=sphereFlexionLimit();
        const auto finishDigit=[&](float bend) {
            const auto bent=mixHandPose(open,limit,bend);
            if (plan.digit==Thumb) plan.stroke.closed.thumb=bent.thumb;
            else plan.stroke.closed.fingers[plan.digit]=bent.fingers[plan.digit];
            ++plan.digit;
            plan.trial=0;
        };
        constexpr unsigned samples=16, trialsPerTick=8;
        for (unsigned k=0;k<trialsPerTick && plan.digit<plan.stroke.stopTime.size();++k) {
            const float bend=plan.refinements ? (plan.lower+plan.upper)*.5f:static_cast<float>(plan.trial)/samples;
            const auto bent=mixHandPose(open,limit,bend);
            auto candidate=open;
            if (plan.digit==Thumb) candidate.thumb=bent.thumb;
            else candidate.fingers[plan.digit]=bent.fingers[plan.digit];
            skin(candidate,plan.digit);
            Closest nearest{Infinity};
            for (const auto i:regions_[plan.digit]) checkTriangle(triangles_[i],plan.center,nearest);
            const bool near=std::sqrt(nearest.squared)-plan.radius<=PadClearance;
            if (plan.refinements) {
                if (near) plan.upper=bend;
                else plan.lower=bend;
                if (--plan.refinements==0) finishDigit(plan.lower);
            } else if (near) {
                if (plan.trial==0) throw std::runtime_error("Open finger intersects the sphere clearance");
                plan.lower=static_cast<float>(plan.trial-1)/samples;
                plan.upper=bend;
                plan.refinements=10;
            } else if (plan.trial==samples) finishDigit(1);
            else ++plan.trial;
        }
    }

    std::optional<SphereHandStroke> HandSphereContact::advancePlan(glm::vec3 center,float radius)
    {
        if (!plan_) plan_=Plan{center,radius,{openHandPose(),{.82f,.91f,.99f,.91f,.95f}}};
        auto& plan=*plan_;
        if (glm::length(plan.center-center)>.00001f || plan.radius!=radius)
            throw std::runtime_error("Sphere moved relative to the wrist during contact planning");
        if (plan.digit<plan.stroke.stopTime.size()) {
            advanceDigits(plan);
            return {};
        }
        // Shared skin at the webs is influenced by both neighbouring digits. Admit
        // the combined path (also used in reverse) only after checking those regions.
        constexpr unsigned samples=24;
        if (plan.pathSample==0) {
            skin(plan.stroke.closed);
            if (!contacts(center,radius).supporting)
                throw std::runtime_error("Cannot form opposed sphere pad contacts");
        }
        skin(plan.stroke.sample(static_cast<float>(plan.pathSample)/samples));
        const auto sample=contacts(center,radius,plan.pathSample==0);
        if (sample.skinGap<=.0002f) throw std::runtime_error("Closing hand enters the sphere clearance");
        if (!fingersClear()) throw std::runtime_error("Closing fingers intersect neighbouring skin");
        if (plan.pathSample==0 && sample.supporting) throw std::runtime_error("Open hand already has opposing sphere contacts");
        if (++plan.pathSample<=samples) return {};
        const auto stroke=plan.stroke;
        plan_.reset();
        return stroke;
    }
}
