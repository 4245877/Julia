#include "julia/embodiment/BodyProfile.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <bit>
#include <cmath>
#include <set>
#include <stdexcept>

namespace julia::embodiment
{
    namespace
    {
        bool finite(const glm::mat4& m)
        {
            for (int c=0;c<4;++c) for (int r=0;r<4;++r)
                if (!std::isfinite(m[c][r])) return false;
            return true;
        }
        bool rigid(const glm::mat4& m, bool similarity = false)
        {
            if (!finite(m) || glm::length(m[0]) < 1e-8f ||
                glm::length(glm::vec4{m[0][3],m[1][3],m[2][3],m[3][3]-1}) > 1e-5f) return false;
            const glm::mat3 a{m};
            const float s = similarity ? glm::length(a[0]) : 1;
            if (!std::isfinite(s) || s<1e-8f) return false;
            const glm::mat3 r = a / s, gram = glm::transpose(r)*r;
            if (std::abs(glm::determinant(r)-1) > 1e-4f) return false;
            for (int c=0;c<3;++c) for (int row=0;row<3;++row)
                if (std::abs(gram[c][row]-(c==row ? 1.0f:0.0f)) > 1e-4f) return false;
            return true;
        }
        std::uint32_t resolve(const NodeLink& link, const model::ModelAsset& asset,
            std::vector<BindingIssue>& issues)
        {
            const auto& nodes = asset.nodes();
            std::uint32_t result = animation::NoNode;
            std::size_t count = 0;
            for (std::uint32_t i=0;i<nodes.size();++i)
                if (nodes[i].name==link.name) { result=i; ++count; }
            if (count==0) issues.push_back({BindingError::MissingNode,"Missing exact node: " + link.name});
            else if (count>1) issues.push_back({BindingError::AmbiguousNode,"Ambiguous exact node: " + link.name});
            else
            {
                const auto p=nodes[result].parent;
                if ((p==animation::NoNode ? std::string{} : nodes[p].name)!=link.parentName)
                    issues.push_back({BindingError::WrongParent,"Wrong direct parent for: " + link.name});
                if (!finite(glm::inverse(nodes[result].globalBindTransform)))
                    issues.push_back({BindingError::InvalidProfile,"Singular bind frame: " + link.name});
            }
            return result;
        }
        glm::mat4 rotation(const JointDefinition& j, const glm::vec3& q)
        {
            glm::mat4 r{1};
            for (std::size_t k=0;k<j.coordinates.size();++k)
                r=glm::rotate(r,q[static_cast<int>(k)],j.coordinates[k].axis);
            return r;
        }
    }

    std::uint64_t bodyRigFingerprint(const model::ModelAsset& asset)
    {
        std::uint64_t h=14695981039346656037ull;
        const auto byte=[&](std::uint8_t b) { h^=b; h*=1099511628211ull; };
        const auto word=[&](std::uint32_t w) { for (int i=0;i<4;++i) byte(static_cast<std::uint8_t>(w>>(8*i))); };
        const auto str=[&](const std::string& s) { word(static_cast<std::uint32_t>(s.size())); for (unsigned char c:s) byte(c); };
        const auto matrix=[&](const glm::mat4& m) { for (int c=0;c<4;++c) for (int r=0;r<4;++r) word(std::bit_cast<std::uint32_t>(m[c][r])); };
        word(static_cast<std::uint32_t>(asset.nodes().size()));
        for (const auto& n:asset.nodes()) { str(n.name); word(n.parent); matrix(n.localTransform); }
        word(static_cast<std::uint32_t>(asset.bones().size()));
        for (const auto& b:asset.bones()) { str(b.name); word(b.nodeIndex); matrix(b.offsetMatrix); }
        return h;
    }

    std::vector<BindingIssue> validateBodyProfile(const BodyProfile& p, const model::ModelAsset& asset)
    {
        std::vector<BindingIssue> issues;
        const auto invalid=[&](const std::string& m) { issues.push_back({BindingError::InvalidProfile,m}); };
        if (p.name.empty() || p.joints.empty() || !rigid(p.bodyFromResource,true)) invalid("Invalid profile name, joints or body/resource similarity");
        std::set<JointId> ids;
        std::set<SegmentId> segments;
        std::set<std::string> driven;
        for (const auto& j:p.joints)
        {
            if (!ids.insert(j.id).second || !segments.insert(j.segment).second || !driven.insert(j.node.name).second)
                invalid("Duplicate joint, segment or driven node: " + j.name);
            if (j.name.empty() || !rigid(j.frameInBodyBind) || j.coordinates.empty() || j.coordinates.size()>3)
                invalid("Invalid joint frame/coordinates: " + j.name);
            const auto node=resolve(j.node,asset,issues), end=resolve(j.segmentEnd,asset,issues);
            if (node!=animation::NoNode && end!=animation::NoNode)
            {
                auto ancestor=end;
                while (ancestor!=animation::NoNode && ancestor!=node) ancestor=asset.nodes()[ancestor].parent;
                if (ancestor!=node || end==node) invalid("Segment end is not a descendant: " + j.name);
                const auto origin=p.bodyFromResource*asset.nodes()[node].globalBindTransform*glm::vec4{0,0,0,1};
                if (glm::length(glm::vec3{origin-j.frameInBodyBind[3]})>2e-5f) invalid("Joint pivot differs from calibrated node: " + j.name);
            }
            std::set<std::string> names;
            for (const auto& c:j.coordinates)
            {
                if (c.name.empty() || c.positiveDirection.empty() || !names.insert(c.name).second ||
                    !std::isfinite(glm::length(c.axis)) || std::abs(glm::length(c.axis)-1)>1e-4f ||
                    !std::isfinite(c.lower) || !std::isfinite(c.upper) || c.lower>0 || c.upper<0 || c.lower>=c.upper)
                    invalid("Invalid coordinate: " + j.name + "/" + c.name);
            }
        }
        for (const auto& f:p.followers)
        {
            resolve(f.node,asset,issues);
            const auto j=std::find_if(p.joints.begin(),p.joints.end(),[&](const auto& x){return x.id==f.joint;});
            if (j==p.joints.end() || f.node.parentName!=j->node.name || !driven.insert(f.node.name).second ||
                !std::isfinite(glm::length(f.fraction)) || glm::any(glm::lessThan(f.fraction,glm::vec3{0})) ||
                glm::any(glm::greaterThan(f.fraction,glm::vec3{1}))) invalid("Invalid twist follower: " + f.node.name);
        }
        std::set<EffectorId> effectorIds;
        for (const auto& e:p.effectors)
        {
            resolve(e.node,asset,issues);
            if (!effectorIds.insert(e.id).second || e.name.empty() || !rigid(e.frameInBodyBind)) invalid("Invalid effector: " + e.name);
        }
        if (p.rigFingerprint==0 || p.rigFingerprint!=bodyRigFingerprint(asset))
            issues.push_back({BindingError::IncompatibleRig,"Rig fingerprint differs from calibrated resource: " + p.name});
        return issues;
    }

    BodyRigBinding::BodyRigBinding(BodyProfile profile, const model::ModelAsset& asset)
        : profile_(std::move(profile)),asset_(asset)
    {
        const auto issues=validateBodyProfile(profile_,asset);
        if (!issues.empty())
        {
            std::string message="Body profile binding failed:";
            for (const auto& issue:issues) message+='\n'+issue.message;
            throw std::invalid_argument(message);
        }
        std::vector<BindingIssue> unused;
        const auto adapter=[&](const NodeLink& link,const glm::mat4& frame)
        {
            const auto n=resolve(link,asset,unused), parent=asset.nodes()[n].parent;
            const auto parentBind=parent==animation::NoNode ? glm::mat4{1}:asset.nodes()[parent].globalBindTransform;
            return Adapter{n,glm::inverse(parentBind)*glm::inverse(profile_.bodyFromResource)*frame,
                glm::inverse(frame)*profile_.bodyFromResource*asset.nodes()[n].globalBindTransform};
        };
        for (const auto& j:profile_.joints) joints_.push_back(adapter(j.node,j.frameInBodyBind));
        for (const auto& f:profile_.followers) followers_.push_back(adapter(f.node,profile_.joints[jointIndex(f.joint)].frameInBodyBind));
        for (const auto& e:profile_.effectors) effectors_.push_back(resolve(e.node,asset,unused));
    }
    std::size_t BodyRigBinding::jointIndex(JointId id) const
    {
        for (std::size_t i=0;i<profile_.joints.size();++i) if (profile_.joints[i].id==id) return i;
        throw std::out_of_range("Unknown semantic joint ID");
    }
    std::uint32_t BodyRigBinding::jointNode(JointId id) const { return joints_[jointIndex(id)].node; }
    glm::mat4 BodyRigBinding::frame(std::uint32_t node,const glm::mat4& bindFrame,std::span<const glm::mat4> globals) const
    {
        if (globals.size()!=asset_.nodes().size() || !finite(globals[node])) throw std::invalid_argument("Invalid body global pose");
        return profile_.bodyFromResource*globals[node]*glm::inverse(asset_.nodes()[node].globalBindTransform)*
            glm::inverse(profile_.bodyFromResource)*bindFrame;
    }
    glm::mat4 BodyRigBinding::jointFrame(JointId id,std::span<const glm::mat4> globals) const
    {
        const auto i=jointIndex(id); return frame(joints_[i].node,profile_.joints[i].frameInBodyBind,globals);
    }
    glm::mat4 BodyRigBinding::effectorFrame(EffectorId id,std::span<const glm::mat4> globals) const
    {
        for (std::size_t i=0;i<profile_.effectors.size();++i)
            if (profile_.effectors[i].id==id) return frame(effectors_[i],profile_.effectors[i].frameInBodyBind,globals);
        throw std::out_of_range("Unknown semantic effector ID");
    }
    glm::mat4 BodyRigBinding::worldFromResource(const glm::mat4& worldFromBody) const
    {
        if (!rigid(worldFromBody)) throw std::invalid_argument("worldFromBody must be a proper rigid pose in metres");
        return worldFromBody*profile_.bodyFromResource;
    }

    DirectBodyControl::DirectBodyControl(const BodyRigBinding& b):binding_(b),coordinates_(b.profile().joints.size(),glm::vec3{0}) {}
    void DirectBodyControl::reset() { std::fill(coordinates_.begin(),coordinates_.end(),glm::vec3{0}); }
    float DirectBodyControl::coordinate(JointId id,std::size_t k) const
    {
        const auto i=binding_.jointIndex(id);
        if (k>=binding_.profile_.joints[i].coordinates.size()) throw std::out_of_range("Invalid joint coordinate index");
        return coordinates_[i][static_cast<int>(k)];
    }
    JointCommandResult DirectBodyControl::setCoordinate(JointId id,std::size_t k,float radians,LimitPolicy policy)
    {
        const float previous=coordinate(id,k);
        if (!std::isfinite(radians)) throw std::invalid_argument("Non-finite joint command");
        if (policy!=LimitPolicy::Reject && policy!=LimitPolicy::Clamp) throw std::invalid_argument("Invalid joint limit policy");
        const auto i=binding_.jointIndex(id);
        const auto& c=binding_.profile_.joints[i].coordinates[k];
        const float value=std::clamp(radians,c.lower,c.upper);
        if (value!=radians && policy==LimitPolicy::Reject) return {CommandStatus::Rejected,radians,previous};
        coordinates_[i][static_cast<int>(k)]=value;
        return {value==radians ? CommandStatus::Applied:CommandStatus::Clamped,radians,value};
    }
    void DirectBodyControl::apply(std::span<glm::mat4> locals) const
    {
        if (locals.size()!=binding_.asset().nodes().size()) throw std::invalid_argument("Invalid body local pose size");
        if (!enabled_) return;
        applyCoordinates(locals,coordinates_);
    }
    void DirectBodyControl::applyCoordinates(std::span<glm::mat4> locals, std::span<const glm::vec3> coordinates) const
    {
        binding_.applyMeasuredCoordinates(locals,coordinates);
    }
    void BodyRigBinding::applyMeasuredCoordinates(std::span<glm::mat4> locals, std::span<const glm::vec3> coordinates) const
    {
        if (locals.size()!=asset().nodes().size() || coordinates.size()!=profile_.joints.size())
            throw std::invalid_argument("Invalid body display pose size");
        for (const auto& q:coordinates)
            if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z))
                throw std::invalid_argument("Non-finite measured coordinates");
        animation::setBindPose(asset().skeleton(),locals);
        for (std::size_t i=0;i<coordinates.size();++i)
        {
            if (coordinates[i]==glm::vec3{0}) continue; // Exact imported bind, including shear/reflection.
            const auto& a=joints_[i];
            locals[a.node]=a.parentToFrame*rotation(profile_.joints[i],coordinates[i])*a.frameToNode;
        }
        for (std::size_t i=0;i<followers_.size();++i)
        {
            const auto& f=profile_.followers[i];
            const auto j=jointIndex(f.joint);
            if (coordinates[j]==glm::vec3{0} || f.fraction==glm::vec3{1}) continue;
            const auto& a=followers_[i];
            locals[a.node]=a.parentToFrame*glm::inverse(rotation(profile_.joints[j],coordinates[j]))*
                rotation(profile_.joints[j],coordinates[j]*f.fraction)*a.frameToNode;
        }
    }
}
