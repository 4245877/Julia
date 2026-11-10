#pragma once
#include "julia/physics/ArmRig.hpp"

namespace julia::physics
{
    // One free pelvis plus 41 profile hinges. Pelvis's three kinematic angles
    // are represented by the free orientation, never duplicated as hinges.
    class BodyRig
    {
    public:
        explicit BodyRig(const model::ModelAsset& asset);
        const embodiment::BodyRigBinding& binding() const { return binding_; }
        Definition description(bool floor=true) const;
        animation::Pose visualPose(Coordinates measured) const;
        glm::dmat4 worldFromBody(const RootState& root) const;
        std::vector<glm::dmat4> visualFrames(const animation::Pose& pose,const RootState& root) const;
        RootState initialRoot(const glm::dmat4& worldFromBody=glm::dmat4{1}) const;
    private:
        embodiment::BodyRigBinding binding_;
        std::vector<int> parents_;
    };
    struct BodyRenderSnapshot
    {
        Snapshot physics;
        animation::Pose actualPose;
        glm::dmat4 worldFromResource{1};
        double mappingUs=0;
        std::vector<glm::dmat4> referenceFrames;
    };
    BodyRenderSnapshot publish(const BodyRig& rig,const Snapshot& state);
    FkParity verifyFramesBeforeDynamics(const BodyRig& rig,const PhysicsWorld& world);
    enum class BodyCase { Drop, Tilt, Pulse, Internal, Passive };
    BodyCase parseBodyCase(const std::string& name);
    std::string bodyCaseName(BodyCase scenario);
    Definition bodyBenchDefinition(const BodyRig& rig,BodyCase scenario);
    void resetBodyBench(const BodyRig& rig,PhysicsWorld& world,BodyCase scenario);
    void stepBodyBench(PhysicsWorld& world,BodyCase scenario,bool smoothScript=false);
    void writeBodyCalibration(std::ostream& out,const BodyRig& rig,const PhysicsWorld& world);
}
