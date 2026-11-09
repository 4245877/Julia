#pragma once
#include "julia/physics/PhysicsWorld.hpp"
#include "julia/embodiment/BodyProfile.hpp"
#include "julia/animation/Pose.hpp"
#include <ostream>

namespace julia::physics
{
    class ArmRig
    {
    public:
        explicit ArmRig(const model::ModelAsset& asset);
        const embodiment::BodyRigBinding& binding() const { return binding_; }
        Definition description(double massScale=1,double inertiaScale=1,double payloadKg=0,bool wall=false) const;
        animation::Pose visualPose(Coordinates measured) const;
        std::array<glm::dmat4,3> visualFrames(const animation::Pose& pose) const;
    private:
        embodiment::BodyRigBinding binding_;
    };
    struct RenderSnapshot
    {
        Snapshot physics;
        animation::Pose actualPose, targetPose;
        glm::dvec3 actualWrist{0}, targetWrist{0};
        double mappingUs=0;
    };
    // One completed state, no interpolation of solver telemetry. Render and CSV
    // consume this same value. A render may repeat it arbitrarily without stepping.
    RenderSnapshot publish(const ArmRig& rig,const Snapshot& state);
    struct FkParity { double position=0, rotationElement=0; };
    FkParity verifyFramesBeforeDynamics(const ArmRig& rig,const PhysicsWorld& world);
    enum class BenchCase { Passive, Hold, Heavy, Inertia, Weak, Impulse, Contact };
    BenchCase parseBenchCase(const std::string& name);
    std::string benchCaseName(BenchCase scenario);
    Definition benchDefinition(const ArmRig& rig,BenchCase scenario);
    void resetBench(PhysicsWorld& world,BenchCase scenario);
    void stepBench(PhysicsWorld& world,BenchCase scenario);
    void csvHeader(std::ostream& out);
    void csvRow(std::ostream& out,const RenderSnapshot& snapshot);
    void writeCalibration(std::ostream& out,const ArmRig& rig,const PhysicsWorld& world);
}
