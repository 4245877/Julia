#pragma once
#include "julia/embodiment/ArmReach.hpp"
#include "julia/animation/Pose.hpp"
namespace julia::app
{
    struct CollisionDemo { embodiment::ReachTarget target; std::shared_ptr<const world::CollisionScene> scene; };
    inline CollisionDemo collisionDemo(const embodiment::BodyRigBinding& binding,bool blocked,std::uint64_t version,const glm::mat4& worldFromBody=glm::mat4{1},
        embodiment::ArmSide arm=embodiment::ArmSide::Left,unsigned effectorOffset=0)
    {
        using namespace embodiment;
        if (effectorOffset>2) throw std::invalid_argument("Unsupported demo hand effector");
        const bool left=arm==ArmSide::Left;
        const auto shoulder=left ? JointId::LeftShoulder:JointId::RightShoulder, elbow=left ? JointId::LeftElbow:JointId::RightElbow,
            wrist=left ? JointId::LeftWrist:JointId::RightWrist;
        const auto effector=static_cast<EffectorId>((left ? 0:4)+effectorOffset);
        DirectBodyControl source(binding); source.setEnabled(true);
        source.setCoordinate(shoulder,0,.8f); source.setCoordinate(shoulder,1,.6f);
        source.setCoordinate(shoulder,2,.1f); source.setCoordinate(elbow,0,1.2f);
        animation::Pose pose(binding.asset().skeleton()); source.apply(pose.locals);
        animation::computeGlobalTransforms(binding.asset().skeleton(),pose.locals,pose.globals);
        CollisionDemo demo; demo.target.arm=arm; demo.target.effector=effector;
        demo.target.position=glm::vec3{binding.effectorFrame(effector,pose.globals)[3]};
        source.reset(); const auto solution=solveArmPosition(source,demo.target,demo.target.position);
        const std::array joints{shoulder,elbow,wrist};
        for (std::size_t k=0;k<6;++k) source.setCoordinate(joints[k<3 ? 0:k==3 ? 1:2],k<3 ? k:k==3 ? 0:k-4,static_cast<float>(solution.coordinates[k]*.5));
        source.apply(pose.locals); animation::computeGlobalTransforms(binding.asset().skeleton(),pose.locals,pose.globals);
        const auto forms=bodyColliders(binding,pose.globals,worldFromBody);
        std::vector<world::SceneObject> objects;
        if (blocked) { world::SceneObject o; o.id=101; const auto form=left ? 1u:4u; o.position=(forms[form].capsule.a+forms[form].capsule.b)*.5;
            o.rotation=glm::normalize(glm::dquat{glm::quat_cast(worldFromBody)}); o.shape=world::Box{{.001,.008,.008}}; objects.push_back(o); }
        world::SceneObject sphere; sphere.id=102; sphere.position=glm::dvec3{worldFromBody*glm::vec4{-.5f,1,.3f,1}};
        sphere.shape=world::Sphere{.045}; objects.push_back(sphere);
        world::SceneObject floor; floor.id=103; floor.position=glm::dvec3{worldFromBody[3]};
        floor.rotation=glm::normalize(glm::dquat{glm::quat_cast(worldFromBody)}); floor.shape=world::Plane{}; objects.push_back(floor);
        demo.scene=std::make_shared<const world::CollisionScene>(version,std::move(objects)); return demo;
    }
}
