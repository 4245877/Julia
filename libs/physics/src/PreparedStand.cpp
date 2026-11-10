#include "julia/physics/BodyRig.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <stdexcept>

namespace julia::physics
{
    StandSettings preparedStand(const BodyRig& rig,const PhysicsWorld& world,const glm::dmat4& placement)
    {
        if(rig.binding().profile().rigFingerprint!=8613155383294238335ull) throw std::invalid_argument("Prepared stance profile mismatch");
        StandSettings p;
        p.posture={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-.12,0,-.12,0,0,0,0,-.12,0,-.12,0,0,0,0,
            .3425800340444959,.00714538107611379,.014756785157529217,.24461026047099255,-.09195066519989689,-.03155380339941451,
            -.13737713893120398,-.010364384215177826,.005651895777470873,.07492093425090064,.21109169940945474,.023080230784454626};
        p.rootGoal.position=glm::dvec3{placement*glm::dvec4{0,.86,0,1}};
        p.rootGoal.orientation=glm::normalize(glm::quat_cast(glm::dmat3{placement}));
        p.comGoal=world.massAudit(p.posture,p.rootGoal).comWorld;
        const auto frames=world.forwardKinematics(p.posture,p.rootGoal);
        p.thoraxGoal=glm::normalize(glm::quat_cast(glm::dmat3{frames[2]}));
        const std::array<double,2> normal{219.29267708924203,320.257322910758};
        const std::array<glm::dvec3,2> moment{glm::dvec3{-6.07344547926881,0,-5.991539824472191},glm::dvec3{13.174754135309112,0,13.848210475916916}};
        for(std::size_t side=0;side<2;++side)
        {
            const auto link=side==0?16u:19u;
            const auto box=frames[link]*world.definition().links[link].jointFromBox;
            p.nominalReaction[side]={glm::dvec3{box*glm::dvec4{0,-.025,0,1}},p.rootGoal.orientation*glm::dvec3{0,normal[side],0},p.rootGoal.orientation*moment[side]};
        }
        return p;
    }
}
