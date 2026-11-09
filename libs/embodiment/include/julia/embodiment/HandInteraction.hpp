#pragma once
#include "julia/embodiment/ArmReach.hpp"
#include <functional>

namespace julia::embodiment
{
    struct RigidPose { glm::vec3 position{0}; glm::quat rotation{1,0,0,0}; };
    glm::mat4 rigidMatrix(const RigidPose& pose);
    RigidPose rigidPose(const glm::mat4& matrix);
    struct ObjectHandle
    {
        std::uint64_t id=0, generation=0;
        bool operator==(const ObjectHandle&) const = default;
    };
    enum class ObjectState { Free, Attached };
    enum class HoldType { None, AssistedAttachment };
    struct InteractiveObject
    {
        ObjectHandle handle;
        RigidPose worldPose;
        float radius=.022f;
        RigidPose localGrip;
        ObjectState state=ObjectState::Free;
        std::uint64_t owner=0;
        ArmSide arm=ArmSide::Left;
        glm::mat4 effectorFromObject{1};
    };
    // Exactly one 44 mm sphere, one grip per hand. Not a general object database.
    struct CreateInteractionSphere { std::uint64_t id; RigidPose pose; float radius=.022f; RigidPose localGrip; };
    struct DeleteInteractionObject { ObjectHandle object; };
    struct GraspObject { ObjectHandle object; ArmSide arm=ArmSide::Left; };
    struct MoveHeldObject { ObjectHandle object; RigidPose targetWorld; };
    struct ReleaseObject { ObjectHandle object; };
    struct CancelInteraction { std::uint64_t action; };
    struct SetHandClosure { ArmSide arm; float closure; bool enabled=true; };
    enum class InteractionPhase { Idle, Opening, Approach, Align, Closing, Holding, Transport,
        ReleaseOpening, Retreat, Cancelling, Succeeded, Failed, Cancelled, ReleaseBlocked };
    const char* interactionPhaseName(InteractionPhase phase);
    struct HandState { bool enabled=false; float closure=0, velocity=0, blend=0; };
    struct InteractionObservation
    {
        std::uint64_t action=0, epoch=0;
        ObjectHandle object;
        ArmSide arm=ArmSide::Left;
        InteractionPhase phase=InteractionPhase::Idle;
        HoldType hold=HoldType::None;
        bool cancellationPending=false;
        float frameError=0, angularError=0;
        std::string result, reason;
        // No force/weight/friction model. Absence is intentional, never zero force.
        std::optional<float> gripForce;
        unsigned phaseCheckPoses=0, phaseCheckQueries=0;
        double phaseCheckMicroseconds=0;
        std::size_t phaseCheckScratchBytes=0;
    };
    class HandLayer
    {
    public:
        struct Joint
        {
            std::uint32_t node;
            unsigned side;
            glm::mat4 before{1}, after{1};
            glm::quat open{1,0,0,0}, closed{1,0,0,0};
        };
        explicit HandLayer(const BodyRigBinding& binding);
        const auto& joints() const { return joints_; }
        const auto& states() const { return states_; }
        void set(ArmSide arm,float closure,bool enabled=true);
        void advance();
        void reset() { states_={}; from_={}; goal_={}; elapsed_={}; duration_={}; blendFrom_={}; }
        bool ready(ArmSide arm) const;
        void apply(std::span<glm::mat4> locals) const { apply(locals,states_); }
        void apply(std::span<glm::mat4> locals,const std::array<HandState,2>& states) const;
    private:
        std::size_t nodeCount_=0;
        std::vector<Joint> joints_;
        std::array<HandState,2> states_{};
        std::array<float,2> from_{},goal_{};
        std::array<float,2> blendFrom_{};
        std::array<unsigned,2> elapsed_{},duration_{};
    };
    // Single writer, owned by BodyInstance. All reach work uses the existing bounded jobs.
    class HandInteraction
    {
    public:
        HandInteraction(std::uint64_t owner,DirectBodyControl& control,ArmReach& reach);
        HandLayer hand;
        const auto& object() const { return object_; }
        const auto& observation() const { return observation_; }
        bool locked() const;
        void create(const CreateInteractionSphere& command);
        void erase(ObjectHandle handle);
        void grasp(std::uint64_t action,std::uint64_t epoch,const GraspObject& command,const glm::mat4& root);
        void move(std::uint64_t action,std::uint64_t epoch,const MoveHeldObject& command,const glm::mat4& root);
        void release(std::uint64_t action,std::uint64_t epoch,ObjectHandle handle);
        void cancel(std::uint64_t action);
        void resetFree(std::uint64_t epoch);
        void beginTick() { observation_.phaseCheckPoses=observation_.phaseCheckQueries=0; observation_.phaseCheckMicroseconds=0; }
        void advance(std::span<const glm::mat4> globals,const glm::mat4& root);
        CollisionContext collisionContext() const;
        std::optional<world::SceneObject> freeCollider() const;
        std::function<void()> geometryChanged;
        std::function<PathCheck(CollisionContext)> validateRest;
    private:
        std::uint64_t owner_,generation_=0;
        DirectBodyControl& control_;
        ArmReach& reach_;
        std::optional<InteractiveObject> object_;
        InteractionObservation observation_;
        RigidPose grip_,pregrip_;
        bool cancelBeforeAttachment_=false;
        bool contactAllowed_=false;
        std::uint64_t reachAction_=0;
        void requireObject(ObjectHandle handle) const;
        void phase(InteractionPhase phase,bool geometry=false);
        void startReach(const RigidPose& goal,const glm::mat4& root);
        void fail(const std::string& reason);
        bool restClear(CollisionContext context);
    };
}
