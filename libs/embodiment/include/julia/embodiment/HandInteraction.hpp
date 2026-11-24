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
    // Explicit calibration of the object frame supplied by CreateInteractionSphere.
    enum class SphereGrasp { Legacy, LeftPalmar };
    struct InteractiveObject
    {
        ObjectHandle handle;
        RigidPose worldPose;
        InteractionShape shape=world::Sphere{.022f};
        RigidPose localGrip;
        ObjectState state=ObjectState::Free;
        std::uint64_t owner=0;
        ArmSide arm=ArmSide::Left;
        glm::mat4 effectorFromObject{1};
        SphereGrasp sphereGrasp=SphereGrasp::Legacy;
        EffectorId gripEffector(ArmSide side) const;
        world::SceneObject collider() const;
    };
    ReachTarget graspPregripTarget(ArmSide arm,const InteractiveObject& object);
    // One interactive object at a time.
    struct CreateInteractionSphere {
        std::uint64_t id; RigidPose pose; float radius=.022f; RigidPose localGrip;
        SphereGrasp grasp=SphereGrasp::Legacy;
    };
    inline constexpr double InteractionCubeHalfExtent=.018;
    inline const RigidPose interactionCubeLocalGrip{{.003f,-.017f,-.008f}};
    struct CreateInteractionBox { std::uint64_t id; RigidPose pose; world::Box box{glm::dvec3{InteractionCubeHalfExtent}}; };
    struct DeleteInteractionObject { ObjectHandle object; };
    struct GraspObject { ObjectHandle object; ArmSide arm=ArmSide::Left; };
    // Explicit, one-shot handoff from a completed Reach in the same body/epoch.
    struct PrepareGrasp { ObjectHandle object; ArmSide arm; std::uint64_t reach, reachEpoch; };
    struct ConfirmGrasp { ObjectHandle object; std::uint64_t action, epoch; };
    // Finish means checked separation and release of the arm, never attachment.
    struct FinishGraspPreparation { std::uint64_t action; };
    struct MoveHeldObject { ObjectHandle object; RigidPose targetWorld; };
    struct ReleaseObject { ObjectHandle object; };
    struct CancelInteraction { std::uint64_t action; };
    struct SetHandClosure { ArmSide arm; float closure; bool enabled=true; };
    enum class InteractionPhase { Idle, Opening, Approach, Align, Closing, Holding, Transport,
        ReleaseOpening, Retreat, Cancelling, Succeeded, Failed, Cancelled, ReleaseBlocked,
        Handoff, ReadyToGrasp, PreparationBlocked };
    const char* interactionPhaseName(InteractionPhase phase);
    enum class HandGrip { Sphere, Cube };
    // Anatomical joint angles of one hand, radians. Finger flexion is positive towards the
    // palm, spread towards the thumb, both measured in the hand frame. Thumb CMC angles are
    // relative to the modelled thumb; MCP/IP flexion turns the tip towards the thumb pad.
    struct FingerAngles { float mcpFlexion=0, mcpSpread=0, pipFlexion=0, dipFlexion=0; };
    struct ThumbAngles { float palmarAbduction=0, radialAbduction=0, pronation=0, mcpFlexion=0, ipFlexion=0; };
    struct HandPose { std::array<FingerAngles,4> fingers{}; ThumbAngles thumb{}; }; // index, middle, ring, pinky
    HandPose mixHandPose(const HandPose& from,const HandPose& to,float weight);
    // Rest is the calibrated relaxed pose when relaxed, otherwise the exact bind pose.
    struct HandState {
        bool enabled=false; float closure=0, velocity=0, blend=0; HandGrip grip=HandGrip::Sphere; bool relaxed=false;
        std::optional<HandPose> articulated;
    };
    // Calibrated targets shared by both hands through mirror symmetry.
    HandPose relaxedHandPose();    // Idle rest
    HandPose openHandPose();       // Empty-hand opening and grasp preparation
    HandPose oppositionHandPose(); // Pre-pinch: the thumb pad turned towards the index pad
    struct SpherePadContact
    {
        float gap=0, facing=0;
        glm::vec3 point{0}; // In the left wrist frame, metres.
        bool supported=false;
    };
    struct SphereContacts
    {
        std::array<SpherePadContact,5> pads;
        float skinGap=0, thumbOpposition=0;
        bool supporting=false;
    };
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
        bool preparation=false, geometryCurrent=false;
        std::uint64_t sourceReach=0, sourceEpoch=0;
        std::uint32_t contactHand=0;
        std::uint64_t contactObject=0;
        PathCheck geometryCheck; // Latest phase check; historical until dependenciesMatch.
        std::optional<SphereContacts> sphereContacts;
    };
    struct SphereHandStroke
    {
        HandPose closed;
        std::array<float,5> stopTime; // Each digit eases to rest at this fraction of the closing interval.
        HandPose sample(float progress) const;
    };
    class HandLayer
    {
    public:
        struct Joint
        {
            std::uint32_t node;
            unsigned side, finger, phalanx; // finger 0..4 index..thumb; phalanx 0..2 from the palm
            glm::mat4 before{1}, after{1};
            glm::quat frame{1,0,0,0}; // Bind orientation of the joint frame, body metres.
            glm::quat open{1,0,0,0}, closed{1,0,0,0}, cubeClosed{1,0,0,0}, rest{1,0,0,0};
            // Aperture of the calibrated cube/sphere holds. Closing passes through it so the
            // contact part of the stroke keeps the verified path to the closed shapes.
            glm::quat graspAperture{1,0,0,0};
        };
        // Closure fractions: the aperture is reached by ApertureEnd, the hold stroke starts at StrokeStart.
        static constexpr float ApertureEnd=.6f, StrokeStart=.15f;
        // Calibrated anatomy of one hand in the bind pose, unit axes in body space.
        // The profile wrist frame keeps its historical axes; its +Z points to the back
        // of the hand on this rig, so palmar is -Z.
        struct Anatomy
        {
            glm::vec3 distal{0}, palmar{0}, radial{0}, spread{0};
            std::array<glm::vec3,4> flexion{}; // shared by MCP, PIP and DIP of each finger
            glm::vec3 thumbPalmar{0}, thumbRadial{0}, thumbAxial{0}, thumbMcp{0}, thumbIp{0};
            HandPose bind; // modelled angles; thumb CMC angles are zero by definition
        };
        explicit HandLayer(const BodyRigBinding& binding);
        const auto& joints() const { return joints_; }
        const auto& states() const { return states_; }
        const Anatomy& anatomy(unsigned side) const { return anatomy_.at(side); }
        // Local rotations of the 15 joints of one side, in joints() order, reproducing the pose.
        std::array<glm::quat,15> rotations(unsigned side,const HandPose& pose) const;
        void applyPose(std::span<glm::mat4> locals,unsigned side,const HandPose& pose) const;
        void set(ArmSide arm,float closure,bool enabled=true,std::optional<HandGrip> grip={});
        void fadeOut(ArmSide arm);
        void advance();
        // Idle owns the relaxed rest pose; reset returns both hands to the exact bind pose.
        void setRelaxed(bool relaxed) { for (auto& s:states_) s.relaxed=relaxed; }
        void reset() { states_={}; from_={}; goal_={}; elapsed_={}; duration_={}; blendFrom_={}; blendGoal_={}; sphereStroke_.reset(); }
        void closeAroundSphere(SphereHandStroke stroke);
        bool ready(ArmSide arm) const;
        void apply(std::span<glm::mat4> locals) const { apply(locals,states_); }
        void apply(std::span<glm::mat4> locals,const std::array<HandState,2>& states) const;
    private:
        std::size_t nodeCount_=0;
        std::vector<Joint> joints_;
        std::array<Anatomy,2> anatomy_{};
        std::array<HandState,2> states_{};
        std::array<float,2> from_{},goal_{};
        std::array<float,2> blendFrom_{},blendGoal_{};
        std::array<unsigned,2> elapsed_{},duration_{};
        std::optional<SphereHandStroke> sphereStroke_;
    };
    class HandSphereContact;
    // Single writer, owned by BodyInstance. All reach work uses the existing bounded jobs.
    class HandInteraction
    {
    public:
        HandInteraction(std::uint64_t owner,DirectBodyControl& control,ArmReach& reach);
        ~HandInteraction();
        HandLayer hand;
        const auto& object() const { return object_; }
        const auto& observation() const { return observation_; }
        bool locked() const;
        void create(const CreateInteractionSphere& command,const world::CollisionScene* scene=nullptr,double minimumGap=0);
        void create(const CreateInteractionBox& command,const world::CollisionScene* scene,double minimumGap);
        void erase(ObjectHandle handle);
        void grasp(std::uint64_t action,std::uint64_t epoch,const GraspObject& command,const glm::mat4& root);
        void prepare(std::uint64_t action,std::uint64_t epoch,const PrepareGrasp& command,PathCheck openingCheck);
        void confirm(std::uint64_t action,const ConfirmGrasp& command);
        void finishPreparation(std::uint64_t action);
        void abandonPreparation(); // ResetBody only: explicit epoch discontinuity, Free pose retained.
        void move(std::uint64_t action,std::uint64_t epoch,const MoveHeldObject& command,const glm::mat4& root);
        void release(std::uint64_t action,std::uint64_t epoch,ObjectHandle handle);
        void cancel(std::uint64_t action);
        void resetFree(std::uint64_t epoch);
        void beginTick() { observation_.phaseCheckPoses=observation_.phaseCheckQueries=0; observation_.phaseCheckMicroseconds=0; }
        void advance(std::span<const glm::mat4> globals,const glm::mat4& root);
        CollisionContext collisionContext() const;
        std::optional<world::SceneObject> freeCollider() const;
        std::function<void()> geometryChanged;
        std::function<PathCheck(ArmSide,CollisionContext)> validateRest;
        std::function<void(ArmSide)> freezeReach;
        std::function<bool()> stanceValid;
        std::uint64_t geometryRevision() const { return geometryRevision_; }
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
        std::uint64_t geometryRevision_=0;
        bool handoffWait_=false, finishingPreparation_=false;
        std::unique_ptr<HandSphereContact> sphereContact_;
        bool closingStarted_=false;
        void createObject(std::uint64_t id,RigidPose pose,InteractionShape shape,RigidPose localGrip,
            SphereGrasp sphereGrasp,const world::CollisionScene* scene,double minimumGap);
        void requireObject(ObjectHandle handle) const;
        void phase(InteractionPhase phase,bool geometry=false);
        void startReach(const RigidPose& goal,const glm::mat4& root);
        void fail(const std::string& reason);
        void retreatBlocked(const std::string& reason);
        bool restClear(CollisionContext context);
    };
}
