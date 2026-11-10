#pragma once
#include <glm/glm.hpp>
#include <array>
#include <string>

namespace julia::physics
{
    // World normal points OUT of the fixed surface, towards the free tool.
    struct ContactTool
    {
        unsigned link=2;
        glm::dvec3 pointInJoint{0}; // Sphere centre; tool axes = hand joint axes.
        double radius=.018;
        std::string surface="wall";
        glm::dvec3 normalWorld{0,0,-1};
        double friction=.6; // Explicit pair, not one geom's material coefficient.
    };
    enum class ContactPhase { Idle, FreeMotion, Search, Confirm, Ramp, Hold, Unload, Retreat, Complete, Fault, Disabled };
    enum class ContactFailure { None, SearchLimit, ContactLost, ForeignContact, Saturation, HoldTimeout, GeometryLimit, Cancelled, ModeChanged };
    const char* contactPhaseName(ContactPhase phase);
    const char* contactFailureName(ContactFailure failure);
    struct ContactSettings
    {
        double force=3, forceRate=3;
        double freeTravel=.008, freeSeconds=1;
        double searchSpeed=.008, searchTravel=.06, searchSeconds=5;
        double confirmForce=.6, lossForce=.2, confirmSeconds=.04, lossSeconds=.08;
        double filterSeconds=.01;
        unsigned delaySamples=0; // Additional completed physical samples, 0..32.
        double forceP=.35, forceI=3, integralLimit=2;
        double normalDamping=35, tangentK=100, tangentD=12;
        double postureK=2, postureD=.3;
        double forceTolerance=.6, holdDwell=.5, holdSeconds=3, settleSeconds=1.5;
        double maxPenetration=.002, maxNormalSpeed=.01, saturationSeconds=.5;
        double retreatDistance=.025, retreatSeconds=1.5;
    };
    bool validContactSettings(const ContactSettings& settings);
    struct PairObservation
    {
        int toolGeom=-1,surfaceGeom=-1;
        unsigned records=0,active=0,foreignActive=0;
        bool near=false;
        glm::dvec3 pointWorld{0},forceOnToolWorld{0},tangentForceWorld{0};
        glm::dvec3 normalWorld{0,0,-1}; // Desired surface face normal, not a solver contact identifier.
        glm::dvec3 relativeTangentVelocityWorld{0};
        double normalForce=0,penetration=0,slipSpeed=0,friction=0;
        double geometricGap=0; // Sphere to the selected face, independent of solver records.
        double normalPower=0,tangentPower=0;
    };
    struct ContactStatus
    {
        ContactPhase phase=ContactPhase::Idle;
        ContactFailure failure=ContactFailure::None;
        ContactPhase samplePhase=ContactPhase::Idle;
        PairObservation pair;
        glm::dvec3 toolPointWorld{0},toolVelocityWorld{0},normalWorld{0,0,-1};
        glm::dmat4 worldFromTool{1};
        double goalForce=0,referenceForce=0,filteredForce=0,feedbackForce=0;
        double feedbackTime=0,phaseSeconds=0,searchDisplacement=0;
        double sampleFeedbackForce=0,sampleFeedbackTime=0,sampleReferenceForce=0;
        double integral=0,confirmationSeconds=0,lossSeconds=0,holdGoodSeconds=0;
        double saturationSeconds=0,normalWork=0,tangentWork=0,slipDistance=0;
        double peakForce=0,peakPenetration=0;
        bool confirmed=false,holdSucceeded=false,saturated=false;
    };
    struct ContactInput
    {
        double time=0;
        glm::dvec3 point{0},velocity{0};
        glm::dmat4 worldFromTool{1};
        std::array<glm::dvec3,44> jacobian{}; // World translational columns at tool centre.
        std::array<double,44> q{},v{};
        PairObservation pair;
    };
    // Pure bounded controller. No backend access, no q/v writes or external wrench.
    class ContactController
    {
    public:
        void reset();
        void start(ContactSettings settings,glm::dvec3 normal,const ContactInput& input);
        bool retarget(double force);
        void cancel();
        void disable(ContactFailure why=ContactFailure::ModeChanged);
        std::array<double,44> evaluate(const ContactInput& input,double h);
        void applied(bool saturated,double h);
        void observe(const ContactInput& input,double h);
        const ContactStatus& status() const { return state_; }
        bool owns() const;
    private:
        void phase(ContactPhase value);
        void fault(ContactFailure reason);
        ContactSettings settings_;
        ContactStatus state_;
        std::array<double,44> posture_{};
        glm::dvec3 origin_{0},searchOrigin_{0},retreatOrigin_{0};
        struct Delayed { double force=0,time=0; };
        std::array<Delayed,33> history_{};
        unsigned historyIndex_=0;
        double totalSeconds_=0,integralBefore_=0;
        bool hadSaturation_=false;
    };
}
