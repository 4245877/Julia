#pragma once
#include <glm/glm.hpp>
#include <array>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

namespace julia::physics
{
    struct Contact;
    struct Snapshot;
    // All vectors are world axes, SI. Origin is part of every wrench.
    struct Wrench
    {
        glm::dvec3 origin{0}, force{0}, moment{0};
        void add(glm::dvec3 point,glm::dvec3 forceN,glm::dvec3 torqueNm={0,0,0});
        Wrench about(glm::dvec3 newOrigin) const;
    };
    bool forceActive(const Contact& contact);
    Wrench contactWrench(const Contact& contact,int receivingGeom,glm::dvec3 origin);
    Wrench pairWrench(const std::vector<Contact>& contacts,int receivingGeom,int surfaceGeom,glm::dvec3 origin);
    enum class RegionKind { Empty, Point, Segment, Polygon };
    const char* regionName(RegionKind kind);
    struct PlanarRegion
    {
        RegionKind kind=RegionKind::Empty;
        std::vector<glm::dvec3> vertices; // Y-up plane, ordered convex boundary.
        double area=0; // m^2; zero for degenerate regions.
    };
    PlanarRegion horizontalHull(std::vector<glm::dvec3> points);
    enum class CopReason { Available, ObserverDisabled, NoFloor, InsufficientNormalForce, Nonfinite };
    const char* copReasonName(CopReason reason);
    struct PressureCenter
    {
        std::optional<glm::dvec3> point;
        CopReason reason=CopReason::ObserverDisabled;
        double normalForce=0, normalMoment=0;
    };
    // Known stationary horizontal floor. The wrench is transported to planeOrigin
    // without discarding contact-height moments or replacing actual contact points.
    PressureCenter pressureCenter(const Wrench& wrench,glm::dvec3 planeOrigin,double minimumNormal=20);
    struct FootSpec
    {
        std::string segment, geom; // Semantic segment + compiled geom NAME.
        glm::dmat4 jointFromSole{1};
    };
    struct SupportSettings
    {
        std::array<FootSpec,2> feet;
        double confirmForce=30, lossForce=10, confirmSeconds=.020, lossSeconds=.040;
        double copMinimumForce=20, nearDistance=.002;
    };
    struct LoadState
    {
        bool loaded=false;
        double confirmationSeconds=0, lossSeconds=0;
        void observe(double normalForce,double dt,const SupportSettings& settings);
    };
    struct FootObservation
    {
        int link=-1,geom=-1,surfaceGeom=-1;
        bool near=false;
        unsigned records=0,active=0;
        double geometricGap=0,penetration=0,normalForce=0,slipSpeed=0;
        glm::dvec3 tangentForce{0},relativeVelocity{0}; // Mean of active points; max slip separately.
        Wrench wrench; // About measured Sole origin, NOT box COM.
        PressureCenter cop;
        LoadState load;
        PlanarRegion soleProjection,loadedRegion;
        // Bound on CoP displacement if one naively projects points and discards
        // their height*tangential-force couples. Not a solver error estimate.
        double projectionErrorBound=0;
    };
    struct DynamicsObservation
    {
        struct Sample
        {
            double time=0,dt=0;
            glm::dvec3 com{0},linearMomentum{0},angularMomentum{0},forceBefore{0},momentBefore{0},force{0},moment{0};
        };
        bool valid=false; // Requires free base: no unmeasured base reaction.
        glm::dvec3 comVelocity{0},comVelocityJacobian{0};
        Wrench gravity,environment,applied,total; // About instantaneous total COM.
        glm::dvec3 linearImpulse{0},angularImpulse{0}; // Composite trapezoid per physical substep.
        glm::dvec3 linearImpulseEndpoint{0},angularImpulseEndpoint{0}; // Right rectangle, comparison only.
        glm::dvec3 deltaLinearMomentum{0},deltaAngularMomentum{0};
        glm::dvec3 linearResidual{0},angularResidual{0};
        glm::dvec3 tickLinearImpulse{0},tickAngularImpulse{0};
        unsigned samples=0;
        std::vector<Sample> substeps; // Bounded completed tick, for independent quadrature audit.
    };
    struct SupportObservation
    {
        bool enabled=false,floorAvailable=false;
        glm::dvec3 planeOrigin{0},planeNormal{0,1,0},comProjection{0};
        std::array<FootObservation,2> feet;
        PressureCenter combinedCop;
        PlanarRegion loadedRegion,soleProjection;
        unsigned otherEnvironmentRecords=0,otherEnvironmentActive=0,selfRecords=0,selfActive=0;
        Wrench otherEnvironment; // About COM; includes wall/nonfoot contacts, excludes self.
        DynamicsObservation dynamics;
    };
    // Shared read-only snapshot math; hysteresis and quadrature are instance state
    // in the owning PhysicsWorld. No drive commands, MuJoCo calls or q/v access.
    void observeSupports(SupportObservation& output,const Snapshot& state,const SupportSettings& settings,
        const std::array<int,2>& links,const std::array<int,2>& geoms,int floor,
        std::optional<double> floorHeight,const std::vector<glm::dmat4>& jointFromBox,
        const std::vector<glm::dvec3>& boxHalfSize,double dt);
    void supportCsvHeader(std::ostream& out);
    void supportCsvRow(std::ostream& out,const Snapshot& state);
    void dynamicsCsvHeader(std::ostream& out);
    void dynamicsCsvRow(std::ostream& out,const Snapshot& state);
    void dynamicsSubstepsCsvHeader(std::ostream& out);
    void dynamicsSubstepsCsvRow(std::ostream& out,const Snapshot& state);
    void supportRegionsCsvHeader(std::ostream& out);
    void supportRegionsCsvRow(std::ostream& out,const Snapshot& state);
}
