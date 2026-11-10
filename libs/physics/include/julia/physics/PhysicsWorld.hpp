#pragma once
#include "julia/physics/JointDrive.hpp"
#include "julia/physics/ContactControl.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

namespace julia::physics
{
    inline constexpr double TickSeconds = 1.0/120.0;
    inline constexpr unsigned Substeps = 8;
    inline constexpr unsigned ArmCoordinates = 6;
    inline constexpr unsigned MaxCoordinates = 44;
    using Coordinates = std::array<double,MaxCoordinates>;
    struct Hinge
    {
        int coordinate = -1;
        glm::dvec3 axis{1,0,0}; // In joint frame, intrinsic rotation after bind.
        double lower = 0, upper = 0, damping = .02; // rad, rad, N m s/rad.
        double limitTimeConstant = .01; // soft constraint seconds (>=2 substeps).
    };
    struct Link
    {
        std::string name;
        // -2: legacy serial predecessor; -1: world; otherwise earlier link index.
        int parent = -2;
        glm::dmat4 parentFromJoint{1}, jointFromSegment{1};
        glm::dvec3 comInSegment{0}; // metres; inertia at COM in segment axes.
        glm::dvec3 inertia{.001}; // Principal diagonal, kg m^2.
        double mass = 1, radius = .025; // kg, m.
        glm::dvec3 capsuleStart{0}, capsuleEnd{0,.2,0}; // In joint frame.
        // Ordered intrinsic rotations at the same pivot; empty means welded.
        // All coordinates share this body's single mass and inertial frame.
        std::vector<Hinge> hinges;
        // Optional oriented box in joint frame (feet). Otherwise capsule/sphere.
        std::optional<glm::dvec3> boxHalfSize;
        glm::dmat4 jointFromBox{1};
    };
    struct Wall { glm::dvec3 center{.2,1.05,.45}, halfSize{.35,.5,.03}; };
    struct Definition
    {
        std::vector<Link> links;
        std::array<Drive,MaxCoordinates> drives{{{80,8,12},{40,4,8},{40,4,8},{40,4,8},{8,.25,2},{6,.2,1.5}}};
        glm::dvec3 gravity{0,-9.81,0}; // m/s^2; Julia is Y-up.
        double contactTimeConstant=.01;
        std::optional<Wall> wall;
        std::optional<ContactTool> contactTool; // Select explicit pair and its accumulated observations; controller separately enabled.
        bool freeRoot=false, selfCollision=false;
        unsigned substeps=Substeps; // One world step per substep, independent of link count.
        std::optional<double> floorHeight;
        // Additional exclusions, on top of MuJoCo's parent/weld filtering.
        std::vector<std::array<unsigned,2>> exclusions;
        // Explicit experiment envelope, SI units. Legacy arm defaults unchanged.
        double maxJointSpeed=50, maxLimitViolation=.04, maxPenetration=.015;
        double maxContactForce=10000, maxKineticEnergy=2000;
        double maxRootLinearSpeed=20, maxRootAngularSpeed=50;
        unsigned maxContacts=64;
    };
    struct RootState
    {
        glm::dvec3 position{0}; // World position of physical root joint origin, m.
        glm::dquat orientation{1,0,0,0}; // body -> world; constructor (w,x,y,z).
        glm::dvec3 linearVelocityWorld{0}; // At root origin, m/s, world axes.
        glm::dvec3 angularVelocityLocal{0}; // rad/s, root body axes (MuJoCo qvel).
    };
    glm::dmat4 rootTransform(const RootState& root);
    struct StateLayout
    {
        unsigned nq=0,nv=0,nu=0;
        int rootQpos=-1, rootQvel=-1;
        unsigned rootQposSize=0,rootQvelSize=0;
        std::vector<int> qpos, qvel, actuator; // Indexed by scalar coordinate.
    };
    enum class CommandKind { Target, Torque, Disable, Reach, Attach, MoveRoot, SmoothTarget };
    struct Capabilities
    {
        bool fixedBase=true, effortDrive=true, positionDrive=true, externalWrench=true;
        bool contactForce=true, freeRoot=false, ik=false, attachment=false;
        bool smoothDrive=true, fixedBaseGravity=true;
        bool normalForceControl=false;
    };
    struct Contact
    {
        int geom1=-1, geom2=-1;
        int link1=-1, link2=-1; // -1: static environment.
        glm::dvec3 point{0}, normal{0};
        double distance=0;
        // Endpoint solver estimate in contact frame; not a force sensor or impulse.
        std::optional<glm::dvec3> force;
        glm::dmat3 worldFromContact{1}; // Columns = normal, tangent1, tangent2 in world.
        glm::dvec3 forceOn2World{0},torqueOn2World{0}; // Opposite signs on geom1.
        glm::dvec3 velocity1World{0},velocity2World{0},angular1World{0},angular2World{0};
        std::array<double,5> friction{}; // Actual compiled/mixed contact coefficients.
        bool active=false; // Solver constraint exists, not just broad/narrow phase proximity.
    };
    PairObservation aggregateContacts(const std::vector<Contact>& contacts,int tool,int surface,glm::dvec3 normal);
    struct Timing { double controlUs=0, stepUs=0, readUs=0, telemetryUs=0, contactUs=0, contactControlUs=0; };
    struct DriveTelemetry
    {
        DriveMode sampleMode=DriveMode::Disabled;
        // Completed-substep references and errors, matching snapshot q/v.
        Coordinates qRef{},vRef{},positionError{},velocityError{};
        // Last control evaluation at time-h, before the final integration.
        Coordinates sampleQ{},sampleV{},sampleQRef{},sampleVRef{},pTorque{},dTorque{},feedforward{},limitedTorque{};
        // Endpoint held actual torque * endpoint velocity; work at all substeps.
        Coordinates power{},deliveredWork{},absorbedWork{},saturatedSeconds{},saturationRunSeconds{};
        std::array<bool,MaxCoordinates> referenceReached{};
        double passiveDampingPower=0,passiveDampingWork=0;
        double externalPower=0,externalWork=0,mechanicalEnergyChange=0;
    };
    struct Snapshot
    {
        std::uint64_t tick=0, epoch=0, worldSteps=0;
        double time=0;
        DriveMode mode=DriveMode::Disabled;
        unsigned dofs=0;
        Coordinates target{}, q{}, v{}, calculatedTorque{}, appliedTorque{}, limitViolation{}, peakLimitViolationThisTick{};
        std::array<bool,MaxCoordinates> saturated{}, saturatedThisTick{};
        std::vector<glm::dmat4> worldFromJoint;
        std::vector<glm::dvec3> comWorld;
        std::vector<Contact> contacts;
        unsigned peakContactsThisTick=0, solverIterations=0, warnings=0;
        double peakNormalForceThisTick=0, maxPenetrationThisTick=0;
        glm::dvec3 externalForce{0}, externalTorque{0};
        double kineticEnergy=0, potentialEnergy=0;
        Timing timing;
        RootState root;
        glm::dvec3 totalComWorld{0}, linearMomentumWorld{0}, angularMomentumAboutComWorld{0};
        double totalMass=0;
        ServoSettings servo;
        DriveTelemetry drive;
        ContactStatus contact;
        // ALL constraints (contact, limits, equality, etc). Never labelled contact power.
        double constraintPower=0;
    };
    struct MassAudit
    {
        double totalMass=0;
        glm::dvec3 comWorld{0};
        glm::dmat3 inertiaAtComWorld{0};
        std::vector<double> masses, massMatrix;
        std::vector<glm::dvec3> comInJoint;
        std::vector<glm::dmat3> inertiaInJoint;
    };
    struct ContactMaterial
    {
        int geom1=-1,geom2=-1,dimension=0;
        std::array<double,5> friction{},solimp{};
        std::array<double,2> solref{};
    };
    void driveCsvHeader(std::ostream& out);
    void driveCsvRow(std::ostream& out,const Snapshot& snapshot,unsigned substeps);
    void contactCsvHeader(std::ostream& out);
    void contactCsvRow(std::ostream& out,const Snapshot& snapshot);
    void contactPointsCsvHeader(std::ostream& out);
    void contactPointsCsvRow(std::ostream& out,const Snapshot& snapshot);
    // One owner advances this WORLD once per configured substep. Read-only
    // snapshots/FK queries/render never integrate or set the live q/v.
    class PhysicsWorld
    {
    public:
        explicit PhysicsWorld(Definition definition,Coordinates initial={});
        ~PhysicsWorld();
        PhysicsWorld(const PhysicsWorld&)=delete;
        PhysicsWorld& operator=(const PhysicsWorld&)=delete;
        const Definition& definition() const;
        Capabilities capabilities() const;
        const StateLayout& layout() const;
        static std::string backendVersion();
        // Explicit experiment boundary: clears q/v, ctrl, warmstart, forces,
        // contact history, counters, targets and snapshots; increases epoch.
        void reset(Coordinates initial = {}, std::optional<RootState> root = std::nullopt);
        bool command(CommandKind kind, Coordinates values={}, std::string* reason=nullptr);
        // Atomic validation; changing settings while Servo is running is rejected.
        bool configureServo(ServoSettings settings,std::string* reason=nullptr);
        // Exclusive ownership of all six hinges, fixed Full arm with sphere tool.
        // Start requires Disabled; other drive commands require Disable first.
        bool startContact(ContactSettings settings={},std::string* reason=nullptr);
        bool contactForceTarget(double forceN,std::string* reason=nullptr);
        void cancelContact(); // Rate-limited unload and retreat, no q/v writes.
        // Pure gravity inverse-dynamics effort, at zero velocity, scratch only.
        // Free-base support is deliberately unavailable in stage7d.
        Coordinates gravityCompensation(Coordinates q) const;
        // Wrench in world axes, about chosen link COM. Held for the next tick only.
        void applyWrench(unsigned link, glm::dvec3 forceN, glm::dvec3 torqueNm={0,0,0});
        void step();
        const Snapshot& snapshot() const;
        // Scratch data only; explicitly allowed outside joint limits, never clamps.
        std::vector<glm::dmat4> forwardKinematics(Coordinates q, std::optional<RootState> root = std::nullopt) const;
        // Compiled inertial data and generalized mass matrix, scratch only.
        MassAudit massAudit(Coordinates q={}, std::optional<RootState> root = std::nullopt) const;
        std::optional<ContactMaterial> contactMaterial() const; // Compiled explicit pair, scratch/live unchanged.
        std::size_t modelBytes() const;
        std::size_t dataBytes() const;
        std::size_t arenaHighWaterBytes() const;
        std::string modelXml() const;
    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
