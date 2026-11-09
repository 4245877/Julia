#pragma once
#include <glm/glm.hpp>
#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace julia::physics
{
    inline constexpr double TickSeconds = 1.0/120.0;
    inline constexpr unsigned Substeps = 8;
    using Coordinates = std::array<double,2>;
    struct Hinge
    {
        int coordinate = -1; // -1: explicitly welded; otherwise 0 or 1.
        glm::dvec3 axis{1,0,0}; // In joint frame, intrinsic rotation after bind.
        double lower = 0, upper = 0, damping = .02; // rad, rad, N m s/rad.
    };
    struct Link
    {
        std::string name;
        // Serial tree: previous link is parent; first link is fixed to world.
        glm::dmat4 parentFromJoint{1}, jointFromSegment{1};
        glm::dvec3 comInSegment{0}; // metres; inertia at COM in segment axes.
        glm::dvec3 inertia{.001}; // Principal diagonal, kg m^2.
        double mass = 1, radius = .025; // kg, m.
        glm::dvec3 capsuleStart{0}, capsuleEnd{0,.2,0}; // In joint frame.
        Hinge hinge;
    };
    struct Drive { double kp=40, kd=4, maxTorque=8; }; // N m/rad, N m s/rad, N m.
    struct Wall { glm::dvec3 center{.2,1.05,.45}, halfSize{.35,.5,.03}; };
    struct Definition
    {
        std::vector<Link> links;
        std::array<Drive,2> drives{{{80,8,12},{40,4,8}}};
        glm::dvec3 gravity{0,-9.81,0}; // m/s^2; Julia is Y-up.
        std::optional<Wall> wall;
    };
    enum class DriveMode { Disabled, Position, Torque };
    enum class CommandKind { Target, Torque, Disable, Reach, Attach, MoveRoot };
    struct Capabilities
    {
        bool fixedBase=true, effortDrive=true, positionDrive=true, externalWrench=true;
        bool contactForce=true, freeRoot=false, ik=false, attachment=false;
    };
    struct Contact
    {
        int geom1=-1, geom2=-1;
        glm::dvec3 point{0}, normal{0};
        double distance=0;
        // Endpoint solver estimate in contact frame; not a force sensor or impulse.
        std::optional<glm::dvec3> force;
    };
    struct Timing { double controlUs=0, stepUs=0, readUs=0; };
    struct Snapshot
    {
        std::uint64_t tick=0, epoch=0, worldSteps=0;
        double time=0;
        DriveMode mode=DriveMode::Disabled;
        unsigned dofs=0;
        Coordinates target{}, q{}, v{}, calculatedTorque{}, appliedTorque{}, limitViolation{};
        std::array<bool,2> saturated{}, saturatedThisTick{};
        std::vector<glm::dmat4> worldFromJoint;
        std::vector<glm::dvec3> comWorld;
        std::vector<Contact> contacts;
        unsigned peakContactsThisTick=0, solverIterations=0, warnings=0;
        double peakNormalForceThisTick=0, maxPenetrationThisTick=0;
        glm::dvec3 externalForce{0}, externalTorque{0};
        double kineticEnergy=0, potentialEnergy=0;
        Timing timing;
    };
    // One owner advances this WORLD exactly eight times per outer tick. Read-only
    // snapshots/FK queries/render never integrate or set the live q/v.
    class PhysicsWorld
    {
    public:
        explicit PhysicsWorld(Definition definition);
        ~PhysicsWorld();
        PhysicsWorld(const PhysicsWorld&)=delete;
        PhysicsWorld& operator=(const PhysicsWorld&)=delete;
        const Definition& definition() const;
        static Capabilities capabilities() { return {}; }
        static std::string backendVersion();
        // Explicit experiment boundary: clears q/v, ctrl, warmstart, forces,
        // contact history, counters, targets and snapshots; increases epoch.
        void reset(Coordinates initial = {});
        bool command(CommandKind kind, Coordinates values={}, std::string* reason=nullptr);
        // Wrench in world axes, about chosen link COM. Held for the next tick only.
        void applyWrench(unsigned link, glm::dvec3 forceN, glm::dvec3 torqueNm={0,0,0});
        void step();
        const Snapshot& snapshot() const;
        // Scratch data only; explicitly allowed outside joint limits, never clamps.
        std::vector<glm::dmat4> forwardKinematics(Coordinates q) const;
        std::size_t modelBytes() const;
        std::size_t dataBytes() const;
        std::size_t arenaHighWaterBytes() const;
        std::string modelXml() const;
    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
