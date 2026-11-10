#pragma once
#include <array>

namespace julia::physics
{
    struct Drive { double kp=40, kd=4, maxTorque=8; }; // Nm/rad, Nm s/rad, Nm.
    enum class DriveMode { Disabled, Position, Torque, Servo };
    struct ServoSettings
    {
        double maxSpeed=.6, maxAcceleration=2; // rad/s, rad/s2, reference only.
        bool gravityCompensation=false; // Supported only for fixed bases.
    };
    bool validServoSettings(const ServoSettings& settings);
    // Continuous q/v, piecewise constant acceleration. Retarget brakes the
    // current reference to rest before a triangular/trapezoidal move to goal.
    // No allocation, no access to physical state, no post-integration clamp.
    class JointReference
    {
    public:
        void reset(double position);
        void target(double goal,const ServoSettings& settings);
        void advance(double seconds);
        double q() const { return q_; }
        double v() const { return v_; }
        bool reached() const { return phase_==count_; }
    private:
        struct Phase { double seconds=0, acceleration=0; };
        std::array<Phase,4> phases_{};
        unsigned phase_=0,count_=0;
        double q_=0,v_=0,goal_=0,remaining_=0;
    };
    struct DriveOutput
    {
        double p=0,d=0,feedforward=0,requested=0,limited=0;
        bool saturated=false;
    };
    // Sole implementation of legacy PD, smooth servo, direct torque and clamp.
    DriveOutput evaluateDrive(const Drive& drive,DriveMode mode,double qRef,double vRef,
        double q,double v,double feedforward,double directTorque);
    struct WorkIncrement { double delivered=0,absorbed=0; };
    // Exact integral of the positive/negative parts of linearly interpolated
    // endpoint powers, including a zero crossing inside a physical substep.
    WorkIncrement integratePower(double powerBefore,double powerAfter,double seconds);
}
