#include "julia/physics/JointDrive.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace julia::physics
{
    bool validServoSettings(const ServoSettings& s)
    {
        // Explicit engineering domain also excludes underflow-scale commands
        // whose phase durations cannot advance at the physical sample period.
        return std::isfinite(s.maxSpeed)&&s.maxSpeed>=1e-4&&s.maxSpeed<=10&&
            std::isfinite(s.maxAcceleration)&&s.maxAcceleration>=1e-4&&s.maxAcceleration<=100;
    }
    void JointReference::reset(double position)
    {
        if(!std::isfinite(position)) throw std::invalid_argument("Nonfinite reference");
        q_=goal_=position; v_=remaining_=0; phase_=count_=0; phases_={};
    }
    void JointReference::target(double goal,const ServoSettings& settings)
    {
        if(!std::isfinite(goal)||!validServoSettings(settings))
            throw std::invalid_argument("Invalid reference goal/limits");
        if(goal==goal_) return; // Repeated accepted targets do not restart braking.
        JointReference proposed=*this; proposed.goal_=goal; proposed.phase_=proposed.count_=0;
        const double a=settings.maxAcceleration;
        const auto add=[&](double t,double acceleration)
        {
            if(!std::isfinite(t)) throw std::invalid_argument("Nonfinite reference phase duration");
            if(t>0) proposed.phases_[proposed.count_++]={t,acceleration};
        };
        const double stop=q_+v_*std::abs(v_)/(2*a);
        if(!std::isfinite(stop)||!std::isfinite(goal-stop)) throw std::invalid_argument("Nonfinite reference displacement");
        add(std::abs(v_)/a,v_>0?-a:a);
        const double delta=goal-stop, distance=std::abs(delta), sign=delta>=0?1.:-1.;
        const double ramp=std::min(settings.maxSpeed/a,std::sqrt(distance/a));
        add(ramp,sign*a);
        if(ramp>0) add(std::max(0.,(distance-a*ramp*ramp)/(a*ramp)),0);
        add(ramp,-sign*a);
        proposed.remaining_=proposed.count_?proposed.phases_[0].seconds:0;
        if(!proposed.count_) { proposed.q_=goal; proposed.v_=0; }
        *this=proposed;
    }
    void JointReference::advance(double seconds)
    {
        if(!std::isfinite(seconds)||seconds<0) throw std::invalid_argument("Invalid reference time");
        while(seconds>0 && phase_<count_)
        {
            const double h=std::min(seconds,remaining_), a=phases_[phase_].acceleration;
            q_+=v_*h+.5*a*h*h; v_+=a*h; seconds-=h; remaining_-=h;
            if(remaining_<=1e-15)
            {
                ++phase_;
                if(phase_<count_) remaining_=phases_[phase_].seconds;
                else { q_=goal_; v_=0; }
            }
        }
    }
    DriveOutput evaluateDrive(const Drive& drive,DriveMode mode,double qr,double vr,
        double q,double v,double ff,double torque)
    {
        DriveOutput o;
        if(mode==DriveMode::Position||mode==DriveMode::Servo)
        {
            o.p=drive.kp*(qr-q);
            o.d=mode==DriveMode::Servo?drive.kd*(vr-v):-drive.kd*v;
            o.feedforward=mode==DriveMode::Servo?ff:0;
            o.requested=o.p+o.d;
            if(mode==DriveMode::Servo) o.requested+=o.feedforward;
        }
        else if(mode==DriveMode::Torque) o.requested=torque;
        o.limited=std::clamp(o.requested,-drive.maxTorque,drive.maxTorque);
        o.saturated=std::abs(o.requested)>drive.maxTorque;
        return o;
    }
    WorkIncrement integratePower(double before,double after,double h)
    {
        WorkIncrement w;
        if(before>=0 && after>=0) w.delivered=.5*(before+after)*h;
        else if(before<=0 && after<=0) w.absorbed=-.5*(before+after)*h;
        else
        {
            const double fraction=std::abs(before)/(std::abs(before)+std::abs(after));
            const double first=.5*before*h*fraction, second=.5*after*h*(1-fraction);
            w.delivered=std::max(first,0.)+std::max(second,0.);
            w.absorbed=-std::min(first,0.)-std::min(second,0.);
        }
        return w;
    }
}
