#include "julia/physics/ContactControl.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace julia::physics
{
    const char* contactPhaseName(ContactPhase p)
    {
        switch(p) {
        case ContactPhase::Idle:return "idle"; case ContactPhase::FreeMotion:return "free";
        case ContactPhase::Search:return "search"; case ContactPhase::Confirm:return "confirm";
        case ContactPhase::Ramp:return "ramp"; case ContactPhase::Hold:return "hold";
        case ContactPhase::Unload:return "unload"; case ContactPhase::Retreat:return "retreat";
        case ContactPhase::Complete:return "complete"; case ContactPhase::Fault:return "fault";
        case ContactPhase::Disabled:return "disabled";
        } return "invalid";
    }
    const char* contactFailureName(ContactFailure f)
    {
        switch(f) {
        case ContactFailure::None:return "none"; case ContactFailure::SearchLimit:return "search-limit";
        case ContactFailure::ContactLost:return "contact-lost"; case ContactFailure::ForeignContact:return "foreign-contact";
        case ContactFailure::Saturation:return "saturation"; case ContactFailure::HoldTimeout:return "hold-timeout";
        case ContactFailure::GeometryLimit:return "geometry-limit"; case ContactFailure::Cancelled:return "cancelled";
        case ContactFailure::ModeChanged:return "mode-changed";
        } return "invalid";
    }
    bool validContactSettings(const ContactSettings& s)
    {
        for(double x:{s.force,s.forceRate,s.freeTravel,s.freeSeconds,s.searchSpeed,s.searchTravel,s.searchSeconds,
            s.confirmForce,s.lossForce,s.confirmSeconds,s.lossSeconds,s.filterSeconds,s.forceP,s.forceI,s.integralLimit,
            s.normalDamping,s.tangentK,s.tangentD,s.postureK,s.postureD,s.forceTolerance,s.holdDwell,s.holdSeconds,
            s.settleSeconds,s.maxPenetration,s.maxNormalSpeed,s.saturationSeconds,s.retreatDistance,s.retreatSeconds})
            if(!std::isfinite(x)||x<0||x>1000) return false;
        return s.force>=1&&s.force<=8&&s.forceRate>=.1&&s.forceRate<=5&&
            s.freeTravel<=.02&&s.freeSeconds>=.5&&s.freeSeconds<=3&&
            s.searchSpeed>=.001&&s.searchSpeed<=.02&&s.searchTravel>=.005&&s.searchTravel<=.08&&
            s.searchSeconds>=.5&&s.searchSeconds<=8&&s.lossForce>0&&s.confirmForce>s.lossForce&&s.confirmForce<=1&&
            s.confirmSeconds>=.01&&s.lossSeconds>=.02&&s.filterSeconds>=.001&&s.filterSeconds<=.05&&s.delaySamples<=32&&
            s.forceP<=2&&s.forceI<=10&&s.integralLimit<=5&&s.normalDamping>=5&&s.normalDamping<=100&&
            s.tangentK<=200&&s.tangentD<=40&&s.postureK<=10&&s.postureD<=2&&
            s.forceTolerance>0&&s.forceTolerance<=1&&s.holdDwell>=.1&&s.holdDwell<=2&&
            s.holdSeconds>=s.holdDwell&&s.holdSeconds<=10&&s.settleSeconds>=.5&&s.settleSeconds<=4&&
            s.maxPenetration>=.0005&&s.maxPenetration<=.003&&s.maxNormalSpeed>0&&s.maxNormalSpeed<=.02&&
            s.saturationSeconds>=.05&&s.saturationSeconds<=2&&s.retreatDistance>=.01&&s.retreatDistance<=.05&&
            s.retreatSeconds>=1&&s.retreatSeconds<=4;
    }
    bool ContactController::owns() const
    { return state_.phase!=ContactPhase::Idle&&state_.phase!=ContactPhase::Disabled; }
    void ContactController::reset() { *this=ContactController{}; }
    void ContactController::phase(ContactPhase p) { state_.phase=p; state_.phaseSeconds=0; }
    void ContactController::fault(ContactFailure why)
    { state_.failure=why; state_.integral=0; state_.referenceForce=0; state_.holdSucceeded=false; phase(ContactPhase::Fault); }
    void ContactController::start(ContactSettings s,glm::dvec3 normal,const ContactInput& in)
    {
        if(!validContactSettings(s)||!std::isfinite(glm::length(normal))||std::abs(glm::length(normal)-1)>1e-8)
            throw std::invalid_argument("Invalid contact settings/normal");
        const auto accumulated=state_;
        reset(); settings_=s; origin_=searchOrigin_=in.point; posture_=in.q;
        state_.normalWork=accumulated.normalWork; state_.tangentWork=accumulated.tangentWork;
        state_.slipDistance=accumulated.slipDistance; state_.peakForce=accumulated.peakForce;
        state_.peakPenetration=accumulated.peakPenetration;
        state_.normalWorld=normal; state_.goalForce=s.force; state_.toolPointWorld=in.point;
        state_.worldFromTool=in.worldFromTool;
        state_.referenceForce=s.confirmForce; // Initial low-force reference, inactive during geometric approach/search.
        state_.pair=in.pair; phase(ContactPhase::FreeMotion);
    }
    bool ContactController::retarget(double f)
    {
        if(!std::isfinite(f)||f<1||f>8||(state_.phase!=ContactPhase::Ramp&&state_.phase!=ContactPhase::Hold)) return false;
        state_.goalForce=f; state_.holdSucceeded=false; state_.holdGoodSeconds=0;
        phase(ContactPhase::Ramp); return true;
    }
    void ContactController::cancel()
    {
        if(!owns()||state_.phase==ContactPhase::Complete||state_.phase==ContactPhase::Fault) return;
        state_.failure=ContactFailure::Cancelled;
        if(!state_.confirmed) { state_.referenceForce=0; retreatOrigin_=state_.toolPointWorld; phase(ContactPhase::Retreat); }
        else phase(ContactPhase::Unload);
    }
    void ContactController::disable(ContactFailure why)
    { if(owns()) { state_.failure=why; state_.integral=0; state_.referenceForce=0; phase(ContactPhase::Disabled); } }
    std::array<double,44> ContactController::evaluate(const ContactInput& in,double h)
    {
        std::array<double,44> torque{};
        if(!owns()) return torque;
        auto& s=state_; const auto& c=settings_; const auto n=s.normalWorld;
        s.sampleFeedbackForce=s.feedbackForce; s.sampleFeedbackTime=s.feedbackTime;
        const double vn=glm::dot(in.velocity,n);
        const auto tangent=[&](glm::dvec3 a) { return a-n*glm::dot(a,n); };
        double fn=0;
        if(s.phase==ContactPhase::FreeMotion)
        {
            const double fraction=std::min(1.,s.phaseSeconds/c.freeSeconds);
            const double offset=-c.freeTravel*(3*fraction*fraction-2*fraction*fraction*fraction);
            const double speed=-c.freeTravel*6*fraction*(1-fraction)/c.freeSeconds;
            fn=180*(offset-glm::dot(in.point-origin_,n))+c.normalDamping*(speed-vn);
            if(s.phaseSeconds>=c.freeSeconds) { searchOrigin_=in.point; phase(ContactPhase::Search); }
        }
        if(s.phase==ContactPhase::Search||s.phase==ContactPhase::Confirm)
        {
            // Velocity-limited search reference with bounded requested force.
            fn=std::clamp(140*(-c.searchSpeed-vn),-3.,3.);
            if(s.phase==ContactPhase::Confirm) fn=-(c.confirmForce+.2)-c.normalDamping*vn;
        }
        if(s.phase==ContactPhase::Ramp||s.phase==ContactPhase::Hold||s.phase==ContactPhase::Unload)
        {
            const double goal=s.phase==ContactPhase::Unload?0:s.goalForce;
            s.referenceForce+=std::clamp(goal-s.referenceForce,-c.forceRate*h,c.forceRate*h);
            const double error=s.referenceForce-s.feedbackForce;
            integralBefore_=s.integral;
            if(!hadSaturation_&&s.confirmed&&s.referenceForce>c.lossForce)
                s.integral=std::clamp(s.integral+error*h,-c.integralLimit,c.integralLimit);
            if(s.phase==ContactPhase::Unload) s.integral*=std::exp(-10*h);
            fn=-s.referenceForce-c.forceP*error-c.forceI*s.integral-c.normalDamping*vn;
            if(s.phase==ContactPhase::Ramp&&std::abs(s.referenceForce-goal)<1e-10) phase(ContactPhase::Hold);
            if(s.phase==ContactPhase::Hold&&s.phaseSeconds>=c.holdSeconds) phase(ContactPhase::Unload);
            if(s.phase==ContactPhase::Unload&&s.referenceForce<=1e-10)
            { s.integral=0; retreatOrigin_=in.point; phase(ContactPhase::Retreat); }
        }
        if(s.phase==ContactPhase::Retreat||s.phase==ContactPhase::Complete)
        {
            const double t=s.phase==ContactPhase::Complete?1.:std::min(1.,s.phaseSeconds/c.retreatSeconds);
            const double offset=c.retreatDistance*(3*t*t-2*t*t*t);
            const double speed=c.retreatDistance*6*t*(1-t)/c.retreatSeconds;
            fn=180*(offset-glm::dot(in.point-retreatOrigin_,n))+c.normalDamping*(speed-vn);
            if(s.phase==ContactPhase::Retreat&&s.phaseSeconds>=c.retreatSeconds&&
                in.pair.normalForce<c.lossForce&&glm::dot(in.point-retreatOrigin_,n)>.8*c.retreatDistance)
                phase(ContactPhase::Complete);
        }
        // Fault retains damping/gravity only: no further inward force or integral.
        if(s.phase==ContactPhase::Fault) fn=-c.normalDamping*vn;
        const auto ft=-c.tangentK*tangent(in.point-origin_)-c.tangentD*tangent(in.velocity);
        s.sampleReferenceForce=s.referenceForce;
        s.samplePhase=s.phase;
        const auto force=n*std::clamp(fn,-15.,15.)+ft;
        std::array<double,44> raw{};
        glm::dmat3 gram{1e-8}; glm::dvec3 projectedRaw{0};
        for(unsigned k=0;k<6;++k)
        {
            raw[k]=c.postureK*(posture_[k]-in.q[k])-c.postureD*in.v[k];
            gram+=glm::outerProduct(in.jacobian[k],in.jacobian[k]);
            projectedRaw+=in.jacobian[k]*raw[k];
        }
        const auto compensation=glm::inverse(gram)*projectedRaw;
        for(unsigned k=0;k<6;++k)
            torque[k]=glm::dot(in.jacobian[k],force)+raw[k]-glm::dot(in.jacobian[k],compensation);
        return torque;
    }
    void ContactController::applied(bool saturated,double h)
    {
        if(!owns()) return;
        state_.saturated=saturated; hadSaturation_=saturated;
        state_.saturationSeconds=saturated?state_.saturationSeconds+h:0;
        if(state_.phase==ContactPhase::Fault) { state_.integral=0; return; }
        if(saturated) state_.integral=integralBefore_; // Conditional integration with current common torque cap.
        if(state_.saturationSeconds>=settings_.saturationSeconds&&state_.phase!=ContactPhase::Fault)
            fault(ContactFailure::Saturation);
    }
    void ContactController::observe(const ContactInput& in,double h)
    {
        auto& s=state_; s.pair=in.pair; s.toolPointWorld=in.point; s.toolVelocityWorld=in.velocity;
        s.normalWorld=in.pair.normalWorld; s.worldFromTool=in.worldFromTool;
        s.normalWork+=in.pair.normalPower*h; s.tangentWork+=in.pair.tangentPower*h;
        s.slipDistance+=in.pair.slipSpeed*h;
        s.peakForce=std::max(s.peakForce,in.pair.normalForce); s.peakPenetration=std::max(s.peakPenetration,in.pair.penetration);
        if(!owns()) return;
        totalSeconds_+=h; s.phaseSeconds+=h;
        s.filteredForce+=(1-std::exp(-h/settings_.filterSeconds))*(in.pair.normalForce-s.filteredForce);
        history_[historyIndex_]={s.filteredForce,in.time};
        const auto delayed=(historyIndex_+33-settings_.delaySamples)%33;
        s.feedbackForce=history_[delayed].force; s.feedbackTime=history_[delayed].time;
        historyIndex_=(historyIndex_+1)%33;
        if(s.phase==ContactPhase::Fault||s.phase==ContactPhase::Complete) return;
        if(in.pair.foreignActive) { fault(ContactFailure::ForeignContact); return; }
        if(in.pair.penetration>settings_.maxPenetration) { fault(ContactFailure::GeometryLimit); return; }
        if(s.phase==ContactPhase::Search||s.phase==ContactPhase::Confirm)
        {
            s.searchDisplacement=-glm::dot(in.point-searchOrigin_,s.normalWorld);
            if(s.searchDisplacement>=settings_.searchTravel||totalSeconds_>=settings_.freeSeconds+settings_.searchSeconds)
            { fault(ContactFailure::SearchLimit); return; }
            if(s.phase==ContactPhase::Search&&in.pair.normalForce>=settings_.confirmForce) phase(ContactPhase::Confirm);
            if(s.phase==ContactPhase::Confirm)
            {
                s.confirmationSeconds=in.pair.normalForce>=settings_.confirmForce?s.confirmationSeconds+h:0;
                if(s.confirmationSeconds>=settings_.confirmSeconds)
                { s.confirmed=true; s.referenceForce=settings_.confirmForce; phase(ContactPhase::Ramp); }
                else if(in.pair.normalForce<=settings_.lossForce) phase(ContactPhase::Search);
            }
        }
        if(s.confirmed&&(s.phase==ContactPhase::Ramp||s.phase==ContactPhase::Hold))
        {
            s.lossSeconds=in.pair.normalForce<=settings_.lossForce?s.lossSeconds+h:0;
            if(s.lossSeconds>=settings_.lossSeconds) { fault(ContactFailure::ContactLost); return; }
        }
        if(s.phase==ContactPhase::Hold)
        {
            const bool good=std::abs(in.pair.normalForce-s.goalForce)<=settings_.forceTolerance&&
                std::abs(glm::dot(in.velocity,s.normalWorld))<=settings_.maxNormalSpeed&&
                in.pair.penetration<=settings_.maxPenetration&&!s.saturated;
            s.holdGoodSeconds=good?s.holdGoodSeconds+h:0;
            if(s.holdGoodSeconds>=settings_.holdDwell) s.holdSucceeded=true;
            if(!s.holdSucceeded&&s.phaseSeconds>=settings_.settleSeconds+settings_.holdDwell)
                fault(ContactFailure::HoldTimeout);
        }
        if(s.phase==ContactPhase::Retreat&&s.phaseSeconds>=settings_.retreatSeconds+2)
            fault(ContactFailure::HoldTimeout);
    }
}
