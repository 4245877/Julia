#pragma once
#include <julia/render_opengl/OpenGLRenderer.hpp>
#include <glm/gtc/matrix_transform.hpp>
#if defined(_MSC_VER)
#pragma warning(push,0)
#endif
#include <stb_easy_font.h>
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
#include <sstream>

namespace bench
{
    inline std::vector<julia::render_opengl::DebugLineVertex> driveHud(
        const julia::physics::Snapshot& s,const julia::render_opengl::CameraConfig& camera,
        unsigned selected,bool paused,int width=960,int heightPixels=720,bool fixedBase=true)
    {
        std::ostringstream text; text<<std::fixed<<std::setprecision(3);
        text<<(fixedBase?"FIXED BASE":s.mode==julia::physics::DriveMode::Stand?"FREE ROOT / LIMITED TWO FOOT STAND":"FREE ROOT / NO BALANCE")<<"   TICK "<<s.tick<<"   "<<(paused?"PAUSED":"RUNNING")<<"   SELECTED "<<selected<<"\n";
        text<<"Q: RAD  V: RAD/S  TORQUE: NM   "<<(fixedBase?"TARGET MAGENTA / ACTUAL GREEN":"REFERENCE ORANGE / ACTUAL CYAN")<<"\n";
        text<<"MODE "<<static_cast<int>(s.mode)<<"  GRAVITY FF "<<(s.servo.gravityCompensation?"ON":"OFF")<<"  REF ORANGE\n";
        text<<"AXIS   TARGET    ACTUAL    V        CALC      APPLIED   SAT   LIMIT / PEAK\n";
        for(unsigned k=0;k<s.dofs;++k) if(s.dofs<=6||k==selected)
            text<<k<<"      "<<std::setw(6)<<s.target[k]<<"    "<<std::setw(6)<<s.q[k]<<"    "<<std::setw(6)<<s.v[k]
                <<"   "<<std::setw(7)<<s.calculatedTorque[k]<<"   "<<std::setw(7)<<s.appliedTorque[k]<<"   "<<s.saturatedThisTick[k]
                <<"    "<<std::scientific<<std::setprecision(1)<<s.limitViolation[k]<<"/"<<s.peakLimitViolationThisTick[k]<<std::fixed<<std::setprecision(3)<<"\n";
        text<<"CONTACTS "<<s.contacts.size()<<"  FORCE "<<s.peakNormalForceThisTick<<" N  PEN "<<s.maxPenetrationThisTick*1000
            <<" MM  SOLVER "<<s.solverIterations<<" WARN "<<s.warnings<<"\n";
        {
        const auto& d=s.drive; const auto k=selected;
        text<<"REF ORANGE "<<d.qRef[k]<<" VREF "<<d.vRef[k]<<" ERR "<<d.positionError[k]<<" SAT TIME "<<d.saturatedSeconds[k]<<" S\n";
        text<<"PD "<<d.pTorque[k]<<"/"<<d.dTorque[k]<<" FF "<<d.feedforward[k]<<" LIMIT "<<d.limitedTorque[k]<<" NM\n";
        text<<"POWER "<<d.power[k]<<" W  WORK +"<<d.deliveredWork[k]<<" -"<<d.absorbedWork[k]<<" J (SELECTED AXIS)\n";
        }
        text<<"BODY E "<<s.kineticEnergy+s.potentialEnergy<<" J  DELTA "<<s.drive.mechanicalEnergyChange<<" J\n";
        text<<"PASSIVE "<<s.drive.passiveDampingWork<<" J  EXTERNAL "<<s.drive.externalWork<<" J\n";
        if(!fixedBase)
        {
            if(s.mode==julia::physics::DriveMode::Stand)
            {
                const auto& t=s.stand;
                text<<"STAND "<<julia::physics::standPhaseName(t.phase)<<" / "<<t.reason<<" FEEDBACK "<<t.balanceFeedback<<" HOLD "<<t.holdSeconds<<" S\n";
                text<<"COM ERR "<<t.comError<<" M SPEED "<<t.speed<<" M/S TILT "<<t.tilt<<" RAD UTIL "<<t.requestedUtilization<<"\n";
                text<<"PREDICTED ORANGE / ACTUAL GREEN-BLUE / GOAL ROOT ORANGE\n";
                text<<"PRED N "<<t.predictedReaction[0].force.y<<" / "<<t.predictedReaction[1].force.y<<" N, SOLVES "<<t.linearSolves<<" CTRL "<<t.controllerUs<<" US\n";
                if(t.variant==julia::physics::StandVariant::Bounded7h||t.commandSerial>0)
                {
                    if(!t.load.enabled) text<<"COM "<<julia::physics::comPhaseName(t.comPhase)<<" #"<<t.commandSerial<<" ERR "<<1000*t.commandError<<" MM HOLD "<<t.achievedSeconds<<" S\n";
                    text<<"GOAL MAGENTA / REFERENCE ORANGE / ACTUAL YELLOW; TOP INSET X30\n";
                    text<<"QP "<<t.solverCode<<" IT "<<t.qpIterations<<" ACTIVE "<<t.activeInequalities<<" CAPS "<<t.activeCaps<<" FRICTION "<<t.activeFriction<<" NORMAL "<<t.activeNormals<<"\n";
                    if(!t.load.enabled) text<<"J/L X-/X+  K Z+  O ORIGIN  C CANCEL; 8 SHIFT / 9 RETARGET\n";
                }
                if(t.load.enabled)
                {
                    const auto& a=t.load;
                    text<<"LOAD "<<julia::physics::comPhaseName(a.phase)<<" #"<<a.serial<<" TASK "<<(a.taskEnabled?"ON":"OFF")<<" HOLD "<<a.achievedSeconds<<" S\n";
                    text<<"ALPHA LEFT COMMAND "<<a.command<<" REF "<<a.reference<<" ACTUAL ";
                    if(a.actualValid) text<<a.actual; else text<<"INVALID";
                    text<<" PRED "; if(a.predictedValid) text<<a.predicted; else text<<"INVALID";
                    text<<"\nALPHA0 "; if(a.baselineValid) text<<a.alpha0; else text<<"CALIBRATING";
                    text<<"  FN L/R "<<s.support.feet[0].normalForce<<" / "<<s.support.feet[1].normalForce<<" N\n";
                    text<<"COM ERR "<<1000*t.comError<<" MM  HORIZONTAL SPEED "<<1000*t.speed<<" MM/S\n";
                    text<<"[ / ] ALPHA0 -/+ .05   O RETURN ALPHA0   C SMOOTH STOP\n";
                }
            }
            const auto& o=s.support;
            if(!o.enabled) text<<"SUPPORT OBSERVER DISABLED\n";
            else
            {
                text<<"SOLE GRAY / LOADED HULL GREEN / COM YELLOW / TOTAL COP WHITE\nLEFT GREEN / RIGHT BLUE / OTHER CONTACT ORANGE / SELF MAGENTA\n";
                for(std::size_t side=0;side<2;++side)
                {
                    const auto& f=o.feet[side];
                    text<<(side==0?"LEFT ":"RIGHT ")<<"NEAR "<<f.near<<" RECORDS "<<f.records<<" ACTIVE "<<f.active<<" LOADED "<<f.load.loaded<<" N "<<f.normalForce<<" T "<<glm::length(f.tangentForce)<<" N\n";
                    text<<"M@SOLE "<<f.wrench.moment.x<<' '<<f.wrench.moment.y<<' '<<f.wrench.moment.z<<" NM / SLIP "<<f.slipSpeed<<" M/S\n";
                    text<<"COP "<<julia::physics::copReasonName(f.cop.reason);
                    if(f.cop.point) text<<' '<<f.cop.point->x<<' '<<f.cop.point->z<<" M";
                    text<<" / REGION "<<julia::physics::regionName(f.loadedRegion.kind)<<"\n";
                }
                const auto& d=o.dynamics;
                text<<"TOTAL COP "<<julia::physics::copReasonName(o.combinedCop.reason)<<" / OTHER "<<o.otherEnvironmentActive<<" SELF "<<o.selfActive<<"\n";
                text<<"COM V "<<glm::length(d.comVelocity)<<" M/S / INTEGRAL RESIDUAL P "<<glm::length(d.linearResidual)<<" NS L "<<glm::length(d.angularResidual)<<" NMS\n";
            }
        }
        text<<"TAB: AXIS  UP/DOWN: TARGET  V: CAMERA  R: RESET  SPACE: PAUSE\n";
        auto str=text.str();
        struct Vertex { float x,y,z; unsigned char color[4]; };
        std::vector<Vertex> buffer(65536);
        const auto quads=stb_easy_font_print(0,0,str.data(),nullptr,buffer.data(),static_cast<int>(buffer.size()*sizeof(Vertex)));
        const glm::dvec3 forward=glm::normalize(glm::dvec3{camera.target-camera.position});
        const auto right=glm::normalize(glm::cross(forward,glm::dvec3{camera.up})), up=glm::cross(right,forward);
        const double height=camera.orthographicHeight, scale=height/heightPixels*1.35;
        const auto origin=glm::dvec3{camera.target}-right*(height*width/heightPixels/2-.015)+up*(height/2-.015);
        const auto point=[&](const Vertex& v) { return glm::vec3{origin+scale*(right*static_cast<double>(v.x)-up*static_cast<double>(v.y))}; };
        std::vector<julia::render_opengl::DebugLineVertex> lines;
        for(int q=0;q<quads;++q) for(int k=0;k<4;++k)
        {
            lines.push_back({point(buffer[static_cast<std::size_t>(q*4+k)]),{1,1,1}});
            lines.push_back({point(buffer[static_cast<std::size_t>(q*4+(k+1)%4)]),{1,1,1}});
        }
        return lines;
    }
    inline std::vector<julia::render_opengl::DebugLineVertex> hud(
        const julia::physics::RenderSnapshot& view,const julia::render_opengl::CameraConfig& camera,unsigned selected,bool paused)
    { return driveHud(view.physics,camera,selected,paused); }
}
