#pragma once
#include "BenchHud.hpp"

namespace contact_bench
{
    inline std::vector<julia::render_opengl::DebugLineVertex> hud(const Snapshot& s,
        const julia::render_opengl::CameraConfig& camera,bool paused)
    {
        const auto& c=s.contact; const auto& p=c.pair;
        std::ostringstream text; text<<std::fixed<<std::setprecision(3);
        text<<"FIXED ARM / NORMAL FORCE FEEDBACK   TICK "<<s.tick<<(paused?" PAUSED":" RUNNING")<<"\n";
        text<<"PAIR g2 <-> wall : "<<p.toolGeom<<" / "<<p.surfaceGeom<<"   PHASE "<<contactPhaseName(c.phase)<<"\n";
        text<<"TOOL CYAN / SURFACE ORANGE / NORMAL YELLOW / FORCE RED / SLIP GREEN\n";
        text<<"GOAL "<<c.goalForce<<"  RAMP "<<c.referenceForce<<"  SOLVER "<<p.normalForce<<" N\n";
        text<<"FILTER "<<c.filteredForce<<"  FEEDBACK "<<c.sampleFeedbackForce<<" N @ "<<c.sampleFeedbackTime<<" S\n";
        text<<"NORMAL "<<c.normalWorld.x<<" "<<c.normalWorld.y<<" "<<c.normalWorld.z<<"  CONFIRMED "<<c.confirmed<<"\n";
        text<<"TANGENT "<<glm::length(p.tangentForceWorld)<<" N  SLIP "<<p.slipSpeed*1000<<" MM/S  MU "<<p.friction<<"\n";
        text<<"PEN / GAP "<<p.penetration*1000<<" / "<<p.geometricGap*1000<<" MM  FOREIGN "<<p.foreignActive<<"\n";
        text<<"SAT "<<c.saturated<<" / "<<c.saturationSeconds<<" S  HOLD OK "<<c.holdSucceeded<<"  FAULT "<<contactFailureName(c.failure)<<"\n";
        text<<"CONTACT WORK NORMAL "<<c.normalWork<<" J  TANGENT "<<c.tangentWork<<" J\n";
        text<<"AXIS      REQUEST / ACTUAL NM     SAT\n";
        for(unsigned k=0;k<6;++k) text<<k<<"         "<<s.calculatedTorque[k]<<" / "<<s.appliedTorque[k]<<"          "<<s.saturatedThisTick[k]<<"\n";
        text<<"SPACE PAUSE   C CANCEL   D DISABLE   S START   R RESET   V CAMERA\nUP/DOWN FORCE +/-1 N IN HOLD   ESC CLOSE\n";
        auto str=text.str();
        struct Vertex { float x,y,z; unsigned char color[4]; };
        std::vector<Vertex> buffer(65536);
        const auto quads=stb_easy_font_print(0,0,str.data(),nullptr,buffer.data(),static_cast<int>(buffer.size()*sizeof(Vertex)));
        const glm::dvec3 forward=glm::normalize(glm::dvec3{camera.target-camera.position});
        const auto right=glm::normalize(glm::cross(forward,glm::dvec3{camera.up})), up=glm::cross(right,forward);
        const double height=camera.orthographicHeight, scale=height/720*1.35;
        const auto origin=glm::dvec3{camera.target}-right*(height*960/720/2-.015)+up*(height/2-.015);
        const auto point=[&](const Vertex& v) { return glm::vec3{origin+scale*(right*static_cast<double>(v.x)-up*static_cast<double>(v.y))}; };
        std::vector<julia::render_opengl::DebugLineVertex> lines;
        for(int q=0;q<quads;++q) for(int k=0;k<4;++k)
        {
            lines.push_back({point(buffer[static_cast<std::size_t>(q*4+k)]),{1,1,1}});
            lines.push_back({point(buffer[static_cast<std::size_t>(q*4+(k+1)%4)]),{1,1,1}});
        }
        return lines;
    }
}
