#pragma once
#include "julia/physics/BodyRig.hpp"
#include "julia/model_import/ModelImporter.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace bodybench
{
    using namespace julia::physics;
    struct Options
    {
        BodyCase scenario=BodyCase::Drop;
        std::uint64_t ticks=720;
        unsigned fps=60;
        bool verify=false, explicitTicks=false;
        std::filesystem::path output="build/diagnostics/stage7c/run";
    };
    inline Options parse(int argc,char** argv)
    {
        Options o;
        for(int i=1;i<argc;++i)
        {
            const std::string flag=argv[i];
            const auto value=[&]() { if(++i>=argc) throw std::invalid_argument("Missing value: "+flag); return std::string{argv[i]}; };
            if(flag=="--case") o.scenario=parseBodyCase(value());
            else if(flag=="--output") o.output=value();
            else if(flag=="--verify") { o.verify=true; o.output=value(); }
            else if(flag=="--ticks"||flag=="--fps")
            {
                const auto s=value(); std::size_t used=0;
                if(s.empty()||s[0]=='-') throw std::invalid_argument("Invalid count");
                const auto n=std::stoull(s,&used);
                if(used!=s.size()||n==0||n>1200000) throw std::invalid_argument("Invalid count");
                if(flag=="--ticks") { o.ticks=n; o.explicitTicks=true; }
                else { if(n!=30&&n!=60&&n!=144) throw std::invalid_argument("Frame rate must be 30/60/144"); o.fps=static_cast<unsigned>(n); }
            }
            else throw std::invalid_argument("Unknown option: "+flag);
        }
        return o;
    }
    struct Log
    {
        std::ofstream state,joints,contacts;
        explicit Log(const std::filesystem::path& dir)
        {
            std::filesystem::create_directories(dir);
            state.open(dir/"state.csv"); joints.open(dir/"joints.csv"); contacts.open(dir/"contacts.csv");
            if(!state||!joints||!contacts) throw std::runtime_error("Cannot create CSV");
            state<<"tick,epoch,time_s,world_steps,root_x_m,root_y_m,root_z_m,qw,qx,qy,qz,vx_world_m_s,vy_world_m_s,vz_world_m_s,wx_local_rad_s,wy_local_rad_s,wz_local_rad_s,com_x_m,com_y_m,com_z_m,px_kg_m_s,py_kg_m_s,pz_kg_m_s,Lx_kg_m2_s,Ly_kg_m2_s,Lz_kg_m2_s,kinetic_J,potential_J,contacts,peak_contacts,peak_normal_N,peak_penetration_m,solver_iterations,warnings,external_fx_N,external_fy_N,external_fz_N,mapping_us\n";
            joints<<"tick,epoch,coordinate,q_rad,v_rad_s,target_rad,calculated_Nm,applied_Nm,saturated_tick,limit_rad,peak_limit_rad\n";
            contacts<<"tick,epoch,geom1,geom2,link1,link2,distance_m,x_m,y_m,z_m,nx,ny,nz,normal_N,tangent1_N,tangent2_N\n";
            state<<std::setprecision(17); joints<<std::setprecision(17); contacts<<std::setprecision(17);
        }
        void row(const BodyRenderSnapshot& view)
        {
            const auto& s=view.physics; const auto& r=s.root;
            const auto vec=[&](glm::dvec3 v) { state<<','<<v.x<<','<<v.y<<','<<v.z; };
            state<<s.tick<<','<<s.epoch<<','<<s.time<<','<<s.worldSteps;
            vec(r.position); state<<','<<r.orientation.w<<','<<r.orientation.x<<','<<r.orientation.y<<','<<r.orientation.z;
            vec(r.linearVelocityWorld); vec(r.angularVelocityLocal); vec(s.totalComWorld); vec(s.linearMomentumWorld); vec(s.angularMomentumAboutComWorld);
            state<<','<<s.kineticEnergy<<','<<s.potentialEnergy<<','<<s.contacts.size()<<','<<s.peakContactsThisTick<<','<<s.peakNormalForceThisTick
                <<','<<s.maxPenetrationThisTick<<','<<s.solverIterations<<','<<s.warnings;
            vec(s.externalForce); state<<','<<view.mappingUs<<'\n';
            for(unsigned k=0;k<s.dofs;++k) joints<<s.tick<<','<<s.epoch<<','<<k<<','<<s.q[k]<<','<<s.v[k]<<','<<s.target[k]<<','<<s.calculatedTorque[k]
                <<','<<s.appliedTorque[k]<<','<<s.saturatedThisTick[k]<<','<<s.limitViolation[k]<<','<<s.peakLimitViolationThisTick[k]<<'\n';
            for(const auto& c:s.contacts)
            {
                contacts<<s.tick<<','<<s.epoch<<','<<c.geom1<<','<<c.geom2<<','<<c.link1<<','<<c.link2<<','<<c.distance<<','<<c.point.x<<','<<c.point.y<<','<<c.point.z
                    <<','<<c.normal.x<<','<<c.normal.y<<','<<c.normal.z;
                if(c.force) contacts<<','<<c.force->x<<','<<c.force->y<<','<<c.force->z; else contacts<<",NA,NA,NA";
                contacts<<'\n';
            }
            if(!state||!joints||!contacts) throw std::runtime_error("CSV write failed");
        }
    };
    inline void metadata(const std::filesystem::path& dir,const BodyRig& rig,const PhysicsWorld& world)
    {
        std::ofstream info{dir/"calibration.txt"}; writeBodyCalibration(info,rig,world);
        std::ofstream xml{dir/"model.xml"}; xml<<world.modelXml();
    }
}
