#pragma once
#include "julia/physics/ArmRig.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>

namespace bench
{
    struct Options
    {
        julia::physics::BenchCase scenario=julia::physics::BenchCase::Hold;
        std::uint64_t ticks=720;
        unsigned fps=60;
        bool verify=false, explicitTicks=false;
        std::filesystem::path output="build/diagnostics/stage7a/run";
    };
    inline Options parse(int argc,char** argv)
    {
        Options o;
        for(int i=1;i<argc;++i)
        {
            const std::string flag=argv[i];
            const auto value=[&]() { if(++i>=argc) throw std::invalid_argument("Missing value for "+flag); return std::string{argv[i]}; };
            if(flag=="--case") o.scenario=julia::physics::parseBenchCase(value());
            else if(flag=="--output") o.output=value();
            else if(flag=="--verify") { o.verify=true; o.output=value(); }
            else if(flag=="--ticks"||flag=="--fps")
            {
                const auto str=value(); std::size_t used=0;
                if(str.empty()||str[0]=='-') throw std::invalid_argument("Invalid unsigned count");
                const auto n=std::stoull(str,&used);
                if(used!=str.size()||n==0||n>1200000) throw std::invalid_argument("Invalid count");
                if(flag=="--ticks") { o.ticks=n; o.explicitTicks=true; }
                else { if(n!=30&&n!=60&&n!=144) throw std::invalid_argument("FPS must be 30, 60 or 144"); o.fps=static_cast<unsigned>(n); }
            }
            else throw std::invalid_argument("Unknown option "+flag);
        }
        return o;
    }
    struct Log
    {
        std::ofstream csv, links, contacts;
        explicit Log(const std::filesystem::path& dir)
        {
            std::filesystem::create_directories(dir);
            csv.open(dir/"state.csv"); links.open(dir/"links.csv"); contacts.open(dir/"contacts.csv");
            if(!csv||!links||!contacts) throw std::runtime_error("Cannot create bench CSV");
            julia::physics::csvHeader(csv);
            links<<"tick,epoch,link,mass_kg,com_x_m,com_y_m,com_z_m,Ixx_kg_m2,Iyy_kg_m2,Izz_kg_m2\n";
            contacts<<"tick,epoch,geom1,geom2,distance_m,x_m,y_m,z_m,nx,ny,nz,normal_N,tangent1_N,tangent2_N\n";
        }
        void row(const julia::physics::RenderSnapshot& view,const julia::physics::Definition& def)
        {
            julia::physics::csvRow(csv,view);
            links<<std::setprecision(17); contacts<<std::setprecision(17);
            const auto& s=view.physics;
            for(std::size_t i=0;i<def.links.size();++i)
            {
                const auto& b=def.links[i]; const auto& c=s.comWorld[i];
                links<<s.tick<<','<<s.epoch<<','<<i<<','<<b.mass<<','<<c.x<<','<<c.y<<','<<c.z<<','<<b.inertia.x<<','<<b.inertia.y<<','<<b.inertia.z<<'\n';
            }
            for(const auto& c:s.contacts)
            {
                contacts<<s.tick<<','<<s.epoch<<','<<c.geom1<<','<<c.geom2<<','<<c.distance<<','<<c.point.x<<','<<c.point.y<<','<<c.point.z
                    <<','<<c.normal.x<<','<<c.normal.y<<','<<c.normal.z;
                if(c.force) contacts<<','<<c.force->x<<','<<c.force->y<<','<<c.force->z;
                else contacts<<",NA,NA,NA";
                contacts<<'\n';
            }
            if(!csv||!links||!contacts) throw std::runtime_error("Bench CSV write failed");
        }
    };
    inline void metadata(const std::filesystem::path& dir,const julia::physics::ArmRig& rig,const julia::physics::PhysicsWorld& world)
    {
        std::ofstream info{dir/"calibration.txt"}; julia::physics::writeCalibration(info,rig,world);
        std::ofstream xml{dir/"model.xml"}; xml<<world.modelXml();
    }
}
