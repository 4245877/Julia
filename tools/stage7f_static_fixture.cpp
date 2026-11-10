// Independent MuJoCo fixture: no Julia library/observer headers or linkage.
#include <mujoco/mujoco.h>
#include <array>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>

int main(int argc,char** argv)
{
    try
    {
        if(argc!=2) throw std::runtime_error("Expected new output directory");
        const std::filesystem::path out=argv[1]; if(!std::filesystem::create_directories(out)) throw std::runtime_error("Output already exists");
        std::ofstream report{out/"static-fixture.txt"}; report<<std::setprecision(17);
        std::array<double,3> maximum{}; unsigned variant=0;
        for(unsigned sub:{16u,32u,64u})
        {
            const double h=1./(120*sub); std::ostringstream xml; xml<<std::setprecision(17);
            xml<<R"(<mujoco><compiler inertiafromgeom="false" fusestatic="false" alignfree="false"/><option integrator="implicitfast" timestep=")"<<h<<R"(" gravity="0 -9.81 0" solver="Newton" iterations="50" tolerance="1e-10" cone="elliptic"/><default><geom friction=".6 .005 .0001" condim="3" solref=".01 1"/></default><worldbody><geom name="floor" type="plane" size="3 3 .1" quat=".7071067811865476 -.7071067811865476 0 0"/><body name="root" pos="0 .051 0"><freejoint align="false"/><inertial pos="0 .12 0" mass="53.4" diaginertia=".6 .6 .6"/><geom type="sphere" size=".04" pos="0 .12 0"/><body pos="-.2 0 0"><inertial pos="0 0 0" mass=".8" diaginertia=".006666666666666667 .008666666666666666 .003333333333333334"/><geom name="left" type="box" size=".1 .05 .15"/></body><body pos=".2 0 0"><inertial pos="0 0 0" mass=".8" diaginertia=".006666666666666667 .008666666666666666 .003333333333333334"/><geom name="right" type="box" size=".1 .05 .15"/></body></body></worldbody></mujoco>)";
            mjVFS vfs; mj_defaultVFS(&vfs); const auto source=xml.str(); mj_addBufferVFS(&vfs,"fixture.xml",source.data(),static_cast<int>(source.size()));
            char error[2048]{}; std::unique_ptr<mjModel,decltype(&mj_deleteModel)> m{mj_loadXML("fixture.xml",&vfs,error,sizeof(error)),mj_deleteModel}; mj_deleteVFS(&vfs);
            if(!m) throw std::runtime_error(error);
            std::unique_ptr<mjData,decltype(&mj_deleteData)> d{mj_makeData(m.get()),mj_deleteData};
            const int floor=mj_name2id(m.get(),mjOBJ_GEOM,"floor"),left=mj_name2id(m.get(),mjOBJ_GEOM,"left"),right=mj_name2id(m.get(),mjOBJ_GEOM,"right");
            const auto force=[&]() {
                std::array<double,5> f{}; f[1]=-55*9.81;
                for(int i=0;i<d->ncon;++i)
                {
                    const auto& c=d->contact[i]; if(c.geom[0]!=floor&&c.geom[1]!=floor) continue;
                    mjtNum cf[6]{}; mj_contactForce(m.get(),d.get(),i,cf); const double sign=c.geom[0]==floor?1.:-1.;
                    for(int axis=0;axis<3;++axis) for(int k=0;k<3;++k) f[static_cast<std::size_t>(axis)]+=sign*c.frame[k*3+axis]*cf[k];
                    const int body=c.geom[0]==floor?c.geom[1]:c.geom[0];
                    if(body==left) f[3]+=cf[0]; if(body==right) f[4]+=cf[0];
                }
                return f;
            };
            mj_forward(m.get(),d.get()); auto before=force(); std::array<double,3> impulse{};
            std::ofstream csv{out/("static"+std::to_string(sub)+".csv")}; csv<<"time_s,dt_s,fx_N,fy_N,fz_N,px,py,pz,Jx,Jy,Jz,residual,left_N,right_N\n"<<std::setprecision(17);
            for(unsigned step=0;step<360*sub;++step)
            {
                mj_step(m.get(),d.get()); mj_forward(m.get(),d.get()); const auto after=force(); std::array<double,3> p{};
                for(int body=1;body<m->nbody;++body) { mjtNum velocity[6]{}; mj_objectVelocity(m.get(),d.get(),mjOBJ_BODY,body,velocity,0); for(int axis=0;axis<3;++axis) p[static_cast<std::size_t>(axis)]+=m->body_mass[body]*velocity[axis+3]; }
                double residual=0;
                for(std::size_t axis=0;axis<3;++axis) { impulse[axis]+=.5*h*(before[axis]+after[axis]); residual+=(p[axis]-impulse[axis])*(p[axis]-impulse[axis]); }
                residual=std::sqrt(residual); maximum[variant]=std::max(maximum[variant],residual);
                csv<<d->time<<','<<h; for(std::size_t k=0;k<3;++k) csv<<','<<after[k]; for(auto x:p) csv<<','<<x; for(auto x:impulse) csv<<','<<x; csv<<','<<residual<<','<<after[3]<<','<<after[4]<<'\n'; before=after;
            }
            if(std::abs(before[3]-269.775)>.05||std::abs(before[4]-269.775)>.05||std::abs(before[3]+before[4]-539.55)>.01) throw std::runtime_error("Known static load failed");
            report<<"substeps="<<sub<<" left_N="<<before[3]<<" right_N="<<before[4]<<" max_residual_Ns="<<maximum[variant]<<'\n'; ++variant;
            std::ofstream model{out/("static"+std::to_string(sub)+".xml")}; model<<source;
        }
        if(maximum[2]>=maximum[0]) throw std::runtime_error("Static integral convergence failed");
        report<<"PASS independent engine-only fixture, known weight/split and convergence\n"; return 0;
    }
    catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
