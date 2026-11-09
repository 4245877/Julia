#include "julia/physics/PhysicsWorld.hpp"
#include <mujoco/mujoco.h>
#include <glm/gtc/quaternion.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace julia::physics
{
    static_assert(mjVERSION_HEADER==337,"This integration requires MuJoCo 3.3.7 headers");
    namespace
    {
        using Clock=std::chrono::steady_clock;
        double us(Clock::time_point a) { return std::chrono::duration<double,std::micro>(Clock::now()-a).count(); }
        bool finite(glm::dvec3 v) { return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z); }
        bool rigid(const glm::dmat4& m)
        {
            for(int c=0;c<4;++c) for(int r=0;r<4;++r) if(!std::isfinite(m[c][r])) return false;
            const glm::dmat3 rot{m}, gram=glm::transpose(rot)*rot;
            for(int c=0;c<3;++c) for(int r=0;r<3;++r)
                if(std::abs(gram[c][r]-(c==r?1.:0.))>1e-5) return false;
            return std::abs(glm::determinant(rot)-1)<1e-5 &&
                glm::length(glm::dvec4{m[0][3],m[1][3],m[2][3],m[3][3]-1})<1e-8;
        }
        void vec(std::ostream& out,glm::dvec3 v) { out<<v.x<<' '<<v.y<<' '<<v.z; }
        void pose(std::ostream& out,const glm::dmat4& m)
        {
            out<<" pos=\""; vec(out,glm::dvec3{m[3]});
            const auto q=glm::normalize(glm::quat_cast(glm::dmat3{m}));
            out<<"\" quat=\""<<q.w<<' '<<q.x<<' '<<q.y<<' '<<q.z<<'"';
        }
        unsigned validate(const Definition& def)
        {
            if(def.links.empty()||def.links.size()>4||!finite(def.gravity)||glm::length(def.gravity)>100)
                throw std::invalid_argument("Invalid physics links/gravity");
            int next=0;
            for(const auto& b:def.links)
            {
                if(b.name.empty()||!rigid(b.parentFromJoint)||!rigid(b.jointFromSegment)||
                    !finite(b.comInSegment)||!finite(b.inertia)||!finite(b.capsuleStart)||!finite(b.capsuleEnd)||
                    !std::isfinite(b.mass)||b.mass<=0||b.mass>100||!std::isfinite(b.radius)||b.radius<=0||b.radius>1||
                    glm::any(glm::lessThanEqual(b.inertia,glm::dvec3{1e-9}))||
                    b.inertia.x>b.inertia.y+b.inertia.z+1e-12||b.inertia.y>b.inertia.x+b.inertia.z+1e-12||
                    b.inertia.z>b.inertia.x+b.inertia.y+1e-12)
                    throw std::invalid_argument("Invalid rigid frame, mass, COM, inertia or collider: "+b.name);
                const auto& h=b.hinge;
                if(h.coordinate != -1)
                {
                    if(h.coordinate!=next||next>=2||!finite(h.axis)||std::abs(glm::length(h.axis)-1)>1e-5||
                        !std::isfinite(h.lower)||!std::isfinite(h.upper)||h.lower>=h.upper||h.lower>0||h.upper<0||
                        !std::isfinite(h.damping)||h.damping<0)
                        throw std::invalid_argument("Invalid hinge/order: "+b.name);
                    ++next;
                }
            }
            if(next==0) throw std::invalid_argument("No physical coordinates");
            for(const auto& d:def.drives)
                if(!std::isfinite(d.kp)||!std::isfinite(d.kd)||!std::isfinite(d.maxTorque)||
                    d.kp<0||d.kp>1000||d.kd<0||d.kd>100||d.maxTorque<=0||d.maxTorque>100)
                    throw std::invalid_argument("Invalid drive parameters");
            if(def.wall&&(!finite(def.wall->center)||!finite(def.wall->halfSize)||
                glm::any(glm::lessThanEqual(def.wall->halfSize,glm::dvec3{0}))))
                throw std::invalid_argument("Invalid wall");
            return static_cast<unsigned>(next);
        }
        std::string xml(const Definition& def)
        {
            std::ostringstream o; o.imbue(std::locale::classic()); o<<std::setprecision(17);
            o<<"<mujoco model=\"Julia fixed arm 7A\"><compiler angle=\"radian\" inertiafromgeom=\"false\" fusestatic=\"false\"/>"
                "<size memory=\"2M\"/><option timestep=\""<<TickSeconds/Substeps<<"\" gravity=\"";
            vec(o,def.gravity);
            o<<"\" integrator=\"implicitfast\" solver=\"Newton\" iterations=\"50\" tolerance=\"1e-10\" cone=\"elliptic\">"
                "<flag energy=\"enable\"/></option><default><geom contype=\"1\" conaffinity=\"2\" "
                "friction=\"0.6 0.005 0.0001\" condim=\"3\" solref=\"0.01 1\"/>"
                "<joint solreflimit=\"0.01 1\"/></default><worldbody>";
            if(def.wall)
            {
                o<<"<geom name=\"wall\" type=\"box\" contype=\"2\" conaffinity=\"1\" pos=\"";
                vec(o,def.wall->center); o<<"\" size=\""; vec(o,def.wall->halfSize); o<<"\"/>";
            }
            for(std::size_t i=0;i<def.links.size();++i)
            {
                const auto& b=def.links[i];
                o<<"<body name=\"b"<<i<<'"'; pose(o,b.parentFromJoint); o<<'>';
                if(b.hinge.coordinate>=0)
                {
                    o<<"<joint name=\"j"<<b.hinge.coordinate<<"\" type=\"hinge\" axis=\""; vec(o,b.hinge.axis);
                    o<<"\" limited=\"true\" range=\""<<b.hinge.lower<<' '<<b.hinge.upper<<"\" damping=\""<<b.hinge.damping<<"\"/>";
                }
                auto inertiaPose=b.jointFromSegment;
                inertiaPose[3]=b.jointFromSegment*glm::dvec4{b.comInSegment,1};
                o<<"<inertial"; pose(o,inertiaPose); o<<" mass=\""<<b.mass<<"\" diaginertia=\"";
                vec(o,b.inertia); o<<"\"/><geom name=\"g"<<i<<"\" size=\""<<b.radius<<'"';
                if(glm::length(b.capsuleEnd-b.capsuleStart)<1e-9)
                { o<<" type=\"sphere\" pos=\""; vec(o,b.capsuleStart); }
                else { o<<" type=\"capsule\" fromto=\""; vec(o,b.capsuleStart); o<<' '; vec(o,b.capsuleEnd); }
                o<<"\"/>";
            }
            for(std::size_t i=0;i<def.links.size();++i) o<<"</body>";
            o<<"</worldbody><actuator>";
            for(const auto& b:def.links) if(b.hinge.coordinate>=0)
            {
                const auto k=static_cast<std::size_t>(b.hinge.coordinate);
                const auto limit=def.drives[k].maxTorque;
                o<<"<motor name=\"motor"<<k<<"\" joint=\"j"<<k<<"\" gear=\"1\" ctrllimited=\"true\" ctrlrange=\""
                    <<-limit<<' '<<limit<<"\" forcelimited=\"true\" forcerange=\""<<-limit<<' '<<limit<<"\"/>";
            }
            o<<"</actuator></mujoco>";
            return o.str();
        }
        glm::dmat4 bodyFrame(const mjData* d,int body)
        {
            glm::dmat4 m{1};
            for(int c=0;c<3;++c) for(int r=0;r<3;++r) m[c][r]=d->xmat[9*body+3*r+c];
            m[3]=glm::dvec4{d->xpos[3*body],d->xpos[3*body+1],d->xpos[3*body+2],1};
            return m;
        }
    }
    struct PhysicsWorld::Impl
    {
        Definition def;
        std::unique_ptr<mjModel,decltype(&mj_deleteModel)> m{nullptr,mj_deleteModel};
        std::unique_ptr<mjData,decltype(&mj_deleteData)> d{nullptr,mj_deleteData};
        std::unique_ptr<mjData,decltype(&mj_deleteData)> scratch{nullptr,mj_deleteData};
        std::string source;
        Snapshot state;
        Coordinates effort{};
        std::vector<int> bodies;
        std::array<Hinge,2> hinges;
        std::vector<double> wrench;
        bool failed=false;
        explicit Impl(Definition definition):def(std::move(definition))
        {
            state.dofs=validate(def);
            if(mj_version()!=337) throw std::runtime_error("MuJoCo SDK/DLL version mismatch (expected 3.3.7)");
            source=xml(def);
            mjVFS vfs; mj_defaultVFS(&vfs);
            const auto added=mj_addBufferVFS(&vfs,"arm.xml",source.data(),static_cast<int>(source.size()));
            char error[2048]{};
            if(added==0) m.reset(mj_loadXML("arm.xml",&vfs,error,sizeof(error)));
            mj_deleteVFS(&vfs);
            if(!m) throw std::runtime_error(std::string{"MuJoCo compile: "}+error);
            if(m->nq!=static_cast<int>(state.dofs)||m->nv!=static_cast<int>(state.dofs)||m->nu!=static_cast<int>(state.dofs))
                throw std::runtime_error("Unexpected MuJoCo model dimensions");
            d.reset(mj_makeData(m.get())); scratch.reset(mj_makeData(m.get()));
            if(!d||!scratch) throw std::runtime_error("MuJoCo data allocation failed");
            for(std::size_t i=0;i<def.links.size();++i)
            {
                bodies.push_back(mj_name2id(m.get(),mjOBJ_BODY,("b"+std::to_string(i)).c_str()));
                if(def.links[i].hinge.coordinate>=0) hinges[static_cast<std::size_t>(def.links[i].hinge.coordinate)]=def.links[i].hinge;
            }
            wrench.resize(static_cast<std::size_t>(m->nbody)*6);
        }
        [[noreturn]] void fail(const std::string& why)
        {
            failed=true;
            std::ostringstream o; o<<why<<" tick="<<state.tick<<" steps="<<state.worldSteps
                <<" q="<<d->qpos[0]<<','<<(state.dofs>1?d->qpos[1]:0);
            throw std::runtime_error(o.str());
        }
        void read()
        {
            for(unsigned k=0;k<state.dofs;++k)
            {
                state.q[k]=d->qpos[k]; state.v[k]=d->qvel[k];
                state.appliedTorque[k]=d->qfrc_actuator[k];
                state.limitViolation[k]=std::max({0.,hinges[k].lower-state.q[k],state.q[k]-hinges[k].upper});
                if(!std::isfinite(state.q[k])||!std::isfinite(state.v[k])||!std::isfinite(d->qacc[k])||
                    !std::isfinite(state.appliedTorque[k])||std::abs(state.v[k])>50||state.limitViolation[k]>.04)
                    fail("Physical state/velocity/soft-limit threshold exceeded");
                if(std::abs(state.appliedTorque[k])>def.drives[k].maxTorque+1e-9) fail("Actuator effort limit exceeded");
            }
            state.worldFromJoint.clear(); state.comWorld.clear();
            for(const auto id:bodies)
            {
                state.worldFromJoint.push_back(bodyFrame(d.get(),id));
                state.comWorld.emplace_back(d->xipos[3*id],d->xipos[3*id+1],d->xipos[3*id+2]);
            }
            state.contacts.clear();
            if(d->ncon>64) fail("Contact count threshold exceeded");
            for(int i=0;i<d->ncon;++i)
            {
                const auto& c=d->contact[i]; mjtNum f[6]{}; mj_contactForce(m.get(),d.get(),i,f);
                Contact contact; contact.geom1=c.geom[0]; contact.geom2=c.geom[1];
                contact.point={c.pos[0],c.pos[1],c.pos[2]}; contact.normal={c.frame[0],c.frame[1],c.frame[2]};
                contact.distance=c.dist; contact.force=glm::dvec3{f[0],f[1],f[2]};
                if(!finite(*contact.force)||glm::length(*contact.force)>10000||c.dist<-.015) fail("Contact force/penetration threshold exceeded");
                state.peakNormalForceThisTick=std::max(state.peakNormalForceThisTick,f[0]);
                state.maxPenetrationThisTick=std::max(state.maxPenetrationThisTick,-c.dist);
                state.contacts.push_back(contact);
            }
            state.peakContactsThisTick=std::max(state.peakContactsThisTick,static_cast<unsigned>(d->ncon));
            state.solverIterations=0;
            for(int i=0;i<d->nisland;++i) state.solverIterations+=static_cast<unsigned>(d->solver_niter[i]);
            state.warnings=0;
            for(const auto& warning:d->warning) state.warnings+=static_cast<unsigned>(warning.number);
            if(state.warnings) fail("MuJoCo solver warning (including automatic instability reset)");
            state.potentialEnergy=d->energy[0]; state.kineticEnergy=d->energy[1];
            if(!std::isfinite(state.potentialEnergy)||!std::isfinite(state.kineticEnergy)||state.kineticEnergy>2000)
                fail("Energy threshold exceeded");
            state.time=d->time;
        }
    };
    PhysicsWorld::PhysicsWorld(Definition def):impl_(std::make_unique<Impl>(std::move(def))) { reset(); }
    PhysicsWorld::~PhysicsWorld()=default;
    const Definition& PhysicsWorld::definition() const { return impl_->def; }
    const Snapshot& PhysicsWorld::snapshot() const { return impl_->state; }
    std::string PhysicsWorld::backendVersion() { return mj_versionString(); }
    void PhysicsWorld::reset(Coordinates q)
    {
        auto& p=*impl_;
        for(unsigned k=0;k<2;++k)
            if(!std::isfinite(q[k])||(k>=p.state.dofs?q[k]!=0:(q[k]<p.hinges[k].lower||q[k]>p.hinges[k].upper)))
                throw std::invalid_argument("Reset requires finite, in-range coordinates");
        const auto epoch=p.state.epoch+1; const auto dofs=p.state.dofs;
        mj_resetData(p.m.get(),p.d.get()); mj_resetData(p.m.get(),p.scratch.get());
        p.state={}; p.state.epoch=epoch; p.state.dofs=dofs; p.state.target=q;
        p.effort={}; p.failed=false; std::fill(p.wrench.begin(),p.wrench.end(),0);
        for(unsigned k=0;k<dofs;++k) p.d->qpos[k]=q[k]; // ONLY explicit reset writes live q/v.
        mj_forward(p.m.get(),p.d.get()); p.read();
    }
    bool PhysicsWorld::command(CommandKind kind,Coordinates values,std::string* reason)
    {
        auto& p=*impl_;
        const auto reject=[&](const std::string& s) { if(reason) *reason=s; return false; };
        if(p.failed) return reject("Physics failed; reset required");
        if(kind!=CommandKind::Target&&kind!=CommandKind::Torque&&kind!=CommandKind::Disable)
            return reject("Unsupported command in fixed-base effort bench (no IK/attachment/root motion)");
        for(unsigned k=0;k<2;++k)
        {
            if(!std::isfinite(values[k])||(k>=p.state.dofs&&values[k]!=0)) return reject("Invalid coordinates");
            if(kind==CommandKind::Target&&k<p.state.dofs&&(values[k]<p.hinges[k].lower||values[k]>p.hinges[k].upper))
                return reject("Target outside calibrated limits");
        }
        if(kind==CommandKind::Target) { p.state.target=values; p.state.mode=DriveMode::Position; }
        else if(kind==CommandKind::Torque) { p.effort=values; p.state.mode=DriveMode::Torque; }
        else p.state.mode=DriveMode::Disabled;
        if(reason) reason->clear(); return true;
    }
    void PhysicsWorld::applyWrench(unsigned link,glm::dvec3 force,glm::dvec3 torque)
    {
        auto& p=*impl_;
        if(link>=p.bodies.size()||!finite(force)||!finite(torque)||glm::length(force)>500||glm::length(torque)>100)
            throw std::invalid_argument("Invalid external wrench");
        const auto offset=static_cast<std::size_t>(p.bodies[link])*6;
        for(int k=0;k<3;++k) { p.wrench[offset+k]=force[k]; p.wrench[offset+3+k]=torque[k]; }
    }
    void PhysicsWorld::step()
    {
        auto& p=*impl_;
        if(p.failed) throw std::runtime_error("Physics failed; reset required");
        auto& s=p.state; ++s.tick; s.timing={}; s.saturatedThisTick={};
        s.peakContactsThisTick=0; s.peakNormalForceThisTick=0; s.maxPenetrationThisTick=0;
        s.externalForce={0,0,0}; s.externalTorque={0,0,0};
        std::copy(p.wrench.begin(),p.wrench.end(),p.d->xfrc_applied);
        for(std::size_t b=0;b<p.wrench.size();b+=6)
        { s.externalForce+=glm::dvec3{p.wrench[b],p.wrench[b+1],p.wrench[b+2]}; s.externalTorque+=glm::dvec3{p.wrench[b+3],p.wrench[b+4],p.wrench[b+5]}; }
        for(unsigned sub=0;sub<Substeps;++sub)
        {
            auto start=Clock::now();
            for(unsigned k=0;k<s.dofs;++k)
            {
                const auto& drive=p.def.drives[k];
                const double calculated=s.mode==DriveMode::Position ? drive.kp*(s.target[k]-p.d->qpos[k])-drive.kd*p.d->qvel[k] :
                    (s.mode==DriveMode::Torque?p.effort[k]:0);
                s.calculatedTorque[k]=calculated;
                p.d->ctrl[k]=std::clamp(calculated,-drive.maxTorque,drive.maxTorque);
                s.saturated[k]=std::abs(calculated)>drive.maxTorque;
                s.saturatedThisTick[k]=s.saturatedThisTick[k]||s.saturated[k];
            }
            s.timing.controlUs+=us(start); start=Clock::now();
            mj_step(p.m.get(),p.d.get()); ++s.worldSteps;
            s.timing.stepUs+=us(start); start=Clock::now();
            // mj_step leaves derived transforms at pre-integration q. Refresh ALL
            // endpoint observations with the last held ctrl; never alter live q/v.
            mj_forward(p.m.get(),p.d.get()); p.read();
            s.timing.readUs+=us(start);
            const double expected=static_cast<double>(s.worldSteps)*(TickSeconds/Substeps);
            if(std::abs(s.time-expected)>1e-8) p.fail("Physics time discontinuity");
        }
        std::fill(p.wrench.begin(),p.wrench.end(),0);
        mju_zero(p.d->xfrc_applied,static_cast<int>(p.wrench.size()));
    }
    std::vector<glm::dmat4> PhysicsWorld::forwardKinematics(Coordinates q) const
    {
        auto& p=*impl_;
        if(!std::isfinite(q[0])||!std::isfinite(q[1])) throw std::invalid_argument("Non-finite FK coordinates");
        mj_resetData(p.m.get(),p.scratch.get());
        for(unsigned k=0;k<p.state.dofs;++k) p.scratch->qpos[k]=q[k];
        mj_kinematics(p.m.get(),p.scratch.get());
        std::vector<glm::dmat4> result;
        for(const auto id:p.bodies) result.push_back(bodyFrame(p.scratch.get(),id));
        return result;
    }
    std::size_t PhysicsWorld::modelBytes() const { return sizeof(mjModel)+static_cast<std::size_t>(impl_->m->nbuffer); }
    std::size_t PhysicsWorld::dataBytes() const { return sizeof(mjData)+static_cast<std::size_t>(impl_->d->nbuffer+impl_->d->narena); }
    std::size_t PhysicsWorld::arenaHighWaterBytes() const { return static_cast<std::size_t>(impl_->d->maxuse_arena); }
    std::string PhysicsWorld::modelXml() const { return impl_->source; }
}
