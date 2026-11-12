#include "julia/physics/PhysicsWorld.hpp"
#include "StandControl.hpp"
#include <mujoco/mujoco.h>
#include <glm/gtc/quaternion.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <functional>
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
        bool comCommandSupports(const Snapshot& s)
        {
            const auto& o=s.support;
            if(!o.enabled||!o.floorAvailable||!o.dynamics.valid||o.otherEnvironmentActive||!o.combinedCop.point) return false;
            for(const auto& f:o.feet) if(!f.load.loaded||f.active==0||!f.near||!f.cop.point||f.normalForce<30||f.slipSpeed>.05) return false;
            return true;
        }
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
            if(def.substeps<8||def.substeps>64||def.links.empty()||def.links.size()>64||!finite(def.gravity)||glm::length(def.gravity)>100||
                !std::isfinite(def.contactTimeConstant)||def.contactTimeConstant<2*TickSeconds/def.substeps||def.contactTimeConstant>.1)
                throw std::invalid_argument("Invalid physics links/gravity");
            int next=0;
            for(std::size_t index=0;index<def.links.size();++index)
            {
                const auto& b=def.links[index];
                if(b.parent < -2 || b.parent>=static_cast<int>(index)) throw std::invalid_argument("Invalid tree parent");
                if(index>0 && b.parent==-1 && def.freeRoot) throw std::invalid_argument("Free tree must have one root");
                if(index==0 && def.freeRoot && !b.hinges.empty()) throw std::invalid_argument("Free root cannot contain hinges");
                if(b.boxHalfSize && (!finite(*b.boxHalfSize)||glm::any(glm::lessThanEqual(*b.boxHalfSize,glm::dvec3{0}))||!rigid(b.jointFromBox))) throw std::invalid_argument("Invalid box");
                if(b.name.empty()||!rigid(b.parentFromJoint)||!rigid(b.jointFromSegment)||
                    !finite(b.comInSegment)||!finite(b.inertia)||!finite(b.capsuleStart)||!finite(b.capsuleEnd)||
                    !std::isfinite(b.mass)||b.mass<=0||b.mass>100||!std::isfinite(b.radius)||b.radius<=0||b.radius>1||
                    glm::any(glm::lessThanEqual(b.inertia,glm::dvec3{1e-9}))||
                    b.inertia.x>b.inertia.y+b.inertia.z+1e-12||b.inertia.y>b.inertia.x+b.inertia.z+1e-12||
                    b.inertia.z>b.inertia.x+b.inertia.y+1e-12)
                    throw std::invalid_argument("Invalid rigid frame, mass, COM, inertia or collider: "+b.name);
                if(b.hinges.size()>3) throw std::invalid_argument("Too many rotations on body");
                for(const auto& h:b.hinges)
                {
                    if(h.coordinate!=next||next>=static_cast<int>(MaxCoordinates)||!finite(h.axis)||std::abs(glm::length(h.axis)-1)>1e-5||
                        !std::isfinite(h.lower)||!std::isfinite(h.upper)||h.lower>=h.upper||h.lower>0||h.upper<0||
                        !std::isfinite(h.damping)||h.damping<0||!std::isfinite(h.limitTimeConstant)||
                        h.limitTimeConstant<2*TickSeconds/def.substeps||h.limitTimeConstant>.1)
                        throw std::invalid_argument("Invalid hinge/order: "+b.name);
                    ++next;
                }
            }
            if(next==0 && !def.freeRoot) throw std::invalid_argument("No physical coordinates");
            if(def.supportObservation)
            {
                const auto& s=*def.supportObservation;
                for(double x:{s.confirmForce,s.lossForce,s.confirmSeconds,s.lossSeconds,s.copMinimumForce,s.nearDistance})
                    if(!std::isfinite(x)||x<=0) throw std::invalid_argument("Invalid support observation threshold");
                if(s.lossForce>=s.confirmForce||s.feet[0].segment==s.feet[1].segment||s.feet[0].geom==s.feet[1].geom)
                    throw std::invalid_argument("Invalid independent support pairs");
                for(const auto& f:s.feet) if(f.segment.empty()||f.geom.empty()||!rigid(f.jointFromSole)) throw std::invalid_argument("Invalid Sole frame/identity");
            }
            if(def.contactTool)
            {
                const auto& t=*def.contactTool;
                if(t.link>=def.links.size()||!finite(t.pointInJoint)||!finite(t.normalWorld)||
                    std::abs(glm::length(t.normalWorld)-1)>1e-8||!std::isfinite(t.radius)||t.radius<=0||t.radius>.1||
                    (t.surface=="wall"&&std::abs(std::abs(t.normalWorld.x)+std::abs(t.normalWorld.y)+std::abs(t.normalWorld.z)-1)>1e-8)||
                    (t.surface=="floor"&&glm::length(t.normalWorld-glm::dvec3{0,1,0})>1e-8)||
                    !std::isfinite(t.friction)||t.friction<.01||t.friction>1||
                    (t.surface!="wall"&&t.surface!="floor")||(t.surface=="wall"&&!def.wall)||(t.surface=="floor"&&!def.floorHeight))
                    throw std::invalid_argument("Invalid contact tool/pair");
            }
            for(const auto& d:def.drives)
                if(!std::isfinite(d.kp)||!std::isfinite(d.kd)||!std::isfinite(d.maxTorque)||
                    d.kp<0||d.kp>1000||d.kd<0||d.kd>100||d.maxTorque<=0||d.maxTorque>100)
                    throw std::invalid_argument("Invalid drive parameters");
            if(def.wall&&(!finite(def.wall->center)||!finite(def.wall->halfSize)||
                glm::any(glm::lessThanEqual(def.wall->halfSize,glm::dvec3{0}))))
                throw std::invalid_argument("Invalid wall");
            for(const auto& pair:def.exclusions)
                if(pair[0]>=def.links.size()||pair[1]>=def.links.size()||pair[0]==pair[1]) throw std::invalid_argument("Invalid contact exclusion");
            if(def.floorHeight&&!std::isfinite(*def.floorHeight)) throw std::invalid_argument("Invalid floor");
            for(double v:{def.maxJointSpeed,def.maxLimitViolation,def.maxPenetration,def.maxContactForce,def.maxKineticEnergy,def.maxRootLinearSpeed,def.maxRootAngularSpeed})
                if(!std::isfinite(v)||v<=0) throw std::invalid_argument("Invalid runtime envelope");
            if(!def.maxContacts) throw std::invalid_argument("Invalid contact count envelope");
            return static_cast<unsigned>(next);
        }
        std::string xml(const Definition& def)
        {
            std::ostringstream o; o.imbue(std::locale::classic()); o<<std::setprecision(17);
            o<<"<mujoco model=\"Julia articulated physics\"><compiler angle=\"radian\" inertiafromgeom=\"false\" fusestatic=\"false\" alignfree=\"false\"/>"
                "<size memory=\"2M\"/><option timestep=\""<<TickSeconds/def.substeps<<"\" gravity=\"";
            vec(o,def.gravity);
            o<<"\" integrator=\"implicitfast\" solver=\"Newton\" iterations=\"50\" tolerance=\"1e-10\" cone=\"elliptic\">"
                "<flag energy=\"enable\"/></option><default><geom contype=\"1\" conaffinity=\""<<(def.selfCollision?3:2)<<"\" "
                "friction=\"0.6 0.005 0.0001\" condim=\"3\" solref=\""<<def.contactTimeConstant<<" 1\"/>"
                "<joint solreflimit=\"0.01 1\"/></default><worldbody>";
            if(def.wall)
            {
                o<<"<geom name=\"wall\" type=\"box\" contype=\"2\" conaffinity=\"1\" pos=\"";
                vec(o,def.wall->center); o<<"\" size=\""; vec(o,def.wall->halfSize); o<<"\"/>";
            }
            if(def.floorHeight)
                o<<"<geom name=\"floor\" type=\"plane\" contype=\"2\" conaffinity=\"1\" pos=\"0 "<<*def.floorHeight<<" 0\" quat=\"0.7071067811865476 -0.7071067811865476 0 0\" size=\"3 3 .1\"/>";
            std::function<void(std::size_t)> emit=[&](std::size_t i)
            {
                const auto& b=def.links[i];
                o<<"<body name=\"b"<<i<<'"'; pose(o,b.parentFromJoint); o<<'>';
                if(i==0 && def.freeRoot) o<<"<freejoint name=\"root\" align=\"false\"/>";
                for(const auto& h:b.hinges)
                {
                    o<<"<joint name=\"j"<<h.coordinate<<"\" type=\"hinge\" pos=\"0 0 0\" axis=\""; vec(o,h.axis);
                    o<<"\" limited=\"true\" range=\""<<h.lower<<' '<<h.upper<<"\" damping=\""<<h.damping
                        <<"\" solreflimit=\""<<h.limitTimeConstant<<" 1\"/>";
                }
                auto inertiaPose=b.jointFromSegment;
                inertiaPose[3]=b.jointFromSegment*glm::dvec4{b.comInSegment,1};
                o<<"<inertial"; pose(o,inertiaPose); o<<" mass=\""<<b.mass<<"\" diaginertia=\"";
                vec(o,b.inertia); o<<"\"/><geom name=\"g"<<i<<'"';
                if(b.boxHalfSize)
                { o<<" type=\"box\""; pose(o,b.jointFromBox); o<<" size=\""; vec(o,*b.boxHalfSize); }
                else
                {
                    o<<" size=\""<<b.radius<<'"';
                    if(glm::length(b.capsuleEnd-b.capsuleStart)<1e-9)
                    { o<<" type=\"sphere\" pos=\""; vec(o,b.capsuleStart); }
                    else { o<<" type=\"capsule\" fromto=\""; vec(o,b.capsuleStart); o<<' '; vec(o,b.capsuleEnd); }
                }
                o<<"\"/>";
                for(std::size_t j=i+1;j<def.links.size();++j)
                {
                    const int parent=def.links[j].parent==-2?static_cast<int>(j)-1:def.links[j].parent;
                    if(parent==static_cast<int>(i)) emit(j);
                }
                o<<"</body>";
            };
            for(std::size_t i=0;i<def.links.size();++i)
                if(def.links[i].parent==-1||(i==0&&def.links[i].parent==-2)) emit(i);
            o<<"</worldbody><contact>";
            if(def.contactTool)
            {
                const auto& t=*def.contactTool;
                o<<"<pair geom1=\""<<t.surface<<"\" geom2=\"g"<<t.link<<"\" condim=\"3\" friction=\""
                    <<t.friction<<' '<<t.friction<<" 0.005 0.0001 0.0001\" solref=\""<<def.contactTimeConstant
                    <<" 1\" solimp=\"0.9 0.95 0.001 0.5 2\"/>";
            }
            for(const auto& pair:def.exclusions) o<<"<exclude body1=\"b"<<pair[0]<<"\" body2=\"b"<<pair[1]<<"\"/>";
            o<<"</contact><actuator>";

            for(const auto& b:def.links) for(const auto& h:b.hinges)
            {
                const auto k=static_cast<std::size_t>(h.coordinate);
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
        StateLayout layout;
        Coordinates effort{};
        std::array<JointReference,MaxCoordinates> references;
        std::vector<int> bodies;
        std::array<Hinge,MaxCoordinates> hinges;
        std::vector<double> wrench;
        bool failed=false;
        double initialMechanicalEnergy=0;
        ContactController contactController;
        StandControl standController;
        int toolGeom=-1,surfaceGeom=-1;
        std::array<int,2> footLinks{-1,-1},footGeoms{-1,-1};
        int floorGeom=-1;
        std::vector<glm::dmat4> footBoxes;
        std::vector<glm::dvec3> footHalfSizes;
        glm::dvec3 initialLinearMomentum{0},initialAngularMomentum{0};
        Wrench externalResultant() const
        {
            Wrench w; w.origin=state.totalComWorld;
            for(std::size_t i=0;i<bodies.size();++i)
            {
                const auto k=static_cast<std::size_t>(bodies[i])*6;
                w.add(state.comWorld[i],{wrench[k],wrench[k+1],wrench[k+2]},{wrench[k+3],wrench[k+4],wrench[k+5]});
            }
            return w;
        }
        Wrench environmentResultant() const
        {
            Wrench result; result.origin=state.totalComWorld;
            for(const auto& c:state.contacts) if((c.link1>=0)!=(c.link2>=0))
            {
                const auto w=contactWrench(c,c.link1>=0?c.geom1:c.geom2,state.totalComWorld);
                result.force+=w.force; result.moment+=w.moment;
            }
            return result;
        }
        Wrench totalResultant() const
        {
            auto w=environmentResultant(); const auto a=externalResultant();
            w.force+=state.totalMass*def.gravity+a.force; w.moment+=a.moment; return w;
        }
        void observeSupport(double h,const Wrench& before={})
        {
            if(!def.supportObservation) return;
            const auto start=Clock::now();
            auto& o=state.support;
            observeSupports(o,state,*def.supportObservation,footLinks,footGeoms,floorGeom,def.floorHeight,footBoxes,footHalfSizes,h);
            auto& a=o.dynamics; a.valid=def.freeRoot;
            a.comVelocity=state.linearMomentumWorld/state.totalMass;
            std::array<mjtNum,3*(MaxCoordinates+6)> jac{};
            mj_jacSubtreeCom(m.get(),d.get(),jac.data(),bodies[0]); a.comVelocityJacobian={0,0,0};
            for(int axis=0;axis<3;++axis) for(int j=0;j<m->nv;++j) a.comVelocityJacobian[axis]+=jac[static_cast<std::size_t>(axis*m->nv+j)]*d->qvel[j];
            a.gravity={state.totalComWorld,state.totalMass*def.gravity,{0,0,0}};
            a.environment=environmentResultant(); a.applied=externalResultant(); a.total=totalResultant();
            if(h>0)
            {
                const auto impulse=.5*h*(before.force+a.total.force),angular=.5*h*(before.moment+a.total.moment);
                a.linearImpulse+=impulse; a.angularImpulse+=angular; a.tickLinearImpulse+=impulse; a.tickAngularImpulse+=angular;
                a.linearImpulseEndpoint+=h*a.total.force; a.angularImpulseEndpoint+=h*a.total.moment; ++a.samples;
                a.substeps.push_back({state.time,h,state.totalComWorld,state.linearMomentumWorld,state.angularMomentumAboutComWorld,before.force,before.moment,a.total.force,a.total.moment});
            }
            a.deltaLinearMomentum=state.linearMomentumWorld-initialLinearMomentum;
            a.deltaAngularMomentum=state.angularMomentumAboutComWorld-initialAngularMomentum;
            a.linearResidual=a.deltaLinearMomentum-a.linearImpulse; a.angularResidual=a.deltaAngularMomentum-a.angularImpulse;
            state.timing.supportUs+=us(start);
        }
        glm::dvec3 pointVelocity(int body,glm::dvec3 point,glm::dvec3* angular=nullptr) const
        {
            if(body==0) { if(angular) *angular={0,0,0}; return {0,0,0}; }
            mjtNum v[6]{}; mj_objectVelocity(m.get(),d.get(),mjOBJ_BODY,body,v,0);
            const glm::dvec3 w{v[0],v[1],v[2]},linear{v[3],v[4],v[5]};
            const glm::dvec3 com{d->xipos[3*body],d->xipos[3*body+1],d->xipos[3*body+2]};
            if(angular) *angular=w;
            return linear+glm::cross(w,point-com);
        }
        ContactInput contactInput(bool jacobian=false) const
        {
            ContactInput in; in.time=d->time; in.q=state.q; in.v=state.v;
            if(!def.contactTool) return in;
            const auto& t=*def.contactTool;
            in.worldFromTool=state.worldFromJoint[t.link];
            in.worldFromTool[3]=state.worldFromJoint[t.link]*glm::dvec4{t.pointInJoint,1};
            in.point=glm::dvec3{in.worldFromTool[3]};
            in.velocity=pointVelocity(bodies[t.link],in.point);
            in.pair=aggregateContacts(state.contacts,toolGeom,surfaceGeom,t.normalWorld);
            if(!in.pair.records) in.pair.friction=m->pair_friction[0];
            const auto surfacePoint=t.surface=="wall"?def.wall->center+def.wall->halfSize*t.normalWorld:
                glm::dvec3{0,*def.floorHeight,0};
            in.pair.geometricGap=glm::dot(in.point-surfacePoint,t.normalWorld)-t.radius;
            in.pair.near=std::abs(in.pair.geometricGap)<=.002;
            if(t.surface=="wall") for(int axis=0;axis<3;++axis)
                if(std::abs(t.normalWorld[axis])<.5&&std::abs(in.point[axis]-def.wall->center[axis])>def.wall->halfSize[axis]+t.radius)
                    in.pair.near=false;
            if(jacobian)
            {
                // Fixed maximum nv (44 hinges + 6 root), no per-sample allocation.
                std::array<mjtNum,3*(MaxCoordinates+6)> j{};
                const mjtNum point[3]{in.point.x,in.point.y,in.point.z};
                mj_jac(m.get(),d.get(),j.data(),nullptr,point,bodies[t.link]);
                for(unsigned k=0;k<state.dofs;++k) for(int axis=0;axis<3;++axis)
                    in.jacobian[k][axis]=j[static_cast<std::size_t>(axis*m->nv+layout.qvel[k])];
            }
            return in;
        }
        double wrenchPower() const
        {
            double power=0;
            for(const auto id:bodies)
            {
                const auto offset=static_cast<std::size_t>(id)*6;
                if(std::all_of(wrench.begin()+static_cast<std::ptrdiff_t>(offset),wrench.begin()+static_cast<std::ptrdiff_t>(offset+6),[](double x) { return x==0; })) continue;
                mjtNum velocity[6]{}; mj_objectVelocity(m.get(),d.get(),mjOBJ_BODY,id,velocity,0);
                for(unsigned k=0;k<3;++k) power+=wrench[offset+k]*velocity[k+3]+wrench[offset+k+3]*velocity[k];
            }
            return power;
        }
        Coordinates gravity(const mjtNum* qpos)
        {
            // Only scratch is changed. Zero qvel removes Coriolis/centrifugal
            // terms; RNE(acc=0) includes gravity but no passive/contact/wrench.
            mj_resetData(m.get(),scratch.get());
            mju_copy(scratch->qpos,qpos,m->nq);
            mj_kinematics(m.get(),scratch.get()); mj_comPos(m.get(),scratch.get());
            mj_comVel(m.get(),scratch.get());
            mj_rne(m.get(),scratch.get(),0,scratch->qfrc_bias);
            Coordinates result{};
            for(unsigned k=0;k<state.dofs;++k) result[k]=scratch->qfrc_bias[layout.qvel[k]];
            return result;
        }
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
            if(m->nq!=static_cast<int>(state.dofs)+(def.freeRoot?7:0)||m->nv!=static_cast<int>(state.dofs)+(def.freeRoot?6:0)||m->nu!=static_cast<int>(state.dofs))
                throw std::runtime_error("Unexpected MuJoCo model dimensions");
            d.reset(mj_makeData(m.get())); scratch.reset(mj_makeData(m.get()));
            if(!d||!scratch) throw std::runtime_error("MuJoCo data allocation failed");
            for(std::size_t i=0;i<def.links.size();++i)
            {
                bodies.push_back(mj_name2id(m.get(),mjOBJ_BODY,("b"+std::to_string(i)).c_str()));
                for(const auto& h:def.links[i].hinges) hinges[static_cast<std::size_t>(h.coordinate)]=h;
            }
            layout.nq=static_cast<unsigned>(m->nq); layout.nv=static_cast<unsigned>(m->nv); layout.nu=static_cast<unsigned>(m->nu);
            if(def.freeRoot)
            {
                const int j=mj_name2id(m.get(),mjOBJ_JOINT,"root");
                layout.rootQpos=m->jnt_qposadr[j]; layout.rootQvel=m->jnt_dofadr[j];
                layout.rootQposSize=7; layout.rootQvelSize=6;
            }
            for(unsigned k=0;k<state.dofs;++k)
            {
                const auto j=mj_name2id(m.get(),mjOBJ_JOINT,("j"+std::to_string(k)).c_str());
                layout.qpos.push_back(m->jnt_qposadr[j]); layout.qvel.push_back(m->jnt_dofadr[j]);
                layout.actuator.push_back(mj_name2id(m.get(),mjOBJ_ACTUATOR,("motor"+std::to_string(k)).c_str()));
            }
            wrench.resize(static_cast<std::size_t>(m->nbody)*6);
            if(def.supportObservation)
            {
                floorGeom=mj_name2id(m.get(),mjOBJ_GEOM,"floor");
                for(std::size_t side=0;side<2;++side)
                {
                    const auto& f=def.supportObservation->feet[side];
                    const auto it=std::find_if(def.links.begin(),def.links.end(),[&](const auto& b) { return b.segment==f.segment; });
                    if(it==def.links.end()||!it->boxHalfSize||std::count_if(def.links.begin(),def.links.end(),[&](const auto& b) { return b.segment==f.segment; })!=1)
                        throw std::invalid_argument("Missing/ambiguous semantic foot box: "+f.segment);
                    footLinks[side]=static_cast<int>(it-def.links.begin());
                    footGeoms[side]=mj_name2id(m.get(),mjOBJ_GEOM,f.geom.c_str());
                    if(footGeoms[side]<0||m->geom_bodyid[footGeoms[side]]!=bodies[static_cast<std::size_t>(footLinks[side])]) throw std::invalid_argument("Compiled foot geom does not match semantic segment");
                    footBoxes.push_back(it->jointFromBox); footHalfSizes.push_back(*it->boxHalfSize);
                }
            }
            if(def.contactTool)
            {
                toolGeom=mj_name2id(m.get(),mjOBJ_GEOM,("g"+std::to_string(def.contactTool->link)).c_str());
                surfaceGeom=mj_name2id(m.get(),mjOBJ_GEOM,def.contactTool->surface.c_str());
                if(toolGeom<0||surfaceGeom<0) throw std::runtime_error("Compiled contact geometry missing");
                if(m->npair!=1||std::abs(m->pair_friction[0]-def.contactTool->friction)>1e-12||
                    std::abs(m->pair_friction[1]-def.contactTool->friction)>1e-12)
                    throw std::runtime_error("Compiled explicit contact pair friction mismatch");
            }
        }
        [[noreturn]] void fail(const std::string& why)
        {
            failed=true;
            std::ostringstream o; o<<why<<" tick="<<state.tick<<" steps="<<state.worldSteps<<" q=";
            for(unsigned k=0;k<state.dofs;++k) o<<d->qpos[layout.qpos[k]]<<',';
            if(def.freeRoot)
            {
                o<<" root_qpos_m_wxyz="; for(int k=0;k<7;++k) o<<d->qpos[layout.rootQpos+k]<<',';
                o<<" root_qvel_world_m_s_local_rad_s="; for(int k=0;k<6;++k) o<<d->qvel[layout.rootQvel+k]<<',';
            }
            o<<" contacts="<<d->ncon;
            for(int i=0;i<d->ncon;++i)
            { const auto& c=d->contact[i]; mjtNum f[6]{}; mj_contactForce(m.get(),d.get(),i,f); o<<" [geom "<<c.geom[0]<<','<<c.geom[1]<<" dist="<<c.dist<<" N="<<f[0]<<']'; }
            throw std::runtime_error(o.str());
        }
        void read()
        {
            for(unsigned k=0;k<state.dofs;++k)
            {
                state.q[k]=d->qpos[layout.qpos[k]]; state.v[k]=d->qvel[layout.qvel[k]];
                state.appliedTorque[k]=d->qfrc_actuator[layout.qvel[k]];
                state.limitViolation[k]=std::max({0.,hinges[k].lower-state.q[k],state.q[k]-hinges[k].upper});
                state.peakLimitViolationThisTick[k]=std::max(state.peakLimitViolationThisTick[k],state.limitViolation[k]);
                if(!std::isfinite(state.q[k])||!std::isfinite(state.v[k])||!std::isfinite(d->qacc[layout.qvel[k]])||
                    !std::isfinite(state.appliedTorque[k])||std::abs(state.v[k])>def.maxJointSpeed||state.limitViolation[k]>def.maxLimitViolation)
                    fail("Physical state/velocity/soft-limit threshold exceeded coordinate="+std::to_string(k)+
                        " velocity_rad_s="+std::to_string(state.v[k])+" limit_rad="+std::to_string(state.limitViolation[k]));
                if(std::abs(state.appliedTorque[k])>def.drives[k].maxTorque+1e-9) fail("Actuator effort limit exceeded");
            }
            state.worldFromJoint.clear(); state.comWorld.clear();
            for(const auto id:bodies)
            {
                state.worldFromJoint.push_back(bodyFrame(d.get(),id));
                state.comWorld.emplace_back(d->xipos[3*id],d->xipos[3*id+1],d->xipos[3*id+2]);
            }
            state.totalMass=0; state.totalComWorld={0,0,0}; state.linearMomentumWorld={0,0,0}; state.angularMomentumAboutComWorld={0,0,0};
            for(std::size_t i=0;i<bodies.size();++i)
            { state.totalMass+=m->body_mass[bodies[i]]; state.totalComWorld+=m->body_mass[bodies[i]]*state.comWorld[i]; }
            state.totalComWorld/=state.totalMass;
            for(std::size_t i=0;i<bodies.size();++i)
            {
                const int id=bodies[i]; mjtNum velocity[6]{};
                mj_objectVelocity(m.get(),d.get(),mjOBJ_BODY,id,velocity,0); // BODY: inertial COM, world axes.
                const glm::dvec3 omega{velocity[0],velocity[1],velocity[2]}, v{velocity[3],velocity[4],velocity[5]};
                const glm::dvec3 momentum=m->body_mass[id]*v;
                glm::dmat3 rotation{1}, diagonal{0};
                for(int c=0;c<3;++c) for(int r=0;r<3;++r) rotation[c][r]=d->ximat[9*id+3*r+c];
                for(int k=0;k<3;++k) diagonal[k][k]=m->body_inertia[3*id+k];
                state.linearMomentumWorld+=momentum;
                state.angularMomentumAboutComWorld+=rotation*diagonal*glm::transpose(rotation)*omega+glm::cross(state.comWorld[i]-state.totalComWorld,momentum);
            }
            if(def.freeRoot)
            {
                const auto* q=d->qpos+layout.rootQpos; const auto* v=d->qvel+layout.rootQvel;
                state.root.position={q[0],q[1],q[2]}; state.root.orientation={q[3],q[4],q[5],q[6]};
                state.root.linearVelocityWorld={v[0],v[1],v[2]}; state.root.angularVelocityLocal={v[3],v[4],v[5]};
                if(!finite(state.root.position)||!finite(state.root.linearVelocityWorld)||!finite(state.root.angularVelocityLocal)||
                    !std::isfinite(glm::length(state.root.orientation))||std::abs(glm::length(state.root.orientation)-1)>1e-8||
                    glm::length(state.root.linearVelocityWorld)>def.maxRootLinearSpeed||glm::length(state.root.angularVelocityLocal)>def.maxRootAngularSpeed)
                    fail("Free root pose/velocity threshold exceeded");
                for(int k=0;k<6;++k) if(!std::isfinite(d->qacc[layout.rootQvel+k])) fail("Free root acceleration nonfinite");
            }
            state.contacts.clear();
            if(d->ncon>static_cast<int>(def.maxContacts)) fail("Contact count threshold exceeded");
            for(int i=0;i<d->ncon;++i)
            {
                const auto& c=d->contact[i]; mjtNum f[6]{}; mj_contactForce(m.get(),d.get(),i,f);
                Contact contact; contact.geom1=c.geom[0]; contact.geom2=c.geom[1];
                const auto linkFor=[&](int geom) { const auto it=std::find(bodies.begin(),bodies.end(),m->geom_bodyid[geom]); return it==bodies.end()?-1:static_cast<int>(it-bodies.begin()); };
                contact.link1=linkFor(c.geom[0]); contact.link2=linkFor(c.geom[1]);
                contact.point={c.pos[0],c.pos[1],c.pos[2]}; contact.normal={c.frame[0],c.frame[1],c.frame[2]};
                contact.distance=c.dist; contact.force=glm::dvec3{f[0],f[1],f[2]};
                {
                    const auto contactStart=Clock::now();
                    for(int axis=0;axis<3;++axis) for(int component=0;component<3;++component)
                        contact.worldFromContact[axis][component]=c.frame[3*axis+component];
                    contact.forceOn2World=contact.worldFromContact**contact.force;
                    contact.torqueOn2World=contact.worldFromContact*glm::dvec3{f[3],f[4],f[5]};
                    contact.velocity1World=pointVelocity(m->geom_bodyid[c.geom[0]],contact.point,&contact.angular1World);
                    contact.velocity2World=pointVelocity(m->geom_bodyid[c.geom[1]],contact.point,&contact.angular2World);
                    for(unsigned k=0;k<5;++k) contact.friction[k]=c.friction[k];
                    contact.active=c.efc_address>=0&&c.exclude==0;
                    state.timing.contactUs+=us(contactStart);
                }
                if(!finite(*contact.force)||glm::length(*contact.force)>def.maxContactForce||c.dist < -def.maxPenetration)
                    fail("Contact force/penetration threshold exceeded links="+std::to_string(contact.link1)+","+std::to_string(contact.link2)+
                        " distance_m="+std::to_string(c.dist)+" force_N="+std::to_string(glm::length(*contact.force)));
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
            if(!std::isfinite(state.potentialEnergy)||!std::isfinite(state.kineticEnergy)||state.kineticEnergy>def.maxKineticEnergy)
                fail("Energy threshold exceeded");
            state.time=d->time;
            { state.constraintPower=0; for(int k=0;k<m->nv;++k) state.constraintPower+=d->qfrc_constraint[k]*d->qvel[k]; }
        }
    };
    PhysicsWorld::PhysicsWorld(Definition def,Coordinates initial):impl_(std::make_unique<Impl>(std::move(def))) { reset(initial); }
    PhysicsWorld::~PhysicsWorld()=default;
    const Definition& PhysicsWorld::definition() const { return impl_->def; }
    const Snapshot& PhysicsWorld::snapshot() const { return impl_->state; }
    std::string PhysicsWorld::backendVersion() { return mj_versionString(); }
    glm::dmat4 rootTransform(const RootState& root)
    {
        auto result=glm::mat4_cast(root.orientation); result[3]=glm::dvec4{root.position,1}; return result;
    }
    const StateLayout& PhysicsWorld::layout() const { return impl_->layout; }
    Capabilities PhysicsWorld::capabilities() const
    {
        const auto& p=*impl_; Capabilities c; c.freeRoot=p.def.freeRoot; c.fixedBase=!c.freeRoot; c.fixedBaseGravity=c.fixedBase;
        if(c.fixedBase&&p.state.dofs==ArmCoordinates&&p.def.contactTool)
        {
            const auto& t=*p.def.contactTool; const auto& b=p.def.links[t.link];
            c.normalForceControl=!b.boxHalfSize&&glm::length(b.capsuleEnd-b.capsuleStart)<=1e-12&&
                glm::length(b.capsuleStart-t.pointInJoint)<=1e-12&&std::abs(b.radius-t.radius)<=1e-12;
        }
        return c;
    }
    namespace
    {
        void setRoot(const StateLayout& l,mjData* d,const RootState& root)
        {
            if(l.rootQpos<0) throw std::invalid_argument("Root state requires a free model");
            if(!finite(root.position)||!finite(root.linearVelocityWorld)||!finite(root.angularVelocityLocal)||
                !std::isfinite(glm::length(root.orientation))||std::abs(glm::length(root.orientation)-1)>1e-8)
                throw std::invalid_argument("Root requires finite state and unit quaternion");
            auto* q=d->qpos+l.rootQpos; auto* v=d->qvel+l.rootQvel;
            for(int k=0;k<3;++k) { q[k]=root.position[k]; v[k]=root.linearVelocityWorld[k]; v[k+3]=root.angularVelocityLocal[k]; }
            q[3]=root.orientation.w; q[4]=root.orientation.x; q[5]=root.orientation.y; q[6]=root.orientation.z;
        }
    }
    void PhysicsWorld::reset(Coordinates q,std::optional<RootState> root)
    {
        auto& p=*impl_;
        for(unsigned k=0;k<MaxCoordinates;++k)
            if(!std::isfinite(q[k])||(k>=p.state.dofs?q[k]!=0:(q[k]<p.hinges[k].lower||q[k]>p.hinges[k].upper)))
                throw std::invalid_argument("Reset requires finite, in-range coordinates");
        if(root) { mj_resetData(p.m.get(),p.scratch.get()); setRoot(p.layout,p.scratch.get(),*root); }
        const auto epoch=p.state.epoch+1; const auto dofs=p.state.dofs;
        mj_resetData(p.m.get(),p.d.get()); mj_resetData(p.m.get(),p.scratch.get());
        p.state={}; p.state.epoch=epoch; p.state.dofs=dofs; p.state.target=q;
        p.effort={}; p.failed=false; std::fill(p.wrench.begin(),p.wrench.end(),0);
        for(unsigned k=0;k<dofs;++k) p.d->qpos[p.layout.qpos[k]]=q[k]; // ONLY explicit reset writes live q/v.
        for(unsigned k=0;k<dofs;++k) p.references[k].reset(q[k]);
        p.state.drive.qRef=q;
        p.contactController.reset();
        p.standController.reset();
        if(root) setRoot(p.layout,p.d.get(),*root);
        mj_forward(p.m.get(),p.d.get()); p.read();
        p.initialLinearMomentum=p.state.linearMomentumWorld; p.initialAngularMomentum=p.state.angularMomentumAboutComWorld;
        p.observeSupport(0);
        if(p.def.contactTool) { p.contactController.observe(p.contactInput(),0); p.state.contact=p.contactController.status(); }
        p.initialMechanicalEnergy=p.state.potentialEnergy+p.state.kineticEnergy;
    }
    bool PhysicsWorld::command(CommandKind kind,Coordinates values,std::string* reason)
    {
        auto& p=*impl_;
        const auto reject=[&](const std::string& s) { if(reason) *reason=s; return false; };
        if(p.failed) return reject("Physics failed; reset required");
        if(kind!=CommandKind::Target&&kind!=CommandKind::SmoothTarget&&kind!=CommandKind::Torque&&kind!=CommandKind::Disable)
            return reject("Unsupported command in effort bench (no IK/attachment/kinematic root motion)");
        for(unsigned k=0;k<MaxCoordinates;++k)
        {
            if(!std::isfinite(values[k])||(k>=p.state.dofs&&values[k]!=0)) return reject("Invalid coordinates");
            if((kind==CommandKind::Target||kind==CommandKind::SmoothTarget)&&k<p.state.dofs&&(values[k]<p.hinges[k].lower||values[k]>p.hinges[k].upper))
                return reject("Target outside calibrated limits");
        }
        if(p.contactController.owns()&&kind!=CommandKind::Disable) return reject("Contact owns all arm coordinates; Disable before changing drive mode");
        if(p.state.mode==DriveMode::Stand&&kind!=CommandKind::Disable) return reject("Stand owns all hinges; Disable and prepared reset required");
        if(kind==CommandKind::SmoothTarget)
        {
            auto proposed=p.references;
            try
            {
                for(unsigned k=0;k<p.state.dofs;++k)
                {
                    if(p.state.mode!=DriveMode::Servo) proposed[k].reset(p.state.q[k]);
                    proposed[k].target(values[k],p.state.servo);
                }
            }
            catch(const std::invalid_argument& e) { return reject(e.what()); }
            p.references=std::move(proposed);
            for(unsigned k=0;k<p.state.dofs;++k)
            {
                p.state.drive.qRef[k]=p.references[k].q(); p.state.drive.vRef[k]=p.references[k].v();
                p.state.drive.referenceReached[k]=p.references[k].reached();
            }
            p.state.target=values; p.state.mode=DriveMode::Servo;
        }
        else if(kind==CommandKind::Target)
        { p.state.target=values; p.state.mode=DriveMode::Position; p.state.drive.qRef=values; p.state.drive.vRef={}; }
        else if(kind==CommandKind::Torque) { p.effort=values; p.state.mode=DriveMode::Torque; }
        else { p.state.mode=DriveMode::Disabled; p.contactController.disable(); p.state.contact=p.contactController.status(); p.standController.disable(); p.state.stand=p.standController.status; }
        if(kind!=CommandKind::SmoothTarget)
        { p.state.drive.vRef={}; p.state.drive.referenceReached={}; }
        for(unsigned k=0;k<p.state.dofs;++k)
        {
            p.state.drive.positionError[k]=p.state.drive.qRef[k]-p.state.q[k];
            p.state.drive.velocityError[k]=p.state.drive.vRef[k]-p.state.v[k];
        }
        if(reason) reason->clear(); return true;
    }
    bool PhysicsWorld::configureServo(ServoSettings settings,std::string* reason)
    {
        auto& p=*impl_;
        const auto reject=[&](const char* s) { if(reason) *reason=s; return false; };
        if(p.failed) return reject("Physics failed; reset required");
        if(p.contactController.owns()) return reject("Contact owns arm; Disable before configuring Servo");
        if(p.state.mode==DriveMode::Stand) return reject("Stand owns hinges; Disable before configuring Servo");
        if(!validServoSettings(settings))
            return reject("Invalid servo speed/acceleration limits");
        if(settings.gravityCompensation && p.def.freeRoot) return reject("Gravity compensation requires a fixed base; free root unsupported");
        if(p.state.mode==DriveMode::Servo) return reject("Disable servo before changing settings");
        p.state.servo=settings; if(reason) reason->clear(); return true;
    }
    bool PhysicsWorld::startContact(ContactSettings settings,std::string* reason)
    {
        auto& p=*impl_;
        const auto reject=[&](const char* why) { if(reason) *reason=why; return false; };
        if(p.failed) return reject("Physics failed; reset required");
        if(!validContactSettings(settings)) return reject("Invalid contact settings");
        if(p.def.freeRoot||p.state.dofs!=ArmCoordinates||!p.def.contactTool)
            return reject("Managed contact supports only an explicitly configured fixed Full arm");
        const auto& tool=*p.def.contactTool; const auto& b=p.def.links[tool.link];
        if(b.boxHalfSize||glm::length(b.capsuleEnd-b.capsuleStart)>1e-12||
            glm::length(b.capsuleStart-tool.pointInJoint)>1e-12||std::abs(b.radius-tool.radius)>1e-12)
            return reject("Managed contact requires the declared sphere tool collider");
        if(p.state.mode!=DriveMode::Disabled) return reject("Disable the previous drive before starting contact");
        auto proposed=p.contactController;
        proposed.start(settings,tool.normalWorld,p.contactInput());
        p.contactController=std::move(proposed); p.state.mode=DriveMode::Contact;
        p.state.target=p.state.q; p.state.drive.qRef=p.state.q; p.state.drive.vRef={}; p.state.drive.referenceReached={};
        p.state.contact=p.contactController.status();
        if(reason) reason->clear(); return true;
    }
    bool PhysicsWorld::contactForceTarget(double force,std::string* reason)
    {
        auto& p=*impl_;
        if(p.failed||p.state.mode!=DriveMode::Contact||!p.contactController.retarget(force))
        { if(reason) *reason="Force target requires active Ramp/Hold and finite force in [1,8] N"; return false; }
        p.state.contact=p.contactController.status(); if(reason) reason->clear(); return true;
    }
    bool PhysicsWorld::startStand(const StandSettings& settings,std::string* reason)
    {
        auto& p=*impl_;
        const auto reject=[&](const char* why) { if(reason) *reason=why; return false; };
        if(p.failed||p.state.mode!=DriveMode::Disabled||p.state.tick!=0) return reject("Stand requires Disabled immediately after prepared reset");
        if(settings.loadCommands&&(settings.comCommands||settings.variant!=StandVariant::Bounded7h||settings.boundedSolver!=BoundedSolver::Optimized7i))
            return reject("Load commands require exclusive bounded optimized control");
        if((settings.variant!=StandVariant::Equality7g&&settings.variant!=StandVariant::Bounded7h)||
            (settings.boundedSolver!=BoundedSolver::Reference7h&&settings.boundedSolver!=BoundedSolver::Optimized7i)||
            (settings.variant!=StandVariant::Bounded7h&&settings.boundedSolver!=BoundedSolver::Reference7h)||
            !std::isfinite(settings.motion.maxDisplacement)||settings.motion.maxDisplacement<.002||settings.motion.maxDisplacement>.005||
            !std::isfinite(settings.motion.maxSpeed)||settings.motion.maxSpeed<1e-5||settings.motion.maxSpeed>.001||
            !std::isfinite(settings.motion.maxAcceleration)||settings.motion.maxAcceleration<1e-5||settings.motion.maxAcceleration>.002)
            return reject("Invalid bounded COM motion settings");
        if(!p.def.freeRoot||p.state.dofs!=41||p.def.links.size()!=20||!p.def.floorHeight||!p.def.supportObservation||
            !p.state.support.dynamics.valid||p.def.gravity!=glm::dvec3{0,-9.81,0}||p.def.contactTool||
            std::abs(p.state.totalMass-55)>1e-8||(p.def.substeps!=16&&p.def.substeps!=32&&p.def.substeps!=64))
            return reject("Stand supports the observed 55kg free BodyRig on a stationary horizontal floor");
        for(const auto link:p.footLinks) if(link<0||!p.def.links[static_cast<std::size_t>(link)].boxHalfSize||
            glm::length(*p.def.links[static_cast<std::size_t>(link)].boxHalfSize-glm::dvec3{.045,.025,.085})>1e-10)
            return reject("Stand requires the audited foot box dimensions");
        if(!finite(settings.comGoal)||!finite(settings.rootGoal.position)||
            !std::isfinite(glm::length(settings.rootGoal.orientation))||std::abs(glm::length(settings.rootGoal.orientation)-1)>1e-8||
            glm::length(settings.rootGoal.linearVelocityWorld)!=0||glm::length(settings.rootGoal.angularVelocityLocal)!=0||
            !std::isfinite(glm::length(settings.thoraxGoal))||std::abs(glm::length(settings.thoraxGoal)-1)>1e-8)
            return reject("Invalid stand goals");
        if((settings.comCommands||settings.loadCommands)&&glm::length(settings.rootGoal.orientation*glm::dvec3{0,1,0}-glm::dvec3{0,1,0})>1e-8)
            return reject("COM command placement must preserve world up (translation and yaw only)");
        for(unsigned k=0;k<MaxCoordinates;++k)
            if(!std::isfinite(settings.posture[k])||(k>=41?settings.posture[k]!=0:
                settings.posture[k]<p.hinges[k].lower||settings.posture[k]>p.hinges[k].upper)) return reject("Invalid stand posture");
        for(const auto& w:settings.nominalReaction) if(!finite(w.origin)||!finite(w.force)||!finite(w.moment)||w.force.y<30||glm::length(w.force)>1000||glm::length(w.moment)>100)
            return reject("Invalid nominal reaction prediction");
        p.standController.start(settings,p.state,p.def); p.state.stand=p.standController.status;
        p.state.mode=DriveMode::Stand; p.state.target=settings.posture; p.state.drive.qRef=settings.posture; p.state.drive.vRef={};
        if(reason) reason->clear(); return true;
    }
    bool PhysicsWorld::standBalanceFeedback(bool enabled,std::string* reason)
    {
        auto& p=*impl_;
        if(p.failed||p.state.mode!=DriveMode::Stand||p.state.stand.phase==StandPhase::Lost)
        { if(reason) *reason="Balance ablation requires active Stand"; return false; }
        p.standController.balanceFeedback(enabled); p.state.stand=p.standController.status;
        if(reason) reason->clear(); return true;
    }
    bool PhysicsWorld::standComTarget(glm::dvec2 offset,std::string* reason)
    {
        auto& p=*impl_;
        if(p.failed||p.state.mode!=DriveMode::Stand) { if(reason) *reason="COM target requires active Stand"; return false; }
        if(!comCommandSupports(p.state)) { if(reason) *reason="COM target requires current measured two-foot supports"; return false; }
        if(!p.standController.target(offset,reason)) return false;
        p.state.stand=p.standController.status; return true;
    }
    bool PhysicsWorld::cancelStandCom(std::string* reason)
    {
        auto& p=*impl_;
        if(p.failed||p.state.mode!=DriveMode::Stand) { if(reason) *reason="COM cancel requires active Stand"; return false; }
        if(!comCommandSupports(p.state)) { if(reason) *reason="COM cancel requires current measured two-foot supports"; return false; }
        if(!p.standController.cancel(reason)) return false;
        p.state.stand=p.standController.status; return true;
    }
    bool PhysicsWorld::standLoadTarget(double alpha,std::string* reason)
    {
        auto& p=*impl_;
        if(p.failed||p.state.mode!=DriveMode::Stand||!comCommandSupports(p.state))
        { if(reason) *reason="Load target requires current measured two-foot Stand supports"; return false; }
        if(!p.standController.loadTarget(alpha,reason)) return false;
        p.state.stand=p.standController.status; return true;
    }
    bool PhysicsWorld::cancelStandLoad(std::string* reason)
    {
        auto& p=*impl_;
        if(p.failed||p.state.mode!=DriveMode::Stand||!comCommandSupports(p.state))
        { if(reason) *reason="Load cancel requires current measured two-foot Stand supports"; return false; }
        if(!p.standController.loadCancel(reason)) return false;
        p.state.stand=p.standController.status; return true;
    }
    bool PhysicsWorld::returnStandLoad(std::string* reason)
    {
        auto& p=*impl_;
        if(p.failed||p.state.mode!=DriveMode::Stand||!comCommandSupports(p.state))
        { if(reason) *reason="Load return requires current measured two-foot Stand supports"; return false; }
        if(!p.standController.loadReturn(reason)) return false;
        p.state.stand=p.standController.status; return true;
    }
    void PhysicsWorld::cancelContact()
    { auto& p=*impl_; p.contactController.cancel(); p.state.contact=p.contactController.status(); }
    Coordinates PhysicsWorld::gravityCompensation(Coordinates q) const
    {
        auto& p=*impl_;
        if(p.def.freeRoot) throw std::invalid_argument("Gravity compensation requires a fixed base");
        for(unsigned k=0;k<MaxCoordinates;++k)
            if(!std::isfinite(q[k])||(k>=p.state.dofs&&q[k]!=0)) throw std::invalid_argument("Invalid gravity query");
        // Local input buffer prevents aliasing the scratch qpos reset in gravity().
        std::array<mjtNum,MaxCoordinates> qpos{};
        for(unsigned k=0;k<p.state.dofs;++k) qpos[static_cast<std::size_t>(p.layout.qpos[k])]=q[k];
        return p.gravity(qpos.data());
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
        auto& s=p.state; ++s.tick; s.timing={}; s.saturatedThisTick={}; s.peakLimitViolationThisTick={};
        p.standController.status.controllerUs=0;
        p.standController.status.solverSampleCount=0;
        s.peakContactsThisTick=0; s.peakNormalForceThisTick=0; s.maxPenetrationThisTick=0;
        s.externalForce={0,0,0}; s.externalTorque={0,0,0};
        s.support.dynamics.tickLinearImpulse={0,0,0}; s.support.dynamics.tickAngularImpulse={0,0,0};
        s.support.dynamics.substeps.clear();
        std::copy(p.wrench.begin(),p.wrench.end(),p.d->xfrc_applied);
        for(std::size_t b=0;b<p.wrench.size();b+=6)
        { s.externalForce+=glm::dvec3{p.wrench[b],p.wrench[b+1],p.wrench[b+2]}; s.externalTorque+=glm::dvec3{p.wrench[b+3],p.wrench[b+4],p.wrench[b+5]}; }
        for(unsigned sub=0;sub<p.def.substeps;++sub)
        {
            auto start=Clock::now();
            const double h=TickSeconds/p.def.substeps;
            const auto observationStart=Clock::now();
            const auto beforeResultant=p.def.supportObservation?p.totalResultant():Wrench{};
            s.timing.supportUs+=p.def.supportObservation?us(observationStart):0;
            s.drive.sampleMode=s.mode;
            Coordinates ff{},beforeV{},actualTorque{};
            Coordinates contactEffort{};
            Coordinates standEffort{};
            const double externalBefore=p.wrenchPower();
            if(s.mode==DriveMode::Servo && s.servo.gravityCompensation) ff=p.gravity(p.d->qpos);
            if(s.mode==DriveMode::Contact)
            {
                const auto contactStart=Clock::now();
                // Uses only the previous completed endpoint. No solver callback,
                // no SmoothTarget replanning, one owner for all six coordinates.
                contactEffort=p.contactController.evaluate(p.contactInput(true),h);
                s.timing.contactControlUs+=us(contactStart);
                ff=p.gravity(p.d->qpos);
            }
            if(s.mode==DriveMode::Stand)
            {
                // Four bounded inverse-dynamics evaluations per external tick (480Hz).
                // Validate actual supports at every physical substep before reuse.
                standEffort=p.standController.evaluate(p.m.get(),p.d.get(),p.layout,p.def,s,p.bodies,p.footLinks,h,
                    sub%(p.def.substeps/4)==0);
                s.stand=p.standController.status;
            }
            for(unsigned k=0;k<s.dofs;++k)
            {
                auto& t=s.drive;
                t.sampleQ[k]=p.d->qpos[p.layout.qpos[k]]; t.sampleV[k]=beforeV[k]=p.d->qvel[p.layout.qvel[k]];
                t.sampleQRef[k]=s.mode==DriveMode::Position?s.target[k]:t.qRef[k]; t.sampleVRef[k]=t.vRef[k];
                const auto output=evaluateDrive(p.def.drives[k],s.mode,t.sampleQRef[k],t.sampleVRef[k],t.sampleQ[k],t.sampleV[k],ff[k],
                    s.mode==DriveMode::Stand?standEffort[k]:s.mode==DriveMode::Contact?contactEffort[k]:p.effort[k]);
                t.pTorque[k]=output.p; t.dTorque[k]=output.d; t.feedforward[k]=output.feedforward;
                s.calculatedTorque[k]=output.requested; t.limitedTorque[k]=output.limited;
                p.d->ctrl[p.layout.actuator[k]]=output.limited;
                s.saturated[k]=output.saturated;
                s.saturatedThisTick[k]=s.saturatedThisTick[k]||s.saturated[k];
                t.saturationRunSeconds[k]=s.saturated[k]?t.saturationRunSeconds[k]+h:0;
                if(s.saturated[k]) t.saturatedSeconds[k]+=h;
            }
            if(s.mode==DriveMode::Contact)
                p.contactController.applied(std::any_of(s.saturated.begin(),s.saturated.begin()+s.dofs,[](bool b) { return b; }),h);
            s.timing.controlUs+=us(start); start=Clock::now();
            mj_step(p.m.get(),p.d.get()); ++s.worldSteps;
            s.timing.stepUs+=us(start); start=Clock::now();
            for(unsigned k=0;k<s.dofs;++k) actualTorque[k]=p.d->qfrc_actuator[p.layout.qvel[k]];
            auto& t=s.drive; t.passiveDampingPower=0;
            for(unsigned k=0;k<s.dofs;++k)
            {
                const double after=p.d->qvel[p.layout.qvel[k]], torque=actualTorque[k];
                t.power[k]=torque*after;
                const auto work=integratePower(torque*beforeV[k],t.power[k],h);
                t.deliveredWork[k]+=work.delivered; t.absorbedWork[k]+=work.absorbed;
                const double damping=p.hinges[k].damping;
                t.passiveDampingPower-=damping*after*after;
                t.passiveDampingWork-=.5*damping*(beforeV[k]*beforeV[k]+after*after)*h;
                if(s.mode==DriveMode::Servo)
                {
                    p.references[k].advance(h); t.qRef[k]=p.references[k].q(); t.vRef[k]=p.references[k].v();
                }
                t.referenceReached[k]=s.mode==DriveMode::Servo && p.references[k].reached();
                t.positionError[k]=t.qRef[k]-p.d->qpos[p.layout.qpos[k]]; t.velocityError[k]=t.vRef[k]-after;
            }
            s.timing.telemetryUs+=us(start); start=Clock::now();
            // mj_step leaves derived transforms at pre-integration q. Refresh ALL
            // endpoint observations with the last held ctrl; never alter live q/v.
            mj_forward(p.m.get(),p.d.get()); p.read();
            p.observeSupport(h,beforeResultant);
            if(p.standController.status.load.enabled)
            { p.standController.observeLoad(s,h); s.stand=p.standController.status; }
            if(p.def.contactTool)
            {
                const auto contactStart=Clock::now();
                p.contactController.observe(p.contactInput(),h); s.contact=p.contactController.status();
                s.timing.contactUs+=us(contactStart);
            }
            s.drive.externalPower=p.wrenchPower();
            s.drive.externalWork+=.5*(externalBefore+s.drive.externalPower)*h;
            s.drive.mechanicalEnergyChange=s.kineticEnergy+s.potentialEnergy-p.initialMechanicalEnergy;
            s.timing.readUs+=us(start);
            const double expected=static_cast<double>(s.worldSteps)*(TickSeconds/p.def.substeps);
            if(std::abs(s.time-expected)>1e-8) p.fail("Physics time discontinuity");
        }
        std::fill(p.wrench.begin(),p.wrench.end(),0);
        mju_zero(p.d->xfrc_applied,static_cast<int>(p.wrench.size()));
    }
    std::vector<glm::dmat4> PhysicsWorld::forwardKinematics(Coordinates q,std::optional<RootState> root) const
    {
        auto& p=*impl_;
        for(unsigned k=0;k<MaxCoordinates;++k)
            if(!std::isfinite(q[k])||(k>=p.state.dofs&&q[k]!=0)) throw std::invalid_argument("Invalid FK coordinates");
        mj_resetData(p.m.get(),p.scratch.get());
        for(unsigned k=0;k<p.state.dofs;++k) p.scratch->qpos[p.layout.qpos[k]]=q[k];
        if(root) setRoot(p.layout,p.scratch.get(),*root);
        mj_kinematics(p.m.get(),p.scratch.get());
        std::vector<glm::dmat4> result;
        for(const auto id:p.bodies) result.push_back(bodyFrame(p.scratch.get(),id));
        return result;
    }
    MassAudit PhysicsWorld::massAudit(Coordinates q,std::optional<RootState> root) const
    {
        const auto frames=forwardKinematics(q,root);
        auto& p=*impl_; mj_forward(p.m.get(),p.scratch.get());
        MassAudit a;
        for(std::size_t i=0;i<p.bodies.size();++i)
        {
            const auto id=p.bodies[i]; const auto mass=p.m->body_mass[id];
            a.masses.push_back(mass); a.totalMass+=mass;
            const auto com=glm::dvec3{p.m->body_ipos[3*id],p.m->body_ipos[3*id+1],p.m->body_ipos[3*id+2]};
            a.comInJoint.push_back(com);
            const auto* quat=p.m->body_iquat+4*id;
            const auto rot=glm::mat3_cast(glm::dquat{quat[0],quat[1],quat[2],quat[3]});
            glm::dmat3 diagonal{0}; for(int k=0;k<3;++k) diagonal[k][k]=p.m->body_inertia[3*id+k];
            a.inertiaInJoint.push_back(rot*diagonal*glm::transpose(rot));
            a.comWorld+=mass*glm::dvec3{frames[i]*glm::dvec4{com,1}};
        }
        a.comWorld/=a.totalMass;
        for(std::size_t i=0;i<frames.size();++i)
        {
            const glm::dmat3 r{frames[i]};
            const auto d=glm::dvec3{frames[i]*glm::dvec4{a.comInJoint[i],1}}-a.comWorld;
            a.inertiaAtComWorld+=r*a.inertiaInJoint[i]*glm::transpose(r)+
                a.masses[i]*(glm::dot(d,d)*glm::dmat3{1}-glm::outerProduct(d,d));
        }
        a.massMatrix.resize(static_cast<std::size_t>(p.m->nv*p.m->nv));
        mj_fullM(p.m.get(),a.massMatrix.data(),p.scratch->qM);
        return a;
    }
    std::optional<ContactMaterial> PhysicsWorld::contactMaterial() const
    {
        const auto& p=*impl_; if(!p.def.contactTool) return std::nullopt;
        ContactMaterial a; a.geom1=p.m->pair_geom1[0]; a.geom2=p.m->pair_geom2[0]; a.dimension=p.m->pair_dim[0];
        for(unsigned k=0;k<5;++k) { a.friction[k]=p.m->pair_friction[k]; a.solimp[k]=p.m->pair_solimp[k]; }
        for(unsigned k=0;k<2;++k) a.solref[k]=p.m->pair_solref[k];
        return a;
    }
    std::size_t PhysicsWorld::modelBytes() const { return sizeof(mjModel)+static_cast<std::size_t>(impl_->m->nbuffer); }
    std::size_t PhysicsWorld::dataBytes() const { return sizeof(mjData)+static_cast<std::size_t>(impl_->d->nbuffer+impl_->d->narena); }
    std::size_t PhysicsWorld::arenaHighWaterBytes() const { return static_cast<std::size_t>(impl_->d->maxuse_arena); }
    std::size_t PhysicsWorld::standControllerBytes() const { return sizeof(StandControl); }
    bool PhysicsWorld::inspectStandQp(qp::Problem& problem,qp::Result& result) const { return impl_->standController.inspectQp(problem,result); }
    std::string PhysicsWorld::modelXml() const { return impl_->source; }
    void driveCsvHeader(std::ostream& out)
    {
        out<<"schema,tick,epoch,coordinate,time_s,sample_time_s,mode,sample_mode,goal_rad,q_ref_rad,v_ref_rad_s,q_rad,v_rad_s,error_rad,error_rad_s,"
            "sample_q_rad,sample_v_rad_s,sample_q_ref_rad,sample_v_ref_rad_s,p_Nm,d_Nm,ff_Nm,requested_Nm,limited_Nm,actual_Nm,"
            "power_W,delivered_J,absorbed_J,net_J,saturated_last,saturated_tick,saturated_s,saturation_run_s,reference_reached,"
            "gravity_compensation,max_ref_speed_rad_s,max_ref_acceleration_rad_s2,total_passive_damping_W,total_passive_damping_J,total_external_W,total_external_J,mechanical_energy_change_J\n";
    }
    void driveCsvRow(std::ostream& out,const Snapshot& s,unsigned substeps)
    {
        const auto& t=s.drive; out<<std::setprecision(17);
        for(unsigned k=0;k<s.dofs;++k)
            out<<1<<','<<s.tick<<','<<s.epoch<<','<<k<<','<<s.time<<','<<(s.tick?s.time-TickSeconds/substeps:0)<<','<<static_cast<int>(s.mode)
                <<','<<static_cast<int>(t.sampleMode)
                <<','<<s.target[k]<<','<<t.qRef[k]<<','<<t.vRef[k]<<','<<s.q[k]<<','<<s.v[k]<<','<<t.positionError[k]<<','<<t.velocityError[k]
                <<','<<t.sampleQ[k]<<','<<t.sampleV[k]<<','<<t.sampleQRef[k]<<','<<t.sampleVRef[k]<<','<<t.pTorque[k]<<','<<t.dTorque[k]
                <<','<<t.feedforward[k]<<','<<s.calculatedTorque[k]<<','<<t.limitedTorque[k]<<','<<s.appliedTorque[k]<<','<<t.power[k]
                <<','<<t.deliveredWork[k]<<','<<t.absorbedWork[k]<<','<<t.deliveredWork[k]-t.absorbedWork[k]<<','<<s.saturated[k]<<','<<s.saturatedThisTick[k]
                <<','<<t.saturatedSeconds[k]<<','<<t.saturationRunSeconds[k]<<','<<t.referenceReached[k]<<','<<s.servo.gravityCompensation
                <<','<<s.servo.maxSpeed<<','<<s.servo.maxAcceleration<<','<<t.passiveDampingPower<<','<<t.passiveDampingWork
                <<','<<t.externalPower<<','<<t.externalWork<<','<<t.mechanicalEnergyChange<<'\n';
    }
}
