#include "julia/physics/PhysicsWorld.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>

namespace julia::physics
{
    void Wrench::add(glm::dvec3 p,glm::dvec3 f,glm::dvec3 t) { force+=f; moment+=glm::cross(p-origin,f)+t; }
    Wrench Wrench::about(glm::dvec3 p) const { return {p,force,moment+glm::cross(origin-p,force)}; }
    bool forceActive(const Contact& c) { return c.active&&c.force&&c.force->x>1e-9; }
    Wrench contactWrench(const Contact& c,int geom,glm::dvec3 origin)
    {
        Wrench w; w.origin=origin;
        if(forceActive(c)&&(geom==c.geom1||geom==c.geom2))
        { const double sign=geom==c.geom2?1.:-1.; w.add(c.point,sign*c.forceOn2World,sign*c.torqueOn2World); }
        return w;
    }
    Wrench pairWrench(const std::vector<Contact>& contacts,int geom,int surface,glm::dvec3 origin)
    {
        Wrench result; result.origin=origin;
        for(const auto& c:contacts) if((c.geom1==geom&&c.geom2==surface)||(c.geom2==geom&&c.geom1==surface))
        { const auto w=contactWrench(c,geom,origin); result.force+=w.force; result.moment+=w.moment; }
        return result;
    }
    const char* regionName(RegionKind k)
    { switch(k) { case RegionKind::Empty:return "empty"; case RegionKind::Point:return "point"; case RegionKind::Segment:return "segment"; case RegionKind::Polygon:return "polygon"; } return "invalid"; }
    PlanarRegion horizontalHull(std::vector<glm::dvec3> p)
    {
        PlanarRegion r;
        std::sort(p.begin(),p.end(),[](auto a,auto b) { return a.x==b.x?a.z<b.z:a.x<b.x; });
        p.erase(std::unique(p.begin(),p.end(),[](auto a,auto b) { return glm::length(a-b)<=1e-8; }),p.end());
        if(p.empty()) return r;
        if(p.size()==1) { r.kind=RegionKind::Point; r.vertices=p; return r; }
        const auto turn=[](auto a,auto b,auto c) { return (b.x-a.x)*(c.z-a.z)-(b.z-a.z)*(c.x-a.x); };
        std::vector<glm::dvec3> hull;
        for(const auto v:p) { while(hull.size()>=2&&turn(hull[hull.size()-2],hull.back(),v)<=0) hull.pop_back(); hull.push_back(v); }
        const auto lower=hull.size();
        for(std::size_t i=p.size()-1;i-->0;) { const auto v=p[i]; while(hull.size()>lower&&turn(hull[hull.size()-2],hull.back(),v)<=0) hull.pop_back(); hull.push_back(v); }
        hull.pop_back(); double twiceArea=0;
        for(std::size_t i=1;i+1<hull.size();++i) twiceArea+=turn(hull[0],hull[i],hull[i+1]);
        r.area=std::abs(twiceArea)*.5;
        if(r.area<=1e-8)
        {
            double farthest=-1; glm::dvec3 a=p.front(),b=p.back();
            for(auto x:p) for(auto y:p) { const double d=glm::dot(x-y,x-y); if(d>farthest) { farthest=d; a=x; b=y; } }
            r.kind=RegionKind::Segment; r.vertices={a,b}; r.area=0;
        }
        else { r.kind=RegionKind::Polygon; r.vertices=std::move(hull); }
        return r;
    }
    const char* copReasonName(CopReason r)
    { switch(r) { case CopReason::Available:return "available"; case CopReason::ObserverDisabled:return "observer_disabled"; case CopReason::NoFloor:return "no_floor"; case CopReason::InsufficientNormalForce:return "insufficient_normal_force"; case CopReason::Nonfinite:return "nonfinite"; } return "invalid"; }
    PressureCenter pressureCenter(const Wrench& source,glm::dvec3 origin,double minimum)
    {
        const auto w=source.about(origin); PressureCenter c; c.normalForce=w.force.y; c.normalMoment=w.moment.y;
        if(!std::isfinite(c.normalForce)||!std::isfinite(glm::length(w.force))||!std::isfinite(glm::length(w.moment))||!std::isfinite(glm::length(origin))||!std::isfinite(minimum)||minimum<=0) { c.reason=CopReason::Nonfinite; return c; }
        if(c.normalForce<minimum) { c.reason=CopReason::InsufficientNormalForce; return c; }
        c.point=origin+glm::cross(glm::dvec3{0,1,0},w.moment)/c.normalForce; c.reason=CopReason::Available; return c;
    }
    void LoadState::observe(double force,double dt,const SupportSettings& s)
    {
        if(!loaded)
        {
            lossSeconds=0; confirmationSeconds=force>=s.confirmForce?confirmationSeconds+dt:0;
            if(confirmationSeconds+1e-12>=s.confirmSeconds) { loaded=true; confirmationSeconds=0; }
        }
        else
        {
            confirmationSeconds=0; lossSeconds=force<=s.lossForce?lossSeconds+dt:0;
            if(lossSeconds+1e-12>=s.lossSeconds) { loaded=false; lossSeconds=0; }
        }
    }
    void observeSupports(SupportObservation& o,const Snapshot& s,const SupportSettings& settings,
        const std::array<int,2>& links,const std::array<int,2>& geoms,int floor,std::optional<double> height,
        const std::vector<glm::dmat4>& boxes,const std::vector<glm::dvec3>& half,double dt)
    {
        o.enabled=true; o.floorAvailable=height.has_value(); o.planeOrigin={0,height.value_or(0),0};
        const auto project=[&](glm::dvec3 p) { p.y=o.planeOrigin.y; return p; };
        o.comProjection=project(s.totalComWorld);
        std::vector<glm::dvec3> allSoles,allLoaded; Wrench combined; combined.origin=o.planeOrigin;
        for(std::size_t side=0;side<2;++side)
        {
            const auto oldLoad=o.feet[side].load; auto& f=o.feet[side]; f={}; f.load=oldLoad;
            f.link=links[side]; f.geom=geoms[side]; f.surfaceGeom=floor;
            const auto frame=s.worldFromJoint[static_cast<std::size_t>(f.link)];
            f.wrench=pairWrench(s.contacts,f.geom,floor,glm::dvec3{(frame*settings.feet[side].jointFromSole)[3]});
            f.normalForce=f.wrench.force.y; f.tangentForce=f.wrench.force-glm::dvec3{0,f.normalForce,0};
            const auto box=frame*boxes[side]; f.geometricGap=1e100;
            std::vector<glm::dvec3> sole,loaded;
            for(int x:{-1,1}) for(int z:{-1,1})
                sole.push_back(project(glm::dvec3{box*glm::dvec4{half[side].x*x,-half[side].y,half[side].z*z,1}}));
            for(int x:{-1,1}) for(int y:{-1,1}) for(int z:{-1,1})
                f.geometricGap=std::min(f.geometricGap,(box*glm::dvec4{half[side]*glm::dvec3{x,y,z},1}).y-o.planeOrigin.y);
            f.near=height&&f.geometricGap<=settings.nearDistance;
            for(const auto& c:s.contacts) if((c.geom1==f.geom&&c.geom2==floor)||(c.geom2==f.geom&&c.geom1==floor))
            {
                ++f.records; f.penetration=std::max(f.penetration,-c.distance);
                if(!forceActive(c)) continue;
                ++f.active; const double sign=c.geom2==f.geom?1.:-1.;
                const auto force=sign*c.forceOn2World,vel=sign*(c.velocity2World-c.velocity1World);
                f.relativeVelocity+=vel; f.slipSpeed=std::max(f.slipSpeed,glm::length(glm::dvec3{vel.x,0,vel.z}));
                if(force.y>1e-6) loaded.push_back(project(c.point));
                f.projectionErrorBound+=std::abs(c.point.y-o.planeOrigin.y)*glm::length(glm::dvec3{force.x,0,force.z});
            }
            if(f.active) f.relativeVelocity/=f.active;
            if(height)
            {
                f.cop=pressureCenter(f.wrench,o.planeOrigin,settings.copMinimumForce);
                if(f.cop.point) f.projectionErrorBound/=f.normalForce; else f.projectionErrorBound=0;
                f.soleProjection=horizontalHull(sole); f.loadedRegion=horizontalHull(loaded);
                f.load.observe(f.normalForce,dt,settings);
            }
            else { f.near=false; f.geometricGap=0; f.cop.reason=CopReason::NoFloor; }
            const auto w=f.wrench.about(o.planeOrigin); combined.force+=w.force; combined.moment+=w.moment;
            allSoles.insert(allSoles.end(),sole.begin(),sole.end()); allLoaded.insert(allLoaded.end(),loaded.begin(),loaded.end());
        }
        o.combinedCop=height?pressureCenter(combined,o.planeOrigin,settings.copMinimumForce):PressureCenter{};
        if(!height) o.combinedCop.reason=CopReason::NoFloor;
        o.loadedRegion=height?horizontalHull(allLoaded):PlanarRegion{}; o.soleProjection=height?horizontalHull(allSoles):PlanarRegion{};
        o.otherEnvironmentRecords=o.otherEnvironmentActive=o.selfRecords=o.selfActive=0;
        o.otherEnvironment={s.totalComWorld,{0,0,0},{0,0,0}};
        for(const auto& c:s.contacts)
        {
            if(c.link1>=0&&c.link2>=0) { ++o.selfRecords; if(forceActive(c)) ++o.selfActive; continue; }
            if((c.link1>=0)==(c.link2>=0)) continue;
            const int bodyGeom=c.link1>=0?c.geom1:c.geom2,envGeom=c.link1>=0?c.geom2:c.geom1;
            if(envGeom==floor&&(bodyGeom==geoms[0]||bodyGeom==geoms[1])) continue;
            ++o.otherEnvironmentRecords;
            if(forceActive(c)) { ++o.otherEnvironmentActive; const auto w=contactWrench(c,bodyGeom,s.totalComWorld); o.otherEnvironment.force+=w.force; o.otherEnvironment.moment+=w.moment; }
        }
    }
    namespace
    {
        void vec(std::ostream& out,glm::dvec3 v) { out<<','<<v.x<<','<<v.y<<','<<v.z; }
        void cop(std::ostream& out,const PressureCenter& c)
        { out<<','<<copReasonName(c.reason); if(c.point) vec(out,*c.point); else out<<",NA,NA,NA"; }
    }
    void supportCsvHeader(std::ostream& o)
    {
        o<<"schema,tick,epoch,time_s,side,enabled,floor,link,geom,surface,near,records,active,loaded,confirm_s,loss_s,gap_m,penetration_m,normal_N,fx_N,fy_N,fz_N,tx_N,ty_N,tz_N,ox_m,oy_m,oz_m,mx_Nm,my_Nm,mz_Nm,rvx_m_s,rvy_m_s,rvz_m_s,slip_m_s,cop_reason,cop_x_m,cop_y_m,cop_z_m,projection_bound_m,sole_kind,sole_area_m2,loaded_kind,loaded_area_m2,combined_cop_reason,combined_x_m,combined_y_m,combined_z_m,other_records,other_active,self_records,self_active,com_projection_x_m,com_projection_y_m,com_projection_z_m\n";
    }
    void supportCsvRow(std::ostream& out,const Snapshot& s)
    {
        out<<std::setprecision(17); const auto& o=s.support;
        for(std::size_t i=0;i<2;++i)
        {
            const auto& f=o.feet[i]; out<<1<<','<<s.tick<<','<<s.epoch<<','<<s.time<<','<<(i==0?"LeftFoot":"RightFoot")<<','<<o.enabled<<','<<o.floorAvailable<<','<<f.link<<','<<f.geom<<','<<f.surfaceGeom
                <<','<<f.near<<','<<f.records<<','<<f.active<<','<<f.load.loaded<<','<<f.load.confirmationSeconds<<','<<f.load.lossSeconds<<','<<f.geometricGap<<','<<f.penetration<<','<<f.normalForce;
            vec(out,f.wrench.force); vec(out,f.tangentForce); vec(out,f.wrench.origin); vec(out,f.wrench.moment); vec(out,f.relativeVelocity);
            out<<','<<f.slipSpeed; cop(out,f.cop); out<<','<<f.projectionErrorBound<<','<<regionName(f.soleProjection.kind)<<','<<f.soleProjection.area<<','<<regionName(f.loadedRegion.kind)<<','<<f.loadedRegion.area;
            cop(out,o.combinedCop); out<<','<<o.otherEnvironmentRecords<<','<<o.otherEnvironmentActive<<','<<o.selfRecords<<','<<o.selfActive; vec(out,o.comProjection); out<<'\n';
        }
    }
    void dynamicsCsvHeader(std::ostream& o)
    {
        o<<"schema,tick,epoch,time_s,valid,validity_reason,samples,mass_kg,com_x_m,com_y_m,com_z_m,vx_m_s,vy_m_s,vz_m_s,jvx_m_s,jvy_m_s,jvz_m_s";
        for(const char* name:{"gravity","environment","applied","total"}) for(const char* unit:{"fx_N","fy_N","fz_N","mx_Nm","my_Nm","mz_Nm"}) o<<','<<name<<'_'<<unit;
        for(const char* name:{"impulse","angular_impulse","endpoint_impulse","endpoint_angular_impulse","delta_p","delta_L","residual_p","residual_L","tick_impulse","tick_angular_impulse"}) for(const char* axis:{"x","y","z"}) o<<','<<name<<'_'<<axis;
        o<<",observer_us,tick_us\n";
    }
    void dynamicsCsvRow(std::ostream& out,const Snapshot& s)
    {
        const auto& d=s.support.dynamics; out<<std::setprecision(17)<<1<<','<<s.tick<<','<<s.epoch<<','<<s.time<<','<<d.valid<<','<<(d.valid?"free_body":s.support.enabled?"unmeasured_base_reaction":"observer_disabled")<<','<<d.samples<<','<<s.totalMass;
        vec(out,s.totalComWorld); vec(out,d.comVelocity); vec(out,d.comVelocityJacobian);
        for(auto w:{d.gravity,d.environment,d.applied,d.total}) { vec(out,w.force); vec(out,w.moment); }
        for(auto v:{d.linearImpulse,d.angularImpulse,d.linearImpulseEndpoint,d.angularImpulseEndpoint,d.deltaLinearMomentum,d.deltaAngularMomentum,d.linearResidual,d.angularResidual,d.tickLinearImpulse,d.tickAngularImpulse}) vec(out,v);
        out<<','<<s.timing.supportUs<<','<<s.timing.controlUs+s.timing.stepUs+s.timing.telemetryUs+s.timing.readUs<<'\n';
    }
    void supportRegionsCsvHeader(std::ostream& o) { o<<"schema,tick,epoch,owner,region,kind,vertex,x_m,y_m,z_m\n"; }
    void dynamicsSubstepsCsvHeader(std::ostream& o)
    {
        o<<"schema,tick,epoch,time_s,dt_s";
        for(const char* name:{"com","p","L","force_before","moment_before","force","moment"}) for(const char* axis:{"x","y","z"}) o<<','<<name<<'_'<<axis;
        o<<'\n';
    }
    void dynamicsSubstepsCsvRow(std::ostream& out,const Snapshot& s)
    {
        const auto& d=s.support.dynamics; if(!s.support.enabled) return;
        const auto row=[&](const DynamicsObservation::Sample& v) {
            out<<std::setprecision(17)<<1<<','<<s.tick<<','<<s.epoch<<','<<v.time<<','<<v.dt;
            for(auto x:{v.com,v.linearMomentum,v.angularMomentum,v.forceBefore,v.momentBefore,v.force,v.moment}) vec(out,x);
            out<<'\n';
        };
        if(s.tick==0) row({s.time,0,s.totalComWorld,s.linearMomentumWorld,s.angularMomentumAboutComWorld,d.total.force,d.total.moment,d.total.force,d.total.moment});
        else for(const auto& v:d.substeps) row(v);
    }
    void supportRegionsCsvRow(std::ostream& out,const Snapshot& s)
    {
        const auto row=[&](const char* owner,const char* name,const PlanarRegion& r) {
            for(std::size_t k=0;k<r.vertices.size();++k) { out<<std::setprecision(17)<<1<<','<<s.tick<<','<<s.epoch<<','<<owner<<','<<name<<','<<regionName(r.kind)<<','<<k; vec(out,r.vertices[k]); out<<'\n'; }
        };
        row("combined","sole",s.support.soleProjection); row("combined","loaded",s.support.loadedRegion);
        for(std::size_t i=0;i<2;++i) { row(i==0?"LeftFoot":"RightFoot","sole",s.support.feet[i].soleProjection); row(i==0?"LeftFoot":"RightFoot","loaded",s.support.feet[i].loadedRegion); }
    }
}
