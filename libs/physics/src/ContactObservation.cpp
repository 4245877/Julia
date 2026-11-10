#include "julia/physics/PhysicsWorld.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>

namespace julia::physics
{
    PairObservation aggregateContacts(const std::vector<Contact>& contacts,int tool,int surface,glm::dvec3 normal)
    {
        PairObservation o; o.toolGeom=tool; o.surfaceGeom=surface; o.normalWorld=normal;
        for(const auto& c:contacts)
        {
            const bool selected=(c.geom1==tool&&c.geom2==surface)||(c.geom2==tool&&c.geom1==surface);
            const bool active=c.active&&c.force&&c.force->x>1e-9;
            if(!selected) { if(active) ++o.foreignActive; continue; }
            ++o.records; o.near=o.near||c.distance<=.002;
            o.penetration=std::max(o.penetration,-c.distance);
            o.friction=c.friction[0];
            if(!active) continue;
            ++o.active;
            const double sign=c.geom2==tool?1.:-1.;
            const auto force=sign*c.forceOn2World;
            const auto relative=sign*(c.velocity2World-c.velocity1World);
            const auto tangentV=relative-normal*glm::dot(relative,normal);
            o.pointWorld+=c.point; o.forceOnToolWorld+=force;
            o.normalForce+=glm::dot(force,normal);
            o.tangentForceWorld+=force-normal*glm::dot(force,normal);
            o.relativeTangentVelocityWorld+=tangentV;
            o.slipSpeed=std::max(o.slipSpeed,glm::length(tangentV));
            // Pair power, each solver point counted ONCE. The equal/opposite
            // forces act at the same world point; include both point velocities.
            const auto fn=c.normal*c.force->x;
            const auto relative21=c.velocity2World-c.velocity1World;
            o.normalPower+=glm::dot(fn,relative21);
            o.tangentPower+=glm::dot(c.forceOn2World-fn,relative21);
        }
        if(o.active) { o.pointWorld/=o.active; o.relativeTangentVelocityWorld/=o.active; }
        return o;
    }
    namespace
    {
        void csvVec(std::ostream& o,glm::dvec3 v) { o<<','<<v.x<<','<<v.y<<','<<v.z; }
    }
    void contactCsvHeader(std::ostream& out)
    {
        out<<"schema,tick,epoch,time_s,phase,failure,sample_phase,tool_geom,surface_geom,records,active,near,foreign_active,confirmed,hold_success,"
            "goal_N,reference_N,normal_N,filtered_N,feedback_N,feedback_time_s,sample_feedback_N,sample_feedback_time_s,sample_reference_N,"
            "force_x_N,force_y_N,force_z_N,tangent_x_N,tangent_y_N,tangent_z_N,normal_x,normal_y,normal_z,"
            "tool_x_m,tool_y_m,tool_z_m,tool_qw,tool_qx,tool_qy,tool_qz,tool_vx_m_s,tool_vy_m_s,tool_vz_m_s,slip_vx_m_s,slip_vy_m_s,slip_vz_m_s,slip_speed_m_s,"
            "penetration_m,geometric_gap_m,friction,friction_ratio_valid,tangent_normal_ratio,normal_W,tangent_W,normal_J,tangent_J,slip_distance_m,"
            "phase_s,search_m,integral_N_s,confirm_s,loss_s,good_hold_s,saturated,saturation_s,peak_N,peak_penetration_m\n";
    }
    void contactCsvRow(std::ostream& out,const Snapshot& s)
    {
        const auto& c=s.contact; const auto& p=c.pair;
        out<<std::setprecision(17)<<1<<','<<s.tick<<','<<s.epoch<<','<<s.time<<','<<contactPhaseName(c.phase)<<','<<contactFailureName(c.failure)
            <<','<<contactPhaseName(c.samplePhase)<<','<<p.toolGeom<<','<<p.surfaceGeom<<','<<p.records<<','<<p.active<<','<<p.near<<','<<p.foreignActive
            <<','<<c.confirmed<<','<<c.holdSucceeded<<','<<c.goalForce<<','<<c.referenceForce<<','<<p.normalForce<<','<<c.filteredForce
            <<','<<c.feedbackForce<<','<<c.feedbackTime<<','<<c.sampleFeedbackForce<<','<<c.sampleFeedbackTime<<','<<c.sampleReferenceForce;
        csvVec(out,p.forceOnToolWorld); csvVec(out,p.tangentForceWorld); csvVec(out,c.normalWorld); csvVec(out,c.toolPointWorld);
        const auto orientation=glm::normalize(glm::quat_cast(glm::dmat3{c.worldFromTool}));
        out<<','<<orientation.w<<','<<orientation.x<<','<<orientation.y<<','<<orientation.z;
        csvVec(out,c.toolVelocityWorld); csvVec(out,p.relativeTangentVelocityWorld);
        out<<','<<p.slipSpeed<<','<<p.penetration<<','<<p.geometricGap<<','<<p.friction<<','<<(p.normalForce>=.6)<<',';
        if(p.normalForce>=.6) out<<glm::length(p.tangentForceWorld)/p.normalForce; else out<<"NA";
        out<<','<<p.normalPower<<','<<p.tangentPower<<','<<c.normalWork<<','<<c.tangentWork<<','<<c.slipDistance<<','<<c.phaseSeconds
            <<','<<c.searchDisplacement<<','<<c.integral<<','<<c.confirmationSeconds<<','<<c.lossSeconds<<','<<c.holdGoodSeconds
            <<','<<c.saturated<<','<<c.saturationSeconds<<','<<c.peakForce<<','<<c.peakPenetration<<'\n';
    }
    void contactPointsCsvHeader(std::ostream& out)
    {
        out<<"schema,tick,epoch,time_s,geom1,geom2,selected_pair,active,distance_m,x_m,y_m,z_m,"
            "nx,ny,nz,t1x,t1y,t1z,t2x,t2y,t2z,fn_N,ft1_N,ft2_N,"
            "f2x_N,f2y_N,f2z_N,f1x_N,f1y_N,f1z_N,t2x_Nm,t2y_Nm,t2z_Nm,"
            "v1x_m_s,v1y_m_s,v1z_m_s,v2x_m_s,v2y_m_s,v2z_m_s,"
            "w1x_rad_s,w1y_rad_s,w1z_rad_s,w2x_rad_s,w2y_rad_s,w2z_rad_s,mu1,mu2,spin_mu,roll_mu1,roll_mu2\n";
    }
    void contactPointsCsvRow(std::ostream& out,const Snapshot& s)
    {
        out<<std::setprecision(17);
        for(const auto& c:s.contacts)
        {
            const auto& p=s.contact.pair;
            const bool selected=(c.geom1==p.toolGeom&&c.geom2==p.surfaceGeom)||(c.geom2==p.toolGeom&&c.geom1==p.surfaceGeom);
            out<<1<<','<<s.tick<<','<<s.epoch<<','<<s.time<<','<<c.geom1<<','<<c.geom2<<','<<selected<<','<<c.active<<','<<c.distance;
            csvVec(out,c.point); for(int k=0;k<3;++k) csvVec(out,c.worldFromContact[k]);
            csvVec(out,c.force.value_or(glm::dvec3{0})); csvVec(out,c.forceOn2World); csvVec(out,-c.forceOn2World); csvVec(out,c.torqueOn2World);
            csvVec(out,c.velocity1World); csvVec(out,c.velocity2World); csvVec(out,c.angular1World); csvVec(out,c.angular2World);
            for(double mu:c.friction) out<<','<<mu;
            out<<'\n';
        }
    }
}
