#pragma once
#include "julia/embodiment/ArmReach.hpp"
#include <algorithm>
#include <cmath>

namespace julia::embodiment::pose_ik
{
inline constexpr double angularScale=.15;
        template<class ChainType> float poseError(const ChainType& chain,const std::array<double,6>& q,glm::vec3 goal,glm::quat orientation)
        {
            const auto frame=chain.frame(q);
            const auto r=orientationResidual(orientation,glm::quat_cast(glm::mat3{frame}));
            const glm::dvec3 p{goal-glm::vec3{frame[3]}};
            return static_cast<float>(std::sqrt(glm::dot(p,p)+angularScale*angularScale*glm::dot(r,r)));
        }
        template<class ChainType> bool poseReached(const ChainType& chain,const std::array<double,6>& q,glm::vec3 goal,glm::quat orientation,
            const ReachTarget& target,float fraction)
        {
            const auto frame=chain.frame(q);
            return glm::length(goal-glm::vec3{frame[3]})<=target.tolerance*fraction &&
                glm::length(orientationResidual(orientation,glm::quat_cast(glm::mat3{frame})))<=target.angularTolerance*fraction;
        }
        // Spatial angular Jacobian and log residual are both expressed in Body.
        // Damped normal equations, projected bounds, <=0.15 rad and bounded backtracking.
        template<class ChainType> bool iteratePose(const ChainType& chain,glm::vec3 goal,glm::quat orientation,std::array<double,6>& q,float& error)
        {
            const auto frame=chain.frame(q);
            const auto rotation=orientationResidual(orientation,glm::quat_cast(glm::mat3{frame}));
            const glm::dvec3 position{goal-glm::vec3{frame[3]}};
            const std::array<double,6> residual{position.x,position.y,position.z,
                angularScale*rotation.x,angularScale*rotation.y,angularScale*rotation.z};
            std::array<std::array<double,6>,6> jacobian{};
            for (std::size_t k=0;k<6;++k) {
                auto a=q,b=q; a[k]=std::max(q[k]-.001,chain.lower[k]); b[k]=std::min(q[k]+.001,chain.upper[k]);
                if (a[k]==b[k]) continue;
                const auto fa=chain.frame(a),fb=chain.frame(b);
                const double inverse=1/(b[k]-a[k]);
                const auto p=glm::dvec3{glm::vec3{fb[3]-fa[3]}}*inverse;
                const auto omega=orientationResidual(glm::quat_cast(glm::mat3{fb}),glm::quat_cast(glm::mat3{fa}))*inverse;
                // d Log(Rgoal Ractual^-1) = -Jr^-1(error) * omega.
                // Half-angle formula is finite at pi; series avoids cancellation at zero.
                const double theta=glm::length(rotation);
                const double coefficient=theta<1e-4 ? 1.0/12+theta*theta/720:
                    (1-.5*theta/std::tan(.5*theta))/(theta*theta);
                const auto w=(omega+.5*glm::cross(rotation,omega)+coefficient*glm::cross(rotation,glm::cross(rotation,omega)))*angularScale;
                jacobian[k]={p.x,p.y,p.z,w.x,w.y,w.z};
                double gradient=0; for (std::size_t r=0;r<6;++r) gradient+=jacobian[k][r]*residual[r];
                if ((q[k]<=chain.lower[k]+1e-7 && gradient<0) || (q[k]>=chain.upper[k]-1e-7 && gradient>0)) jacobian[k]={};
            }
            std::array<std::array<double,7>,6> normal{};
            for (std::size_t i=0;i<6;++i) {
                for (std::size_t r=0;r<6;++r) normal[i][6]+=jacobian[i][r]*residual[r];
                for (std::size_t j=0;j<6;++j) for (std::size_t r=0;r<6;++r) normal[i][j]+=jacobian[i][r]*jacobian[j][r];
                normal[i][i]+=.000025;
            }
            for (std::size_t i=0;i<6;++i) {
                std::size_t pivot=i;
                for (std::size_t j=i+1;j<6;++j) if (std::abs(normal[j][i])>std::abs(normal[pivot][i])) pivot=j;
                std::swap(normal[i],normal[pivot]);
                const double divisor=normal[i][i];
                if (!std::isfinite(divisor) || std::abs(divisor)<1e-14) return false;
                for (std::size_t k=i;k<7;++k) normal[i][k]/=divisor;
                for (std::size_t j=0;j<6;++j) if (j!=i) {
                    const double factor=normal[j][i];
                    for (std::size_t k=i;k<7;++k) normal[j][k]-=factor*normal[i][k];
                }
            }
            double largest=.15;
            for (const auto& row:normal) largest=std::max(largest,std::abs(row[6]));
            for (double step=.15/largest;step>=.0001;step*=.5) {
                auto candidate=q;
                for (std::size_t k=0;k<6;++k) candidate[k]=std::clamp(q[k]+step*normal[k][6],chain.lower[k],chain.upper[k]);
                const float e=poseError(chain,candidate,goal,orientation);
                if (e<error-1e-8f) { q=candidate; error=e; return true; }
            }
            return false;
        }
}
