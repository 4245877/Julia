#pragma once
#include "julia/physics/PhysicsWorld.hpp"
#include <mujoco/mujoco.h>
#include "BoundedQp.hpp"

namespace julia::physics
{
    // Fixed sizes for the explicitly supported BodyRig. No allocation per update.
    class StandControl
    {
    public:
        void reset();
        void start(const StandSettings& settings,const Snapshot& state,const Definition& definition);
        void disable();
        void balanceFeedback(bool enabled) { settings_.balanceFeedback=enabled; status.balanceFeedback=enabled; }
        bool target(glm::dvec2 offset,std::string* reason);
        bool cancel(std::string* reason);
        Coordinates evaluate(const mjModel* model,mjData* data,const StateLayout& layout,
            const Definition& definition,Snapshot& state,const std::vector<int>& bodies,
            const std::array<int,2>& footLinks,double dt,bool update);
        StandStatus status;
    private:
        StandSettings settings_;
        std::array<glm::dmat4,2> initialFeet_{};
        std::array<double,12*47> previousJ_{};
        std::array<double,3*47> previousComJ_{};
        Coordinates held_{};
        bool previous_=false;
        double start_=0,lastUpdate_=0,rest_=0;
        glm::dvec3 initialCom_{0},reference_{0};
        std::array<glm::dvec3,6> curve_{};
        double curveTime_=0,curveDuration_=0;
        bool cancelled_=false;
        qp::Solver solver_;
        qp::Problem problem_;
        bool plan(glm::dvec3 goal,bool cancel,std::string* reason);
        void advanceReference(double dt);
        bool bounded(const mjModel* model,const mjData* data,const StateLayout& layout,const Definition& def,
            const std::array<double,47*47>& mass,const std::array<double,12*47>& j,
            const std::array<double,59*59>& cost,const std::array<double,59>& task,
            const std::array<double,12>& contact,const std::array<double,12>& jdot,
            const std::array<glm::dvec3,2>& points,const std::array<glm::dmat3,2>& rotations);
        void lost(const char* reason);
    };
}
