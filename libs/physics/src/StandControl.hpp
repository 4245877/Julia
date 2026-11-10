#pragma once
#include "julia/physics/PhysicsWorld.hpp"
#include <mujoco/mujoco.h>

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
        void lost(const char* reason);
    };
}
