#pragma once
#include "julia/embodiment/HandInteraction.hpp"

namespace julia::embodiment
{
    class OpenHandSurface
    {
    public:
        explicit OpenHandSurface(std::vector<std::array<glm::vec3,3>> triangles);
        glm::vec3 closestPoint(glm::vec3 center) const;
        float radius() const { return radius_; } // About the wrist.
    private:
        struct Face { std::array<glm::vec3,3> points; glm::vec3 min,max; };
        std::vector<Face> faces_;
        float radius_=0;
    };

    // Contact uses the same weighted skin positions as rendering. Planning works in
    // wrist space so a settled arm pose or a carried rigid transform cannot move a pad
    // relative to the sphere.
    class HandSphereContact
    {
    public:
        HandSphereContact(const BodyRigBinding& binding,const HandLayer& hand);
        const auto& openSurface() const { return openSurface_; }
        // Called while the aligned hand is stationary. Each tick does a bounded
        // batch of digit trials or checks one sample of the combined stroke.
        std::optional<SphereHandStroke> advancePlan(glm::vec3 center,float radius);
        void cancelPlan() { plan_.reset(); }
        SphereContacts measure(std::span<const glm::mat4> globals,glm::vec3 center,float radius,
            bool releasing=false);
    private:
        struct Triangle { std::array<std::size_t,3> vertices; int region,pad; };
        struct Closest { float squared; glm::vec3 point{0}; };
        struct Plan
        {
            glm::vec3 center;
            float radius;
            SphereHandStroke stroke;
            std::size_t digit=0;
            unsigned trial=0, refinements=0, pathSample=0;
            float lower=0, upper=0;
        };
        const BodyRigBinding& binding_;
        const HandLayer& hand_;
        animation::Pose pose_;
        std::vector<const model::Vertex*> vertices_;
        std::vector<Triangle> triangles_;
        std::vector<glm::vec3> points_;
        std::vector<glm::mat4> skin_;
        std::vector<std::size_t> skinBones_;
        std::array<std::vector<std::size_t>,6> regions_;
        std::array<std::vector<std::size_t>,5> digitVertices_;
        std::array<glm::vec3,5> padAxes_;
        std::array<glm::vec3,5> padNormals_;
        std::shared_ptr<const OpenHandSurface> openSurface_;
        std::optional<Plan> plan_;
        void skin(const HandPose& pose,std::optional<std::size_t> digit={});
        void skin(std::span<const glm::mat4> globals,std::optional<std::size_t> digit={});
        void checkTriangle(const Triangle& triangle,glm::vec3 center,Closest& closest) const;
        SphereContacts contacts(glm::vec3 center,float radius,bool releasing=false) const;
        void advanceDigits(Plan& plan);
        bool fingersClear() const;
    };
}
