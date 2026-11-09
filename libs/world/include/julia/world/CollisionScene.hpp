#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cstdint>
#include <variant>
#include <vector>
namespace julia::world
{
    struct CollisionFilter { std::uint32_t category = 1, mask = ~0u; };
    bool interacts(CollisionFilter a, CollisionFilter b);
    struct Capsule { glm::dvec3 a{0}, b{0}; double radius = 0; };
    struct Sphere { double radius = 0; };
    struct Box { glm::dvec3 halfExtent{0}; };
    // Solid local half-space y <= 0; outward normal +Y.
    struct Plane {};
    struct SceneObject
    {
        std::uint64_t id = 0;
        glm::dvec3 position{0}; // World metres, rigid pose only.
        glm::dquat rotation{1,0,0,0};
        CollisionFilter filter;
        std::variant<Sphere,Box,Plane> shape;
    };
    class CollisionScene
    {
    public:
        CollisionScene(std::uint64_t version, std::vector<SceneObject> objects);
        CollisionScene& operator=(const CollisionScene&) = delete;
        CollisionScene& operator=(CollisionScene&&) = delete;
        std::uint64_t version() const { return version_; }
        const std::vector<SceneObject>& objects() const { return objects_; }
    private:
        std::uint64_t version_;
        std::vector<SceneObject> objects_;
    };
    struct Clearance
    {
        double gap = 0; // Surface separation outside; overlap indicator inside a box (not penetration depth).
        glm::dvec3 bodyPoint{0}, otherPoint{0}, normal{1,0,0};
    };
    Clearance clearance(const Capsule& a, const Capsule& b);
    Clearance clearance(const Capsule& a, const SceneObject& b);
}
