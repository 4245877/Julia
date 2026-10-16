#include "julia/render_opengl/GLMesh.hpp"
#include "julia/render_opengl/OutlineNormals.hpp"
#include "julia_test.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace
{
    using namespace julia::render_opengl;

    bool close(const glm::vec3& actual, const glm::vec3& expected)
    {
        return glm::length(actual - expected) < 0.0001f;
    }

    template<class Exception, class Callable>
    bool throws(Callable&& call)
    {
        try { call(); }
        catch (const Exception&) { return true; }
        return false;
    }

    void testHardCube()
    {
        // Each face has distinct vertices and hard lighting normals. The two
        // triangles on a face must contribute the same total corner angle as
        // an undivided face, rather than biasing diagonal corners by count.
        const std::array<std::array<glm::vec3, 4>, 6> faces{{
            {{{-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1}}},
            {{{1, -1, -1}, {-1, -1, -1}, {-1, 1, -1}, {1, 1, -1}}},
            {{{-1, -1, -1}, {-1, -1, 1}, {-1, 1, 1}, {-1, 1, -1}}},
            {{{1, -1, 1}, {1, -1, -1}, {1, 1, -1}, {1, 1, 1}}},
            {{{-1, 1, 1}, {1, 1, 1}, {1, 1, -1}, {-1, 1, -1}}},
            {{{-1, -1, -1}, {1, -1, -1}, {1, -1, 1}, {-1, -1, 1}}}
        }};
        std::vector<GLVertex> vertices;
        std::vector<std::uint32_t> indices;
        for (const auto& face : faces)
        {
            const auto base = static_cast<std::uint32_t>(vertices.size());
            const auto normal = glm::normalize(glm::cross(face[1] - face[0], face[2] - face[0]));
            for (const auto& position : face)
            {
                GLVertex vertex;
                vertex.position = position;
                vertex.normal = normal;
                vertices.push_back(vertex);
            }
            indices.insert(indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
        }
        const auto original = vertices;
        const auto normals = buildOutlineNormals(vertices, indices);
        JULIA_CHECK(normals.size() == vertices.size());
        for (std::size_t i = 0; i < vertices.size(); ++i)
        {
            JULIA_CHECK(close(normals[i], glm::normalize(vertices[i].position)));
            JULIA_CHECK(vertices[i].normal == original[i].normal);
            JULIA_CHECK(vertices[i].position == original[i].position);
        }
    }

    std::vector<GLVertex> seamVertices()
    {
        std::vector<GLVertex> vertices(6);
        // Two faces of very different area meet at 90 degrees. The duplicate
        // origins carry unrelated UVs and hard normals, but one hull direction.
        vertices[0].position = {0.0f, 0.0f, 0.0f};
        vertices[1].position = {10.0f, 0.0f, 0.0f};
        vertices[2].position = {0.0f, 1.0f, 0.0f};
        vertices[3].position = {-0.0f, 0.0f, 0.0f};
        vertices[4].position = {0.0f, 1.0f, 0.0f};
        vertices[5].position = {0.0f, 0.0f, 1.0f};
        for (std::size_t i = 0; i < vertices.size(); ++i)
            vertices[i].normal = i < 3 ? glm::vec3{0.0f, 0.0f, 1.0f} : glm::vec3{1.0f, 0.0f, 0.0f};
        vertices[0].texCoord = {0.0f, 0.0f};
        vertices[3].texCoord = {1.0f, 1.0f};
        return vertices;
    }

    void testSeamsAndInfluences()
    {
        constexpr std::array<std::uint32_t, 6> indices{0, 1, 2, 3, 4, 5};
        auto vertices = seamVertices();
        const auto expected = glm::normalize(glm::vec3{1.0f, 0.0f, 1.0f});
        auto normals = buildOutlineNormals(vertices, indices);
        JULIA_CHECK(close(normals[0], expected));
        JULIA_CHECK(close(normals[0], normals[3]));
        JULIA_CHECK(vertices[0].texCoord == glm::vec2(0.0f));
        JULIA_CHECK(vertices[3].texCoord == glm::vec2(1.0f));

        // Bone order, duplicate IDs, and non-unit weight sums do not create a
        // seam when the effective deformation is identical.
        vertices[0].boneIds = {2, 1, -1, -1};
        vertices[0].boneWeights = {1.0f, 3.0f, 0.0f, 0.0f};
        vertices[3].boneIds = {1, 2, 1, -1};
        vertices[3].boneWeights = {3.0f, 2.0f, 3.0f, 0.0f};
        normals = buildOutlineNormals(vertices, indices);
        JULIA_CHECK(close(normals[0], expected));
        JULIA_CHECK(close(normals[0], normals[3]));

        // The same position with another bone or another weight ratio can
        // separate in a pose and must not borrow a neighboring hull normal.
        vertices[3].boneIds = {1, 7, -1, -1};
        vertices[3].boneWeights = {0.75f, 0.25f, 0.0f, 0.0f};
        normals = buildOutlineNormals(vertices, indices);
        JULIA_CHECK(close(normals[0], {0.0f, 0.0f, 1.0f}));
        JULIA_CHECK(close(normals[3], {1.0f, 0.0f, 0.0f}));
        vertices[3].boneIds = {1, 2, -1, -1};
        vertices[3].boneWeights = {0.25f, 0.75f, 0.0f, 0.0f};
        normals = buildOutlineNormals(vertices, indices);
        JULIA_CHECK(close(normals[0], {0.0f, 0.0f, 1.0f}));
        JULIA_CHECK(close(normals[3], {1.0f, 0.0f, 0.0f}));
    }

    void testDegenerateAndInvalid()
    {
        std::vector<GLVertex> vertices(4);
        vertices[0].position = {0.0f, 0.0f, 0.0f};
        vertices[1].position = {1.0f, 0.0f, 0.0f};
        vertices[2].position = {2.0f, 0.0f, 0.0f};
        vertices[3].position = {3.0f, 0.0f, 0.0f};
        vertices[0].normal = {2.0f, 0.0f, 0.0f};
        vertices[1].normal = glm::vec3{0.0f};
        vertices[2].normal = {std::numeric_limits<float>::infinity(), 0.0f, 0.0f};
        vertices[3].normal = {0.0f, 0.0f, 4.0f};
        constexpr std::array<std::uint32_t, 3> degenerate{0, 1, 2};
        const auto normals = buildOutlineNormals(vertices, degenerate);
        JULIA_CHECK(close(normals[0], {1.0f, 0.0f, 0.0f}));
        JULIA_CHECK(close(normals[1], {0.0f, 1.0f, 0.0f}));
        JULIA_CHECK(close(normals[2], {0.0f, 1.0f, 0.0f}));
        JULIA_CHECK(close(normals[3], {0.0f, 0.0f, 1.0f}));

        vertices[2].position = {0.0f, 1.0f, 0.0f};
        constexpr std::array<std::uint32_t, 6> opposing{0, 1, 2, 0, 2, 1};
        const auto cancelled = buildOutlineNormals(vertices, opposing);
        JULIA_CHECK(close(cancelled[0], {1.0f, 0.0f, 0.0f}));
        JULIA_CHECK(buildOutlineNormals({}, {}).empty());
        constexpr std::array<std::uint32_t, 1> incomplete{0};
        JULIA_CHECK(throws<std::invalid_argument>([&] { buildOutlineNormals(vertices, incomplete); }));
        constexpr std::array<std::uint32_t, 3> outside{0, 1, 4};
        JULIA_CHECK(throws<std::out_of_range>([&] { buildOutlineNormals(vertices, outside); }));
        vertices[0].position.x = std::numeric_limits<float>::quiet_NaN();
        JULIA_CHECK(throws<std::invalid_argument>([&] { buildOutlineNormals(vertices, degenerate); }));
    }
}

int main()
{
    testHardCube();
    testSeamsAndInfluences();
    testDegenerateAndInvalid();
    return JULIA_TEST_RESULT();
}
