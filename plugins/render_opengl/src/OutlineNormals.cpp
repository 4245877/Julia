#include "julia/render_opengl/OutlineNormals.hpp"
#include "julia/render_opengl/SurfaceNormals.hpp"

#include "julia/render_opengl/GLMesh.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <unordered_map>

namespace julia::render_opengl
{
    namespace
    {
        struct WeldKey
        {
            std::array<std::uint32_t, 3> position{};
            std::array<int, 4> boneIds{-1, -1, -1, -1};
            std::array<std::uint32_t, 4> boneWeights{};

            bool operator==(const WeldKey&) const = default;
        };

        struct WeldHash
        {
            std::size_t operator()(const WeldKey& key) const noexcept
            {
                std::size_t value = 0;
                const auto append = [&](std::size_t part) {
                    value ^= part + std::size_t{0x9e3779b9u} + (value << 6) + (value >> 2);
                };
                for (const auto part : key.position) append(part);
                for (const auto part : key.boneIds) append(static_cast<std::uint32_t>(part));
                for (const auto part : key.boneWeights) append(part);
                return value;
            }
        };

        std::uint32_t floatKey(float value)
        {
            // Signed zero is one position/influence, not a geometric seam.
            return value == 0.0f ? 0u : std::bit_cast<std::uint32_t>(value);
        }

        WeldKey positionKey(const GLVertex& vertex)
        {
            WeldKey key;
            for (int component = 0; component < 3; ++component)
            {
                if (!std::isfinite(vertex.position[component]))
                    throw std::invalid_argument("Outline vertex position is not finite");
                key.position[static_cast<std::size_t>(component)] = floatKey(vertex.position[component]);
            }
            return key;
        }

        WeldKey weldKey(const GLVertex& vertex)
        {
            auto key = positionKey(vertex);

            struct Influence { int id; double weight; };
            std::array<Influence, 4> influences{};
            std::size_t count = 0;
            for (int component = 0; component < 4; ++component)
            {
                const int id = vertex.boneIds[component];
                const float weight = vertex.boneWeights[component];
                if (id >= 0 && weight > 0.0f && std::isfinite(weight))
                    influences[count++] = {id, static_cast<double>(weight)};
            }
            std::sort(influences.begin(), influences.begin() + static_cast<std::ptrdiff_t>(count),
                [](const Influence& a, const Influence& b) {
                    return a.id == b.id ? a.weight < b.weight : a.id < b.id;
                });

            std::array<Influence, 4> combined{};
            std::size_t combinedCount = 0;
            double total = 0.0;
            for (std::size_t i = 0; i < count; ++i)
            {
                const auto influence = influences[i];
                if (combinedCount != 0 && combined[combinedCount - 1].id == influence.id)
                    combined[combinedCount - 1].weight += influence.weight;
                else
                    combined[combinedCount++] = influence;
                total += influence.weight;
            }
            std::size_t destination = 0;
            for (std::size_t i = 0; i < combinedCount; ++i)
            {
                const float weight = static_cast<float>(combined[i].weight / total);
                if (weight > 0.0f)
                {
                    key.boneIds[destination] = combined[i].id;
                    key.boneWeights[destination] = floatKey(weight);
                    ++destination;
                }
            }
            return key;
        }

        glm::vec3 fallbackNormal(const glm::vec3& normal)
        {
            const glm::dvec3 value{normal};
            const double squared = glm::dot(value, value);
            if (std::isfinite(squared) && squared > 0.0)
                return glm::vec3{value / std::sqrt(squared)};
            return {0.0f, 1.0f, 0.0f};
        }

        struct Accumulation
        {
            glm::dvec3 normal{0.0};
            double angle = 0.0;
        };
    }

    std::vector<glm::vec3> buildOutlineNormals(
        std::span<const GLVertex> vertices,
        std::span<const std::uint32_t> indices)
    {
        if (indices.size() % 3 != 0)
            throw std::invalid_argument("Outline triangle index count must be divisible by three");
        for (const auto index : indices)
            if (index >= vertices.size())
                throw std::out_of_range("Outline triangle references a missing vertex");

        std::unordered_map<WeldKey, std::size_t, WeldHash> groups;
        groups.reserve(vertices.size());
        std::vector<std::size_t> groupOfVertex;
        groupOfVertex.reserve(vertices.size());
        std::vector<Accumulation> accumulated;
        accumulated.reserve(vertices.size());
        for (const auto& vertex : vertices)
        {
            const auto [entry, inserted] = groups.try_emplace(weldKey(vertex), accumulated.size());
            if (inserted) accumulated.emplace_back();
            groupOfVertex.push_back(entry->second);
        }

        for (std::size_t triangle = 0; triangle < indices.size(); triangle += 3)
        {
            const std::array<std::uint32_t, 3> corners{
                indices[triangle], indices[triangle + 1], indices[triangle + 2]};
            const std::array<glm::dvec3, 3> positions{
                glm::dvec3{vertices[corners[0]].position},
                glm::dvec3{vertices[corners[1]].position},
                glm::dvec3{vertices[corners[2]].position}};
            // Double precision avoids overflow/underflow for valid float meshes.
            const auto face = glm::cross(positions[1] - positions[0], positions[2] - positions[0]);
            const double faceSquared = glm::dot(face, face);
            if (!(faceSquared > 0.0) || !std::isfinite(faceSquared)) continue;
            const auto direction = face / std::sqrt(faceSquared);

            for (std::size_t corner = 0; corner < 3; ++corner)
            {
                const auto first = positions[(corner + 1) % 3] - positions[corner];
                const auto second = positions[(corner + 2) % 3] - positions[corner];
                const auto cross = glm::cross(first, second);
                const double angle = std::atan2(std::sqrt(glm::dot(cross, cross)), glm::dot(first, second));
                auto& sum = accumulated[groupOfVertex[corners[corner]]];
                sum.normal += direction * angle;
                sum.angle += angle;
            }
        }

        std::vector<glm::vec3> result;
        result.reserve(vertices.size());
        for (std::size_t vertex = 0; vertex < vertices.size(); ++vertex)
        {
            const auto& sum = accumulated[groupOfVertex[vertex]];
            const double squared = glm::dot(sum.normal, sum.normal);
            // Opposing coincident surfaces have no unique hull direction.
            if (std::isfinite(squared) && squared > sum.angle * sum.angle * 1e-24)
                result.emplace_back(sum.normal / std::sqrt(squared));
            else
                result.push_back(fallbackNormal(vertices[vertex].normal));
        }
        return result;
    }

    void rebuildSurfaceNormals(std::vector<GLVertex>& vertices,
        std::vector<std::uint32_t>& indices, float creaseAngleDegrees)
    {
        if (!std::isfinite(creaseAngleDegrees) || creaseAngleDegrees <= 0.0f || creaseAngleDegrees > 180.0f)
            throw std::invalid_argument("Surface crease angle must be in (0, 180]");
        if (indices.size() % 3 != 0)
            throw std::invalid_argument("Surface triangle index count must be divisible by three");
        for (const auto index : indices)
            if (index >= vertices.size())
                throw std::out_of_range("Surface triangle references a missing vertex");

        std::unordered_map<WeldKey, std::size_t, WeldHash> groups;
        std::vector<std::size_t> groupOfVertex;
        groups.reserve(vertices.size());
        groupOfVertex.reserve(vertices.size());
        for (const auto& vertex : vertices)
        {
            // Lighting normals describe the rest surface. UV copies can have
            // different bone weights while sharing the same rest normal; the
            // shader transforms each copy independently during animation.
            const auto [entry, inserted] = groups.try_emplace(positionKey(vertex), groups.size());
            (void)inserted;
            groupOfVertex.push_back(entry->second);
        }

        struct Edge
        {
            std::size_t a, b;
            bool operator==(const Edge&) const = default;
        };
        struct EdgeHash
        {
            std::size_t operator()(const Edge& edge) const noexcept
            {
                return edge.a ^ (edge.b + std::size_t{0x9e3779b9u} + (edge.a << 6) + (edge.a >> 2));
            }
        };
        struct HalfEdge
        {
            std::size_t a, b;
        };
        std::unordered_map<Edge, std::vector<HalfEdge>, EdgeHash> edges;
        edges.reserve(indices.size());
        std::vector<glm::dvec3> directions(indices.size() / 3, glm::dvec3{0.0});
        std::vector<double> angles(indices.size(), 0.0);
        std::vector<std::size_t> parent(indices.size());
        std::iota(parent.begin(), parent.end(), std::size_t{0});
        const auto root = [&](std::size_t corner) {
            while (parent[corner] != corner)
            {
                parent[corner] = parent[parent[corner]];
                corner = parent[corner];
            }
            return corner;
        };
        const auto unite = [&](std::size_t a, std::size_t b) {
            a = root(a);
            b = root(b);
            if (a != b) parent[b] = a;
        };

        for (std::size_t triangle = 0; triangle < indices.size(); triangle += 3)
        {
            const std::array<glm::dvec3, 3> positions{
                glm::dvec3{vertices[indices[triangle]].position},
                glm::dvec3{vertices[indices[triangle + 1]].position},
                glm::dvec3{vertices[indices[triangle + 2]].position}};
            const auto cross = glm::cross(positions[1] - positions[0], positions[2] - positions[0]);
            const double squared = glm::dot(cross, cross);
            if (!(squared > 0.0) || !std::isfinite(squared)) continue;
            directions[triangle / 3] = cross / std::sqrt(squared);
            for (std::size_t corner = 0; corner < 3; ++corner)
            {
                const auto first = positions[(corner + 1) % 3] - positions[corner];
                const auto second = positions[(corner + 2) % 3] - positions[corner];
                const auto angleCross = glm::cross(first, second);
                angles[triangle + corner] = std::atan2(std::sqrt(glm::dot(angleCross, angleCross)), glm::dot(first, second));
                const auto a = triangle + corner;
                const auto b = triangle + (corner + 1) % 3;
                const auto groupA = groupOfVertex[indices[a]], groupB = groupOfVertex[indices[b]];
                edges[{std::min(groupA, groupB), std::max(groupA, groupB)}].push_back({a, b});
            }
        }

        const double creaseCosine = std::cos(glm::radians(static_cast<double>(creaseAngleDegrees)));
        for (const auto& [edge, incident] : edges)
        {
            (void)edge;
            // Ambiguous non-manifold and coincident double-sided surfaces have
            // no unique smoothing fan. Do not borrow normals from those layers.
            if (incident.size() != 2) continue;
            const auto a = incident[0], b = incident[1];
            if (groupOfVertex[indices[a.a]] != groupOfVertex[indices[b.b]] ||
                groupOfVertex[indices[a.b]] != groupOfVertex[indices[b.a]]) continue;
            if (glm::dot(directions[a.a / 3], directions[b.a / 3]) <= creaseCosine) continue;
            unite(a.a, b.b);
            unite(a.b, b.a);
        }

        std::vector<glm::dvec3> sums(indices.size(), glm::dvec3{0.0});
        for (std::size_t corner = 0; corner < indices.size(); ++corner)
            sums[root(corner)] += directions[corner / 3] * angles[corner];

        // Keep original vertices for the first fan, only duplicate where a
        // single index previously straddled a crease or disconnected surface.
        const auto original = vertices;
        std::vector<std::unordered_map<std::size_t, std::uint32_t>> fans(vertices.size());
        for (auto& vertex : vertices) vertex.normal = fallbackNormal(vertex.normal);
        for (std::size_t corner = 0; corner < indices.size(); ++corner)
        {
            const auto source = indices[corner];
            const auto fan = root(corner);
            auto& destinations = fans[source];
            const auto existing = destinations.find(fan);
            if (existing != destinations.end())
            {
                indices[corner] = existing->second;
                continue;
            }
            std::uint32_t destination = source;
            if (!destinations.empty())
            {
                if (vertices.size() >= std::numeric_limits<std::uint32_t>::max())
                    throw std::length_error("Surface normal repair exceeds the vertex index limit");
                destination = static_cast<std::uint32_t>(vertices.size());
                vertices.push_back(original[source]);
            }
            const auto& sum = sums[fan];
            const double squared = glm::dot(sum, sum);
            vertices[destination].normal = squared > 1e-24 && std::isfinite(squared)
                ? glm::vec3{sum / std::sqrt(squared)} : fallbackNormal(original[source].normal);
            destinations.emplace(fan, destination);
            indices[corner] = destination;
        }
    }
}
