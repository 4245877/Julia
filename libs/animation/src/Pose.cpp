#include "julia/animation/Pose.hpp"
#include "MatrixValidation.hpp"
#include <stdexcept>

namespace julia::animation
{
    Pose::Pose(const Skeleton& skeleton) : locals(skeleton.nodes().size()),
        globals(skeleton.nodes().size()), palette(skeleton.bindings().size())
    {
        setBindPose(skeleton, locals);
        computeGlobalTransforms(skeleton, locals, globals);
        buildSkinningPalette(skeleton, globals, palette);
    }

    void setBindPose(const Skeleton& skeleton, std::span<glm::mat4> locals)
    {
        if (locals.size() != skeleton.nodes().size())
            throw std::invalid_argument("Bind pose must provide one matrix per node");
        for (std::size_t i = 0; i < locals.size(); ++i)
            locals[i] = skeleton.nodes()[i].localTransform;
    }

    void computeGlobalTransforms(const Skeleton& skeleton,
        std::span<const glm::mat4> locals, std::span<glm::mat4> globals)
    {
        const auto& nodes = skeleton.nodes();
        if ((!locals.empty() && locals.size() != nodes.size()) || globals.size() != nodes.size())
            throw std::invalid_argument("Incorrect local/global pose size");
        for (std::size_t i = 0; i < nodes.size(); ++i)
        {
            const auto& local = locals.empty() ? nodes[i].localTransform : locals[i];
            if (!detail::finite(local))
                throw std::invalid_argument("Pose contains a non-finite transform");
            const auto global = nodes[i].parent == NoNode ? local : globals[nodes[i].parent] * local;
            if (!detail::finite(global))
                throw std::invalid_argument("Pose global transform is not finite");
            globals[i] = global;
        }
    }

    void buildSkinningPalette(const Skeleton& skeleton,
        std::span<const glm::mat4> globals, std::span<glm::mat4> palette)
    {
        if (globals.size() != skeleton.nodes().size() || palette.size() != skeleton.bindings().size())
            throw std::invalid_argument("Incorrect global pose/palette size");
        for (std::size_t i = 0; i < palette.size(); ++i)
        {
            const auto& binding = skeleton.bindings()[i];
            const auto matrix = globals[binding.nodeIndex] * binding.offsetMatrix;
            if (!detail::finite(matrix))
                throw std::invalid_argument("Skinning matrix is not finite: " + binding.name);
            palette[i] = matrix;
        }
    }
}
