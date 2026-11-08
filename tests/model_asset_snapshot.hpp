#pragma once

// Semantic fields only: no padding, addresses, source paths or GL object names.
// The stage-2 golden file was captured from the pre-refactor GL importer.
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <bit>
#include <cstdint>
#include <ostream>
#include <span>
#include <string>
#include <type_traits>

namespace model_snapshot
{
    struct Hash
    {
        std::uint64_t value = 14695981039346656037ull;
        void add(std::uint64_t n)
        {
            for (int i = 0; i < 8; ++i) { value ^= (n >> (i * 8)) & 255; value *= 1099511628211ull; }
        }
        template<class T> requires std::is_integral_v<T> void add(T n) { add(static_cast<std::uint64_t>(n)); }
        template<class T> requires std::is_enum_v<T> void add(T n) { add(static_cast<std::uint64_t>(n)); }
        void add(float n) { add(std::bit_cast<std::uint32_t>(n)); }
        void add(double n) { add(std::bit_cast<std::uint64_t>(n)); }
        void add(const std::string& s) { add(s.size()); for (unsigned char c : s) add(c); }
        template<glm::length_t L, class T, glm::qualifier Q> void add(const glm::vec<L,T,Q>& v)
        { for (glm::length_t i = 0; i < L; ++i) add(v[i]); }
        void add(const glm::quat& q) { add(q.w); add(q.x); add(q.y); add(q.z); }
        void add(const glm::mat4& m) { for (int i = 0; i < 4; ++i) add(m[i]); }
    };

    template<class Vertices, class Indices, class Material, class ReadImage>
    void mesh(std::ostream& out, const Vertices& vertices, const Indices& indices,
        const Material& m, ReadImage readImage)
    {
        Hash geometry, material, image;
        geometry.add(vertices.size()); geometry.add(indices.size());
        for (const auto& v : vertices)
        { geometry.add(v.position); geometry.add(v.normal); geometry.add(v.texCoord);
          geometry.add(v.boneIds); geometry.add(v.boneWeights); }
        for (const auto i : indices) geometry.add(i);
        material.add(m.diffuseColor); material.add(m.lightResponse); material.add(m.specularStrength);
        material.add(m.rimStrength); material.add(m.opacity); material.add(m.alphaCutoff);
        material.add(m.alphaMode); material.add(m.wrapS); material.add(m.wrapT);
        material.add(m.minFilter); material.add(m.magFilter); material.add(m.toonKind);
        material.add(m.specularPower); material.add(m.shadowSoftness);
        material.add(m.strandDirection); material.add(m.outlineScale);
        readImage(image);
        out << "mesh " << vertices.size() << ' ' << indices.size() << ' '
            << geometry.value << ' ' << material.value << ' ' << image.value << '\n';
    }

    template<class Model> void rig(std::ostream& out, const Model& model)
    {
        Hash skeleton, clips;
        for (const auto& n : model.nodes())
        { skeleton.add(n.name); skeleton.add(n.parent); skeleton.add(n.localTransform);
          skeleton.add(n.globalBindTransform); skeleton.add(n.bindTRS.translation);
          skeleton.add(n.bindTRS.rotation); skeleton.add(n.bindTRS.scale); }
        for (const auto& b : model.bones())
        { skeleton.add(b.name); skeleton.add(b.nodeIndex); skeleton.add(b.offsetMatrix); }
        for (const auto& c : model.animations())
        {
            clips.add(c.name); clips.add(c.durationTicks); clips.add(c.ticksPerSecond);
            clips.add(c.usedDefaultTicksPerSecond); clips.add(c.channels.size());
            for (const auto& channel : c.channels)
            {
                clips.add(channel.nodeIndex);
                const auto keys = [&](const auto& values) { clips.add(values.size());
                    for (const auto& k : values) { clips.add(k.ticks); clips.add(k.value); } };
                keys(channel.translations); keys(channel.rotations); keys(channel.scales);
            }
        }
        out << "rig " << model.nodes().size() << ' ' << model.bones().size() << ' '
            << model.animations().size() << ' ' << skeleton.value << ' ' << clips.value << '\n';
    }

    template<class Player, class Eyes> void eyes(std::ostream& out, Player& player, Eyes& eyes)
    {
        Hash h;
        // This immutable stage-2 reference includes its old clock contract.
        // Drive those clocks explicitly in this test, keeping the production
        // independent-eye contract covered by body_instance/desktop_eyes.
        // No reference hashes or tolerances are changed.
        player.setEyeLayer(nullptr);
        julia::animation::Pose pose(eyes.asset().skeleton());
        for (std::size_t c = 0; player.clip() && c < 15; ++c)
        {
            eyes.behavior().reset(); eyes.clearTargetPoint();
            player.selectClip(c); player.seek(.37); player.play();
            eyes.behavior().setBlink(.65f, .2f);
            eyes.setTargetPoint({10,150,300});
            for (int step = 0; step < 12; ++step)
            {
                if (step == 4) player.pause();
                if (step == 6) player.seek(.8);
                if (step == 8) player.stop();
                const auto* base = player.state() == julia::embodiment::PlaybackState::Stopped ? nullptr : player.clip();
                if (player.state() != julia::embodiment::PlaybackState::Paused && !Eyes::authoredEyes(base))
                    eyes.update(1.0 / 60);
                player.update(1.0 / 60);
                std::copy(player.localTransforms().begin(),player.localTransforms().end(),pose.locals.begin());
                if (!Eyes::authoredEyes(base)) eyes.apply(pose.locals,base);
                julia::animation::computeGlobalTransforms(eyes.asset().skeleton(),pose.locals,pose.globals);
                julia::animation::buildSkinningPalette(eyes.asset().skeleton(),pose.globals,pose.palette);
                h.add(player.timeSeconds());
                for (const auto& m : pose.locals) h.add(m);
                for (const auto& m : pose.globals) h.add(m);
                for (const auto& m : pose.palette) h.add(m);
            }
        }
        out << "eyes " << eyes.available() << ' ' << eyes.lidCount() << ' ' << h.value << '\n';
    }
}
