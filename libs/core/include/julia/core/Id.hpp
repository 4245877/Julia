#pragma once

#include "julia/core/Types.hpp"

#include <functional>

namespace julia::core
{
    // Сильно типизированный идентификатор. Параметр Tag делает Id<A>
    // несовместимым с Id<B>, что исключает случайное смешивание разных
    // видов сущностей при одинаковом базовом целочисленном типе.
    template <typename Tag, typename Value = uint64_t>
    class Id
    {
    public:
        using value_type = Value;

        static constexpr Value Invalid = 0;

        constexpr Id() noexcept = default;
        constexpr explicit Id(Value value) noexcept : value_(value) {}

        [[nodiscard]] constexpr Value value() const noexcept { return value_; }
        [[nodiscard]] constexpr bool valid() const noexcept { return value_ != Invalid; }

        [[nodiscard]] constexpr bool operator==(const Id& other) const noexcept
        {
            return value_ == other.value_;
        }

        [[nodiscard]] constexpr bool operator!=(const Id& other) const noexcept
        {
            return value_ != other.value_;
        }

        [[nodiscard]] constexpr bool operator<(const Id& other) const noexcept
        {
            return value_ < other.value_;
        }

    private:
        Value value_ = Invalid;
    };
}

namespace std
{
    template <typename Tag, typename Value>
    struct hash<julia::core::Id<Tag, Value>>
    {
        std::size_t operator()(const julia::core::Id<Tag, Value>& id) const noexcept
        {
            return std::hash<Value>{}(id.value());
        }
    };
}
