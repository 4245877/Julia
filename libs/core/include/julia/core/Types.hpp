#pragma once

#include <cstddef>
#include <cstdint>

namespace julia::core
{
    using std::int8_t;
    using std::int16_t;
    using std::int32_t;
    using std::int64_t;

    using std::uint8_t;
    using std::uint16_t;
    using std::uint32_t;
    using std::uint64_t;

    using std::size_t;

    using f32 = float;
    using f64 = double;

    // Время в секундах, используется для шага симуляции.
    using Seconds = f64;
}
