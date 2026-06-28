#pragma once

#include "julia/core/Types.hpp"

#include <chrono>

namespace julia::core
{
    // Монотонные часы для измерения времени кадра/симуляции.
    // std::steady_clock не подвержен скачкам системного времени.
    class Clock
    {
    public:
        using clock_type = std::chrono::steady_clock;
        using time_point = clock_type::time_point;

        Clock() : start_(clock_type::now()), last_(start_) {}

        // Возвращает время с момента предыдущего вызова tick() в секундах.
        Seconds tick();

        // Секунды, прошедшие с момента создания часов.
        [[nodiscard]] Seconds elapsed() const;

        void reset();

    private:
        time_point start_;
        time_point last_;
    };

    // Текущее значение монотонных часов в секундах.
    [[nodiscard]] Seconds now();
}
