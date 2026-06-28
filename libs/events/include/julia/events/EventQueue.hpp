#pragma once

#include "julia/events/EventBus.hpp"

#include <cstddef>
#include <functional>
#include <utility>
#include <vector>

namespace julia::events
{
    // Очередь отложенных событий. События накапливаются через push() и
    // доставляются на шину разом при dispatch(), что развязывает источник
    // события и момент его обработки в кадре симуляции.
    class EventQueue
    {
    public:
        template <typename E>
        void push(E event)
        {
            pending_.push_back(
                [stored = std::move(event)](EventBus& bus)
                {
                    bus.publish(stored);
                }
            );
        }

        // Публикует все накопленные события на шине и очищает очередь.
        // Безопасно, если обработчики ставят новые события в очередь.
        void dispatch(EventBus& bus);

        [[nodiscard]] std::size_t size() const noexcept { return pending_.size(); }
        [[nodiscard]] bool empty() const noexcept { return pending_.empty(); }

        void clear() noexcept { pending_.clear(); }

    private:
        std::vector<std::function<void(EventBus&)>> pending_;
    };
}
