#pragma once

#include "julia/events/EventType.hpp"

#include <functional>
#include <unordered_map>
#include <vector>

namespace julia::events
{
    // Синхронная шина событий с подпиской по типу. Обработчики вызываются
    // немедленно при publish(). Для отложенной доставки см. EventQueue.
    //
    // Шина не потокобезопасна: предполагается её использование из одного
    // потока симуляции.
    class EventBus
    {
    public:
        template <typename E>
        void subscribe(std::function<void(const E&)> handler)
        {
            auto erased = [callback = std::move(handler)](const void* event)
            {
                callback(*static_cast<const E*>(event));
            };

            subscribeErased(eventTypeId<E>(), std::move(erased));
        }

        template <typename E>
        void publish(const E& event) const
        {
            publishErased(eventTypeId<E>(), &event);
        }

    private:
        using ErasedHandler = std::function<void(const void*)>;

        void subscribeErased(EventTypeId id, ErasedHandler handler);
        void publishErased(EventTypeId id, const void* event) const;

        std::unordered_map<EventTypeId, std::vector<ErasedHandler>> handlers_;
    };
}
