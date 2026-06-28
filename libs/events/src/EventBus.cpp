#include "julia/events/EventBus.hpp"

namespace julia::events
{
    void EventBus::subscribeErased(EventTypeId id, ErasedHandler handler)
    {
        handlers_[id].push_back(std::move(handler));
    }

    void EventBus::publishErased(EventTypeId id, const void* event) const
    {
        const auto found = handlers_.find(id);
        if (found == handlers_.end())
        {
            return;
        }

        for (const auto& handler : found->second)
        {
            handler(event);
        }
    }
}
