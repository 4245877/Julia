#include "julia/events/EventQueue.hpp"

namespace julia::events
{
    void EventQueue::dispatch(EventBus& bus)
    {
        // Забираем текущую порцию событий, чтобы обработчики могли безопасно
        // ставить новые события в очередь во время доставки (они уйдут в
        // следующий dispatch, а не в этот, исключая бесконечный цикл).
        std::vector<std::function<void(EventBus&)>> batch;
        batch.swap(pending_);

        for (auto& deliver : batch)
        {
            deliver(bus);
        }
    }
}
