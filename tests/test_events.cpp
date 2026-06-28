#include "julia/events/EventBus.hpp"
#include "julia/events/EventQueue.hpp"

#include "julia_test.hpp"

namespace
{
    struct PingEvent
    {
        int value = 0;
    };
}

int main()
{
    using namespace julia::events;

    EventBus bus;

    int received = 0;
    bus.subscribe<PingEvent>(
        [&received](const PingEvent& event)
        {
            received += event.value;
        }
    );

    // Синхронная доставка через шину.
    bus.publish(PingEvent{3});
    JULIA_CHECK(received == 3);

    // Событие без подписчиков ничего не делает и не падает.
    struct UnhandledEvent {};
    bus.publish(UnhandledEvent{});

    // Отложенная доставка через очередь.
    EventQueue queue;
    queue.push(PingEvent{10});
    queue.push(PingEvent{5});
    JULIA_CHECK(queue.size() == 2);
    JULIA_CHECK(!queue.empty());

    queue.dispatch(bus);
    JULIA_CHECK(received == 18);
    JULIA_CHECK(queue.empty());

    return JULIA_TEST_RESULT();
}
