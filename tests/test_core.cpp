#include "julia/core/Id.hpp"
#include "julia/core/Result.hpp"
#include "julia/core/Time.hpp"

#include "julia_test.hpp"

namespace
{
    struct EntityTag;
    using EntityId = julia::core::Id<EntityTag>;
}

int main()
{
    using namespace julia::core;

    // Id
    const EntityId invalid{};
    const EntityId first{5};
    const EntityId alsoFirst{5};

    JULIA_CHECK(!invalid.valid());
    JULIA_CHECK(first.valid());
    JULIA_CHECK(first == alsoFirst);
    JULIA_CHECK(first != invalid);
    JULIA_CHECK(first.value() == 5);

    // Result
    const Result<int> good = Result<int>::success(42);
    JULIA_CHECK(good.hasValue());
    JULIA_CHECK(static_cast<bool>(good));
    JULIA_CHECK(good.value() == 42);

    const Result<int> bad = Result<int>::failure("oops");
    JULIA_CHECK(!bad.hasValue());
    JULIA_CHECK(!bad);
    JULIA_CHECK(bad.error() == "oops");
    JULIA_CHECK(bad.valueOr(7) == 7);

    // Clock / now
    Clock clock;
    JULIA_CHECK(clock.elapsed() >= 0.0);
    JULIA_CHECK(clock.tick() >= 0.0);
    JULIA_CHECK(now() > 0.0);

    return JULIA_TEST_RESULT();
}
