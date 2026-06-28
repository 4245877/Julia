#pragma once

#include <typeindex>
#include <typeinfo>

namespace julia::events
{
    // Стабильный идентификатор типа события на основе RTTI.
    // Любой пользовательский тип может быть событием — наследование от общей
    // базы не требуется.
    using EventTypeId = std::type_index;

    template <typename E>
    [[nodiscard]] EventTypeId eventTypeId() noexcept
    {
        return std::type_index(typeid(E));
    }
}
