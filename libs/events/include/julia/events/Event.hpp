#pragma once

namespace julia::events
{
    // Необязательная база-маркер для событий. Шина событий работает с любым
    // типом, но наследование от Event делает намерение явным и позволяет
    // хранить события полиморфно там, где это нужно.
    struct Event
    {
        Event() = default;
        virtual ~Event() = default;
    };
}
