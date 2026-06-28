#pragma once

#include <string>
#include <utility>
#include <variant>

namespace julia::core
{
    // Лёгкая обёртка для возврата либо значения, либо ошибки, без исключений.
    // Это упрощённый аналог std::expected (C++23) для C++20.
    template <typename T, typename E = std::string>
    class Result
    {
    public:
        Result(T value) : storage_(std::move(value)) {}

        static Result success(T value) { return Result(std::move(value)); }
        static Result failure(E error) { return Result(ErrorTag{}, std::move(error)); }

        [[nodiscard]] bool hasValue() const noexcept
        {
            return std::holds_alternative<T>(storage_);
        }

        [[nodiscard]] explicit operator bool() const noexcept { return hasValue(); }

        [[nodiscard]] T& value() & { return std::get<T>(storage_); }
        [[nodiscard]] const T& value() const& { return std::get<T>(storage_); }
        [[nodiscard]] T&& value() && { return std::get<T>(std::move(storage_)); }

        [[nodiscard]] const E& error() const& { return std::get<E>(storage_); }

        [[nodiscard]] T valueOr(T fallback) const
        {
            return hasValue() ? std::get<T>(storage_) : std::move(fallback);
        }

    private:
        struct ErrorTag {};

        Result(ErrorTag, E error) : storage_(std::move(error)) {}

        std::variant<T, E> storage_;
    };

    template <typename T, typename E = std::string>
    [[nodiscard]] Result<T, E> ok(T value)
    {
        return Result<T, E>::success(std::move(value));
    }

    template <typename T, typename E = std::string>
    [[nodiscard]] Result<T, E> err(E error)
    {
        return Result<T, E>::failure(std::move(error));
    }
}
