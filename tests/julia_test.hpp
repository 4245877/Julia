#pragma once

// Минимальный харнесс для unit-тестов без внешних зависимостей.
// Тест — это обычный исполняемый файл, возвращающий число проваленных
// проверок (0 == успех), что напрямую понятно CTest.

#include <cstdio>

namespace julia_test
{
    inline int& failureCount()
    {
        static int count = 0;
        return count;
    }
}

#define JULIA_CHECK(condition)                                              \
    do                                                                      \
    {                                                                       \
        if (!(condition))                                                   \
        {                                                                   \
            std::printf("FAIL: %s (%s:%d)\n", #condition, __FILE__, __LINE__); \
            ++julia_test::failureCount();                                   \
        }                                                                   \
    } while (false)

#define JULIA_TEST_RESULT() (julia_test::failureCount())
