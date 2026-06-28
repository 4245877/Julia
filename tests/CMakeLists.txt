# Unit-тесты. Регистрируются в CTest через add_test.
# Каждый тест — отдельный исполняемый файл, возвращающий число провалов.

add_executable(julia_test_core test_core.cpp)
target_include_directories(julia_test_core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(julia_test_core PRIVATE julia_core julia::warnings)
add_test(NAME core COMMAND julia_test_core)

add_executable(julia_test_events test_events.cpp)
target_include_directories(julia_test_events PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(julia_test_events PRIVATE julia_events julia::warnings)
add_test(NAME events COMMAND julia_test_events)
