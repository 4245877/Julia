add_executable(julia_body_sim main.cpp)
target_compile_definitions(julia_body_sim PRIVATE JULIA_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets")
target_link_libraries(julia_body_sim PRIVATE julia_model_import julia_embodiment julia::warnings)
