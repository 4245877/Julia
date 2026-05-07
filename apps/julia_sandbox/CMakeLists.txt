add_executable(julia_sandbox
    main.cpp
)

target_compile_features(julia_sandbox PRIVATE cxx_std_20)