# GoogleTest (prefer vendored third_party/googletest)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(FUSION_GTEST_DIR "${CMAKE_SOURCE_DIR}/third_party/googletest")
if(EXISTS "${FUSION_GTEST_DIR}/CMakeLists.txt")
    add_subdirectory("${FUSION_GTEST_DIR}" "${CMAKE_BINARY_DIR}/_deps/googletest-build" EXCLUDE_FROM_ALL)
else()
    include(FetchContent)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_MakeAvailable(googletest)
endif()

# Test executable
add_executable(fusion_tests
    test_kalman_filter.cpp
    test_data_association.cpp
    test_scenario_generator.cpp
    test_fusion_integration.cpp
    test_determinism.cpp
)
target_link_libraries(fusion_tests
    fusion_core
    gtest
    gtest_main
)

add_test(
    NAME fusion_tests
    COMMAND fusion_tests
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

# Smoke test retained
add_executable(fusion_smoke_test smoke_test.cpp)
target_link_libraries(fusion_smoke_test PRIVATE fusion_core)
add_test(NAME fusion_smoke_test COMMAND fusion_smoke_test)
