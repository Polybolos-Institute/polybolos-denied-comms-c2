# Benchmark executable
add_executable(fusion_benchmark
    benchmark_fusion.cpp
)
target_link_libraries(fusion_benchmark
    fusion_core
)

add_executable(fusion_benchmark_big
    benchmark_big_scenario.cpp
)
target_link_libraries(fusion_benchmark_big
    fusion_core
)

# Keep naive comparison bench
add_executable(fusion_bench_vs_naive bench_vs_naive.cpp)
target_link_libraries(fusion_bench_vs_naive PRIVATE fusion_core)
