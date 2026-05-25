cmake_minimum_required(VERSION 3.12)
project(benchmark_rt_cpp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# RT-aware flags: full optimisation, expose GNU/POSIX extensions (clock_nanosleep,
# pthread_setschedparam, etc.), keep frame pointers for perf/ftrace profiling.
add_compile_options(
    -O3
    -march=native
    -fno-omit-frame-pointer
    -D_GNU_SOURCE
)

find_package(unitree_sdk2 REQUIRED)

set(BENCHMARKS
    latency_v3_fix
    latency_intrp_v4
    encoder_jitter
    packet_loss_test
    pulse_sweep
    imu_logger
    go2_eskf_logger
)

foreach(target ${BENCHMARKS})
    add_executable(${target} src/${target}.cpp)
    target_link_libraries(${target} unitree_sdk2 rt pthread)
    target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
endforeach()
