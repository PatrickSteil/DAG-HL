cmake_minimum_required(VERSION 3.16)
project(DAGHL)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "-pipe -march=native -Wfatal-errors -pthread -fopenmp")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")

set(CMAKE_CXX_FLAGS_RELEASE "-ffast-math -DNDEBUG -O3 -g")
set(CMAKE_CXX_FLAGS_DEBUG "-rdynamic -Werror -Wpedantic -pedantic-errors -Wall -Wextra -g -fno-omit-frame-pointer -O0")
set(CMAKE_CXX_FLAGS_SANITIZE "-fsanitize=address,undefined -fno-omit-frame-pointer -g -O1")
set(CMAKE_LINKER_FLAGS_SANITIZE "-fsanitize=address,undefined")

if(CMAKE_BUILD_TYPE STREQUAL "Sanitize")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_SANITIZE}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS_SANITIZE}")
    message(STATUS "Sanitizer build enabled")
endif()

# External dependencies
add_subdirectory(external/ips4o EXCLUDE_FROM_ALL)

# Build DAGHL from main.cpp
add_executable(DAGHL main.cpp)
target_include_directories(DAGHL PUBLIC datastructures)
target_link_libraries(DAGHL PRIVATE ips4o)

# Build Benchmark-DAGHL from main.cpp
add_executable(hldag_bench benchmark/hldag_bench.cpp)
target_include_directories(hldag_bench PUBLIC datastructures)
target_link_libraries(hldag_bench PRIVATE ips4o)

# Build PLL from pll.cpp
add_executable(PLL pll.cpp)
target_include_directories(PLL PUBLIC datastructures)
target_link_libraries(PLL PRIVATE ips4o)

# Build Labels vs CompressedLabel
add_executable(LabelVSCompLabel benchmark/label_vs_compressed_label.cpp)
target_include_directories(LabelVSCompLabel PUBLIC datastructures)
target_link_libraries(LabelVSCompLabel PRIVATE ips4o)

# Enable testing
enable_testing()
find_package(GTest)

if(GTest_FOUND)
    add_executable(unit_test 
        tests/bfs_tools_test.cpp 
        tests/drawer_test.cpp 
        tests/compressed_vector_test.cpp
        tests/forest_test.cpp 
        tests/graph_test.cpp 
        tests/priority_queue_test.cpp 
        tests/utils_test.cpp 
    )
    target_include_directories(unit_test PUBLIC datastructures)
    target_link_libraries(unit_test PRIVATE GTest::gtest_main ips4o)
    target_compile_options(unit_test PRIVATE -fopenmp)
    add_test(NAME unit_test COMMAND unit_test)
else()
    message(STATUS "GoogleTest not found, skipping unit tests")
endif()

find_package(benchmark)

if(benchmark_FOUND)
    add_executable(queue_bench benchmark/queue_bench.cpp)
    target_include_directories(queue_bench PUBLIC datastructures benchmark)
    target_link_libraries(queue_bench PRIVATE benchmark::benchmark benchmark::benchmark_main ips4o)

    if(OpenMP_CXX_FOUND)
        target_link_libraries(queue_bench PUBLIC OpenMP::OpenMP_CXX)
    endif()

    add_executable(edgetree_bench benchmark/edgetree_bench.cpp)
    target_include_directories(edgetree_bench PUBLIC datastructures benchmark)
    target_link_libraries(edgetree_bench PRIVATE benchmark::benchmark benchmark::benchmark_main ips4o)
    if(OpenMP_CXX_FOUND)
        target_link_libraries(edgetree_bench PUBLIC OpenMP::OpenMP_CXX)
    endif()
else()
    message(STATUS "Google Benchmark not found, skipping benchmarks")
endif()

