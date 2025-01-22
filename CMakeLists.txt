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

set(SOURCE_FILES
    main.cpp
)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC datastructures)

add_subdirectory(external/ips4o EXCLUDE_FROM_ALL)
target_link_libraries(${PROJECT_NAME} PRIVATE ips4o)

enable_testing()

add_executable(unit_test 
    tests/bfs_tools_test.cpp 
    tests/drawer_test.cpp 
    tests/forest_test.cpp 
    tests/graph_test.cpp 
    tests/priority_queue_test.cpp 
    tests/utils_test.cpp 
)
target_include_directories(unit_test PUBLIC datastructures)

find_package(GTest REQUIRED)
target_link_libraries(unit_test PRIVATE GTest::gtest_main)
target_link_libraries(unit_test PRIVATE ips4o)

target_compile_options(unit_test PRIVATE -fopenmp)
add_test(NAME unit_test COMMAND unit_test)

add_executable(benchmark_queue benchmark/queue_bench.cpp)
target_include_directories(benchmark_queue PUBLIC datastructures benchmark)

find_package(benchmark REQUIRED)
if(benchmark_FOUND)
    target_link_libraries(benchmark_queue PRIVATE benchmark::benchmark benchmark::benchmark_main)
    target_link_libraries(benchmark_queue PRIVATE ips4o)
    if(OpenMP_CXX_FOUND)
        target_link_libraries(benchmark_queue PUBLIC OpenMP::OpenMP_CXX)
    endif()

    # EdgeTree Benchmark
    add_executable(edgetree_bench benchmark/edgetree_bench.cpp)
    target_include_directories(edgetree_bench PUBLIC datastructures benchmark)
    target_link_libraries(edgetree_bench PRIVATE benchmark::benchmark benchmark::benchmark_main)
    target_link_libraries(edgetree_bench PRIVATE ips4o)
    if(OpenMP_CXX_FOUND)
        target_link_libraries(edgetree_bench PUBLIC OpenMP::OpenMP_CXX)
    endif()
else()
    message(FATAL_ERROR "Google Benchmark requested but not found")
endif()

