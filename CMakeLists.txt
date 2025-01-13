cmake_minimum_required(VERSION 3.16)
project(DAGHL)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "-pipe -march=native -Wfatal-errors -pthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")

set(CMAKE_CXX_FLAGS_RELEASE "-ffast-math -DNDEBUG -O3 -g")
set(CMAKE_CXX_FLAGS_DEBUG "-rdynamic -Werror -Wpedantic -pedantic-errors -Wall -Wextra -D_GLIBCXX_DEBUG -g -fno-omit-frame-pointer -O0")
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

find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    target_compile_options(${PROJECT_NAME} PRIVATE ${OpenMP_CXX_FLAGS})
    target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_OMP")
else()
    message(FATAL_ERROR "OpenMP requested but not found")
endif()

enable_testing()

add_executable(unit_test 
    tests/bfs_tools_test.cpp 
    tests/graph_test.cpp 
    tests/utils_test.cpp 
    tests/priority_queue_test.cpp 
)
target_include_directories(unit_test PUBLIC datastructures)

find_package(GTest REQUIRED)
target_link_libraries(unit_test PRIVATE GTest::gtest_main)
target_link_libraries(unit_test PRIVATE ips4o)

if(OpenMP_CXX_FOUND)
    target_link_libraries(unit_test PUBLIC OpenMP::OpenMP_CXX)
endif()

add_test(NAME unit_test COMMAND unit_test)

