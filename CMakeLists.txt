cmake_minimum_required(VERSION 3.16)
project(HCL)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set compiler flags for different build types
set(CMAKE_CXX_FLAGS "-pipe -march=native -Wfatal-errors -pthread")
set(CMAKE_CXX_FLAGS_RELEASE "-ffast-math -DNDEBUG -O3")
set(CMAKE_CXX_FLAGS_DEBUG "-rdynamic -Werror -Wpedantic -pedantic-errors -Wall -Wextra -D_GLIBCXX_DEBUG -g -fno-omit-frame-pointer -O0")
set(CMAKE_CXX_FLAGS_SANITIZE "-fsanitize=address,undefined -fno-omit-frame-pointer -g -O1")
set(CMAKE_LINKER_FLAGS_SANITIZE "-fsanitize=address,undefined")

# Detect sanitizer build
if(CMAKE_BUILD_TYPE STREQUAL "Sanitize")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_SANITIZE}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS_SANITIZE}")
    message(STATUS "Sanitizer build enabled")
endif()

# Source and header files
set(SOURCE_FILES
    main.cpp
)

set(HEADER_FILES
    datastructures/bfs_tools.h
    datastructures/pruned_landmark_labeling.h
)

# Main executable
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC datastructures)

find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    target_compile_options(${PROJECT_NAME} PRIVATE ${OpenMP_CXX_FLAGS})
    target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_OMP")
else()
    message(FATAL_ERROR "OpenMP requested but not found")
endif()

# Enable testing
enable_testing()

# Test executable
add_executable(${PROJECT_NAME}_test 
    tests/pruned_landmark_labeling_test.cpp 
    tests/graph_test.cpp 
    tests/priority_queue_test.cpp 
)
target_include_directories(${PROJECT_NAME}_test PUBLIC datastructures)

find_package(GTest REQUIRED)
target_link_libraries(${PROJECT_NAME}_test PRIVATE GTest::gtest_main GTest::gmock_main)

if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME}_test PUBLIC OpenMP::OpenMP_CXX)
endif()

add_test(NAME ${PROJECT_NAME}_test COMMAND ${PROJECT_NAME}_test)

