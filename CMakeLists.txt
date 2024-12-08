cmake_minimum_required(VERSION 3.16)
project(HCL)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "-pipe -march=native -Wfatal-errors -pthread")
set(CMAKE_CXX_FLAGS_DEBUG "-rdynamic -Werror -Wpedantic -pedantic-errors -Wall -Wextra -Wparentheses -D_GLIBCXX_DEBUG -g -fno-omit-frame-pointer -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-ffast-math -DNDEBUG -O3")

# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g")

set(SOURCE_FILES
    main.cpp
)

set(HEADER_FILES
    datastructures/bfs_tools.h
    datastructures/pruned_landmark_labeling.h
)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC datastructures)

find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)
endif()

enable_testing()

add_executable(${PROJECT_NAME}_test tests/pruned_landmark_labeling_test.cpp tests/graph_test.cpp)
target_include_directories(${PROJECT_NAME}_test PUBLIC datastructures)

find_package(GTest REQUIRED)
target_link_libraries(${PROJECT_NAME}_test PRIVATE GTest::gtest_main GTest::gmock_main)

if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME}_test PUBLIC OpenMP::OpenMP_CXX)
endif()

add_test(NAME ${PROJECT_NAME}_test COMMAND ${PROJECT_NAME}_test)

