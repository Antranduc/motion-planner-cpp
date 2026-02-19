cmake_minimum_required(VERSION 3.16)
project(motion-planner-cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

# Prevent dependency tests from building
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
Set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
    GIT_SHALLOW TRUE
)

FetchContent_Declare(
    json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
    GIT_SHALLOW TRUE
)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
    GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(eigen json googletest)

# Re-enable testing
set(BUILD_TESTING ON CACHE BOOL "" FORCE)

# Library Target
add_library(motion_planner
    src/geometry.cpp
)
target_include_directories(motion_planner PUBLIC include)
target_link_libraries(motion_planner PUBLIC Eigen3::Eigen nlohmann_json::nlohmann_json)

# Main Executable
add_executable(planner src/main.cpp)
target_link_libraries(planner PRIVATE motion_planner)

# Tests
enable_testing()
add_executable(tests tests/test_geometry.cpp)
target_link_libraries(tests PRIVATE motion_planner GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(tests)
