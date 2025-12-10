cmake_minimum_required(VERSION 3.20)
project(computatioal_geometry_programming)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_library(utils_lib
    src/cpp/utils.cpp
    src/cpp/chull2.cpp
)
target_include_directories(utils_lib
    PUBLIC
    src/hdr
)
target_link_libraries(utils_lib
    PUBLIC
    ${OpenCV_LIBS}
)

add_executable(${PROJECT_NAME}
    src/cpp/main.cpp
)
target_link_libraries(${PROJECT_NAME}
    PRIVATE
    utils_lib
)

include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(utils_tests
    tests/utils_test.cpp
    tests/chull2_test.cpp
)
target_link_libraries(utils_tests
    PRIVATE
    utils_lib
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(utils_tests)

