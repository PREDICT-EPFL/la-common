cmake_minimum_required(VERSION 3.5)
project(main_control_common)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(Eigen3 REQUIRED NO_MODULE)

if (NOT TARGET main_control_common)
    add_library(main_control_common src/KalmanBiasEstimator.cpp)
    target_include_directories(main_control_common PUBLIC include/)
    target_link_libraries(main_control_common PUBLIC Eigen3::Eigen)
endif ()
