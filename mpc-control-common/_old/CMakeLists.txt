cmake_minimum_required(VERSION 3.5)
project(mpc_control_common)

if (NOT TARGET mpc_control_common)

    set(CMAKE_CXX_STANDARD 11)
    set(CMAKE_BUILD_TYPE Release)

    if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
    endif ()

    if (NOT WITH_CASADI)
        set(WITH_CASADI ON)
    endif ()
    message("mpc_control_common WITH_CASADI: ${WITH_CASADI}")

    find_package(Eigen3 REQUIRED NO_MODULE)
    find_package(casadi REQUIRED)

    add_library(mpc_control_common INTERFACE)
    target_include_directories(mpc_control_common INTERFACE include)
    target_link_libraries(mpc_control_common INTERFACE Eigen3::Eigen casadi::casadi)

    if (${WITH_CASADI})
        add_library(mpc_control_common_casadi INTERFACE)
        target_include_directories(mpc_control_common_casadi INTERFACE include_casadi)
        target_link_libraries(mpc_control_common_casadi INTERFACE mpc_control_common Eigen3::Eigen casadi::casadi)
    endif ()
endif ()
