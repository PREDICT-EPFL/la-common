cmake_minimum_required(VERSION 3.5)
project(model_common)

if (NOT TARGET model_common)

    set(CMAKE_CXX_STANDARD 11)
    set(CMAKE_BUILD_TYPE Release)

    if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
    endif ()

    if (NOT WITH_CASADI)
        set(WITH_CASADI OFF)
    endif ()
    message("model_common WITH_CASADI: ${WITH_CASADI}")

    find_package(Eigen3 REQUIRED NO_MODULE)
    find_package(yaml-cpp 0.5.2 REQUIRED)

    add_library(model_common INTERFACE)
    target_include_directories(model_common INTERFACE
                               $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                               $<INSTALL_INTERFACE:include>)
    target_link_libraries(model_common INTERFACE Eigen3::Eigen yaml-cpp)

    if (${WITH_CASADI})
        find_package(Casadi REQUIRED)
        target_include_directories(model_common INTERFACE
                                   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include_casadi>
                                   $<INSTALL_INTERFACE:include>)
        target_link_libraries(model_common INTERFACE casadi)
    endif ()

endif ()
