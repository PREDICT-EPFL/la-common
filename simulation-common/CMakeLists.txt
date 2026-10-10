cmake_minimum_required(VERSION 3.5)
project(simulation_common)

if (NOT TARGET simulation_common)

    set(CMAKE_CXX_STANDARD 11)
    set(CMAKE_BUILD_TYPE Release)

    if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
    endif ()

    if (NOT WITH_CASADI)
        set(WITH_CASADI OFF)
    endif ()
    message("simulation_common WITH_CASADI: ${WITH_CASADI}")

    find_package(Eigen3 REQUIRED NO_MODULE) # This ensures Eigen3 is installed on the system. The target to link to is 'Eigen3::Eigen' afterwards.

    add_library(simulation_common INTERFACE)
    target_include_directories(simulation_common INTERFACE
                               $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                               $<INSTALL_INTERFACE:include>)
    target_link_libraries(simulation_common INTERFACE Eigen3::Eigen)

    if (${WITH_CASADI})
        find_package(Casadi REQUIRED) # This ensures Casadi is installed on the system. The target to link to is 'casadi' afterwards.
        target_include_directories(simulation_common INTERFACE include_casadi/)
        target_include_directories(simulation_common INTERFACE
                                   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include_casadi>
                                   $<INSTALL_INTERFACE:include>)
        target_link_libraries(simulation_common INTERFACE casadi)
    endif ()

endif ()
