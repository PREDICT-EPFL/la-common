# la-common

Common C++ libraries shared applied control projects: e.g., real-time control of a mini race car, in simulation or on the real car; flight simulation and control of an airborne wind energy system.

## Libraries

| Library | CMake target | Description |
|---|---|---|
| `main-control-common` | `main_control_common` | Base classes and utilities for control loops: control loop skeleton (`ControlBase`), math, types and helper utilities, and a Kalman bias estimator. |
| `model-common` | `model_common` | Base classes for system dynamics and parameters, with Eigen and CasADi variants, and quaternion math. |
| `simulation-common` | `simulation_common` | Simulator base classes, simulation engines (Eigen and CasADi), ODE integration, and sensor models. |
| `mpc-control-common` | `mpc_control_common` | Generic building blocks for MPC: optimizer interface, CasADi Opti-based optimal control problem base class, and multiple shooting and Radau collocation transcriptions. |

All libraries are header-only except `main_control_common`.

## Requirements

- C++11 compiler and CMake
- [Eigen](https://eigen.tuxfamily.org/) 3
- [yaml-cpp](https://github.com/jbeder/yaml-cpp) (`model-common`)
- [CasADi](https://web.casadi.org/) (optional for `model-common` and `simulation-common`, set `-DWITH_CASADI=ON`; used by `mpc-control-common` by default)

## Usage

The libraries are not installed as separate packages. A package that needs one includes it in its own source tree with `add_subdirectory`, so it is always built from the sources you are editing.

Add this repository as a single git submodule at the root of your project:

```shell
git submodule add https://github.com/jowaibel/la-common.git la-common
```

Then create a symlink in the `lib/` folder of each package that needs a library, pointing to the library folder in `la-common`. For example, for a control package, create the symlink `my-control-package/lib/main-control-common` pointing to `la-common/main-control-common` (`ln -s`).

The package then uses the library like any subfolder:

```cmake
# my-control-package/CMakeLists.txt
add_subdirectory(lib/main-control-common)
target_link_libraries(my_control PUBLIC main_control_common)
```

Git stores symlinks as such, which works on Linux and macOS; on Windows, git needs symlink support enabled.

The top-level `COLCON_IGNORE` file keeps [colcon](https://colcon.readthedocs.io/) from building the libraries as standalone packages when this repository is placed inside a ROS 2 workspace.

## Credits

This project is developed by Johannes Waibel at the [Automatic Control Laboratory](https://www.epfl.ch/labs/la/) at [EPFL](https://www.epfl.ch/), Switzerland.

This work was supported by the Swiss National Science Foundation under the [NCCR Automation](https://nccr-automation.ch/) (grant agreement 51NF40_225155).

## License

BSD 2-Clause, see [LICENSE](LICENSE).
