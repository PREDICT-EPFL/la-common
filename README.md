# la-common

Common C++ libraries shared by the packages of [racing-project-ros2](https://github.com/jowaibel/racing-project-ros2): real-time control of a mini race car, in simulation or on the real car.

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

The libraries are meant to be included in a source tree with `add_subdirectory`, for example as a git submodule, and are not installed as separate packages:

```cmake
add_subdirectory(path/to/la-common/mpc-control-common)
target_link_libraries(my_target PUBLIC mpc_control_common)
```

The top-level `COLCON_IGNORE` file keeps [colcon](https://colcon.readthedocs.io/) from building them as standalone packages when this repository is placed inside a ROS 2 workspace.

## Credits

This project is developed by Johannes Waibel, Roland Schwan, and Colin N. Jones at the [Automatic Control Laboratory](https://www.epfl.ch/labs/la/) at [EPFL](https://www.epfl.ch/), Switzerland.

This work was supported by the Swiss National Science Foundation under the [NCCR Automation](https://nccr-automation.ch/) (grant agreement 51NF40_225155).

## Citing

If you use this code in scientific work, please cite the accompanying paper:

```bibtex
@INPROCEEDINGS{waibel2026laopt,
  author={Waibel, Johannes and Schwan, Roland and Jones, Colin N.},
  booktitle={2026 IEEE Conference on Control Technology and Applications (CCTA)},
  title={{laOPT}: A Native {C++} Optimal Control Toolbox for High-Performance Implementations, and Application to Racing},
  year={2026}
}
```

## License

BSD 2-Clause, see [LICENSE](LICENSE).
