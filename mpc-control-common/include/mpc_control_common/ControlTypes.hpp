#ifndef SRC_CONTROLTYPES_HPP
#define SRC_CONTROLTYPES_HPP

#include <chrono>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace control_types {

/* Eigen types */
using Vector2 = Eigen::Matrix<double, 2, 1, Eigen::DontAlign>;
using Vector3 = Eigen::Matrix<double, 3, 1, Eigen::DontAlign>;
using Matrix2 = Eigen::Matrix<double, 2, 2, Eigen::DontAlign>;
using Matrix3 = Eigen::Matrix<double, 3, 3, Eigen::DontAlign>;
using Quaternion = Eigen::Quaternion<double, Eigen::DontAlign>;

/* Chrono types */
using SteadyClock = std::chrono::steady_clock;
using TimePoint = std::chrono::steady_clock::time_point;

} // namespace control

#endif //SRC_CONTROLTYPES_HPP
