#ifndef SRC_CONTROLMATH_HPP
#define SRC_CONTROLMATH_HPP

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace control_math {

inline double getShortestangleErr(double angleRef, double angle)
{
    double angleErr = std::fmod(angleRef - angle, 2 * M_PI); // Maps to -2*M_PI < ... < 2*M_PI

    if (angleErr > M_PI) { angleErr -= 2 * M_PI; } // Maps to -M_PI < ... <= M_PI
    if (angleErr <= -M_PI) { angleErr += 2 * M_PI; } // Maps to -M_PI < ... <= M_PI
    return angleErr;
}

inline double constrain(double valueToConstrain,
                        double lowerLimit, double upperLimit)
{
    return std::min(std::max(valueToConstrain, lowerLimit), upperLimit);
}
inline Eigen::Vector3d constrain(const Eigen::Vector3d &vectorToConstrain,
                                 double lowerLimit, double upperLimit)
{
    return {constrain(vectorToConstrain(0), lowerLimit, upperLimit),
            constrain(vectorToConstrain(1), lowerLimit, upperLimit),
            constrain(vectorToConstrain(2), lowerLimit, upperLimit)};
}
inline Eigen::Vector3d constrain(const Eigen::Vector3d &vectorToConstrain,
                                 const Eigen::Vector3d &lowerLimit, const Eigen::Vector3d &upperLimit)
{
    return {constrain(vectorToConstrain(0), lowerLimit(0), upperLimit(0)),
            constrain(vectorToConstrain(1), lowerLimit(1), upperLimit(1)),
            constrain(vectorToConstrain(2), lowerLimit(2), upperLimit(2))};
}

template<typename T>
inline T constrain(T valueToConstrain, T symmetricLimit)
{
    return constrain(valueToConstrain, -symmetricLimit, symmetricLimit);
}

inline double degrees(const double &radians)
{
    /* Transforms input value (in rad) to degrees */
    return radians * 180.0 / M_PI;
}
inline Eigen::VectorXd degrees(const Eigen::VectorXd &radians)
{
    /* Transforms input value (in rad) to degrees */
    return radians * 180.0 / M_PI;
}

inline double radians(const double &degrees)
{
    /* Transforms input value (in deg) to radians */
    return degrees * M_PI / 180.0;
}
inline Eigen::VectorXd radians(const Eigen::VectorXd &degrees)
{
    /* Transforms input value (in deg) to radians */
    return degrees * M_PI / 180.0;
}

template<class T>
inline int sign(T val)
{
    return (T(0) < val) - (val < T(0));
}

inline bool closeEnough(const double &f1, const double &f2, const double &tolerance = 0.001)
{
    return (std::abs(f1 - f2) < tolerance);
}

/* Elementary transformations ======================================================================================= */
inline Eigen::Matrix3d T1(const double &angle)
{
    /* Transformation matrix: Rotates around 1st axis (X) by angle */

    return (Eigen::Matrix3d()
            << 1, 0, 0,
            0, cos(angle), sin(angle),
            0, -sin(angle), cos(angle)).finished();
}

inline Eigen::Matrix3d T2(const double &angle)
{
    /* Transformation matrix: Rotates around 2nd axis (Y) by angle */

    return (Eigen::Matrix3d()
            << cos(angle), 0, -sin(angle),
            0, 1, 0,
            sin(angle), 0, cos(angle)).finished();
}

inline Eigen::Matrix3d T3(const double &angle)
{
    /* Transformation matrix: Rotates around 3rd axis (Z) by angle */
    return (Eigen::Matrix3d()
            << cos(angle), sin(angle), 0,
            -sin(angle), cos(angle), 0,
            0, 0, 1).finished();
}

inline Eigen::Quaterniond T1quat(const double &rotAng) { return {cos(-rotAng / 2.0), sin(-rotAng / 2.0), 0, 0}; }

inline Eigen::Quaterniond T2quat(const double &rotAng) { return {cos(-rotAng / 2.0), 0, sin(-rotAng / 2.0), 0}; }

inline Eigen::Quaterniond T3quat(const double &rotAng) { return {cos(-rotAng / 2.0), 0, 0, sin(-rotAng / 2.0)}; }

} // namespace control

#endif //SRC_CONTROLMATH_HPP
