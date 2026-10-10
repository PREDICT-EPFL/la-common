#ifndef SRC_CONTROLMATH_HPP
#define SRC_CONTROLMATH_HPP

#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace control_common {
namespace math {

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

inline Eigen::Matrix2d Tz_2d(const double &angle)
{
    /* Transformation matrix: Rotates around 3rd axis (Z) by angle */
    return (Eigen::Matrix2d()
            << cos(angle), sin(angle),
            -sin(angle), cos(angle)).finished();
}

inline Eigen::Quaterniond T1quat(const double &rotAng) { return {cos(-rotAng / 2.0), sin(-rotAng / 2.0), 0, 0}; }

inline Eigen::Quaterniond T2quat(const double &rotAng) { return {cos(-rotAng / 2.0), 0, sin(-rotAng / 2.0), 0}; }

inline Eigen::Quaterniond T3quat(const double &rotAng) { return {cos(-rotAng / 2.0), 0, 0, sin(-rotAng / 2.0)}; }

/* PID modules ====================================================================================================== */

class Integral
{
public:
    explicit Integral(const double &sampletime_)
    {
        setSampletime(sampletime_);
    }
    Integral(const double &lowerLimit, const double &upperLimit)
    {
        set_upper_saturation_limit(upperLimit);
        set_lower_saturation_limit(lowerLimit);
    }
    Integral(const double &sampletime_, const double &lowerLimit, const double &upperLimit)
    {
        setSampletime(sampletime_);
        set_upper_saturation_limit(upperLimit);
        set_lower_saturation_limit(lowerLimit);
    }
    Integral() = default;

    void setSampletime(const double &sampletime_)
    {
        if (sampletime_ > 0.0) m_dt = sampletime_;
        else std::cout << "[Integral] Sampletime must be > 0. Nothing has been written.\n";
    }
    void set_upper_saturation_limit(const double &upper_limit) { m_integral_upper_limit = upper_limit; }
    void set_lower_saturation_limit(const double &lower_limit) { m_integral_lower_limit = lower_limit; }
    void reset() { set_value(0.0); }

    double get_value() const { return m_integral; }
    double operator()() const { return get_value(); }

    void set_value(double value)
    {
        m_integral = value;
        limitIntegral();
    }

    double integrate(double value)
    {
        /* If dt > 0.5 s, something is wrong. Do not integrate (use dt = 0) */
        const double dt = (m_dt > 0.5) ? 0.0 : m_dt;
        set_value(m_integral + value * dt);
        return get_value();
    }
    double integrate(double value, double dt)
    {
        m_dt = dt;
        return integrate(value);
    }

private:
    double m_integral_upper_limit{std::numeric_limits<double>::max()};
    double m_integral_lower_limit{std::numeric_limits<double>::min()};

    double m_dt{0.01};
    double m_integral{0.0};

    void limitIntegral()
    {
        if (m_integral > m_integral_upper_limit) m_integral = m_integral_upper_limit;
        else if (m_integral < m_integral_lower_limit) m_integral = m_integral_lower_limit;
    }
};

}
}

#endif //SRC_CONTROLMATH_HPP
