#ifndef SRC_QUATERNIONMATHCASADI_HPP
#define SRC_QUATERNIONMATHCASADI_HPP

#include <casadi/casadi.hpp>

namespace model_common {
namespace quatmath_casadi {

using namespace casadi;

/** quaternion arithmetic */
SX T1quat(const SX &rotAng) { return SX::vertcat({cos(-rotAng / 2.0), sin(-rotAng / 2.0), 0, 0}); }
SX T2quat(const SX &rotAng) { return SX::vertcat({cos(-rotAng / 2.0), 0, sin(-rotAng / 2.0), 0}); }
SX T3quat(const SX &rotAng) { return SX::vertcat({cos(-rotAng / 2.0), 0, 0, sin(-rotAng / 2.0)}); }

template<typename sym_t>
sym_t quat_multiply(const sym_t &q1, const sym_t &q2)
{
    sym_t s1 = q1(0);
    sym_t v1 = q1(Slice(1, 4), 0);

    sym_t s2 = q2(0);
    sym_t v2 = q2(Slice(1, 4), 0);

    sym_t s = (s1 * s2) - sym_t::dot(v1, v2);
    sym_t v = sym_t::cross(v1, v2) + (s1 * v2) + (s2 * v1);

    return sym_t::vertcat({s, v});
}
template<typename sym_t>
sym_t quat_inverse(const sym_t &q)
{
    std::vector<sym_t> tmp{q(0), -q(1), -q(2), -q(3)};
    return sym_t::vertcat(tmp);
}
template<typename sym_t>
sym_t quat_transform(const sym_t &q_ba, const sym_t &a_vect)
{
    sym_t tmp = quat_multiply(q_ba, quat_multiply(sym_t::vertcat({0, a_vect}), quat_inverse(q_ba)));
    return tmp(Slice(1, 4), 0);
}

template<typename sym_t>
sym_t euler2quat(const sym_t &euler_rpy)
{
    return quat_multiply(T1quat(euler_rpy(0)), quat_multiply(T2quat(euler_rpy(1)), T3quat(euler_rpy(2))));
}
template<typename sym_t>
sym_t quat2euler(const sym_t &q_nb)
{
    sym_t q1 = q_nb(1);
    sym_t q2 = q_nb(2);
    sym_t q3 = q_nb(3);
    sym_t q4 = q_nb(0);

    /* Relevant components of the Direction Cosine Matrix */
    sym_t T_bg_23 = 2.0 * (q2 * q3 + q4 * q1);
    sym_t T_bg_33 = 2.0 * (q4 * q4 + q3 * q3) - 1.0;
    sym_t T_bg_13 = 2.0 * (q1 * q3 - q4 * q2);
    sym_t T_bg_12 = 2.0 * (q1 * q2 + q4 * q3);
    sym_t T_bg_11 = 2.0 * (q4 * q4 + q1 * q1) - 1.0;

    /* Euler angles: Roll, Pitch, Yaw */
    sym_t euler_angles = sym_t::vertcat({atan2(T_bg_23, T_bg_33),
                                         -asin(T_bg_13),
                                         atan2(T_bg_12, T_bg_11)});
    return euler_angles;
}

} //namespace quatmath_casadi
} //namespace model_common

#endif //SRC_QUATERNIONMATHCASADI_HPP
