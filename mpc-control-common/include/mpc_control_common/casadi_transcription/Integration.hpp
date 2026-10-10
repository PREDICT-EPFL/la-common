#ifndef SRC_INTEGRATION_HPP
#define SRC_INTEGRATION_HPP

#include "casadi/casadi.hpp"

namespace mpc_control_common {
namespace transcription_casadi {
namespace integration {

template<typename sym_t = casadi::SX>
sym_t RK11_int(const casadi::Function &f_cont,
               const sym_t &x, const sym_t &u, const sym_t &p, const sym_t &d, const sym_t &h)
{
    using symvec_t = std::vector<sym_t>;

    /* Euler's method */
    sym_t k1 = f_cont(symvec_t({x, u, p, d}))[0];
    sym_t x_next = x + h * k1;
    return x_next;
}
template<typename sym_t = casadi::SX>
sym_t RK2a_int(const casadi::Function &f_cont,
               const sym_t &x, const sym_t &u, const sym_t &p, const sym_t &d, const sym_t &h)
{
    using symvec_t = std::vector<sym_t>;

    /* Improved Euler's method (Midpoint) */
    sym_t k1 = f_cont(symvec_t({x, u, p, d}))[0];
    sym_t k2 = f_cont(symvec_t({x + h / 2.0 * k1, u, p, d}))[0];
    sym_t x_next = x + h * k2;
    return x_next;
}
template<typename sym_t = casadi::SX>
sym_t RK2b_int(const casadi::Function &f_cont,
               const sym_t &x, const sym_t &u, const sym_t &p, const sym_t &d, const sym_t &h)
{
    using symvec_t = std::vector<sym_t>;

    /* Heun's method (Trapezoidal) */
    sym_t k1 = f_cont(symvec_t({x, u, p, d}))[0];
    sym_t k2 = f_cont(symvec_t({x + h * k1, u, p, d}))[0];
    sym_t x_next = x + h / 2.0 * (k1 + k2);
    return x_next;
}
template<typename sym_t = casadi::SX>
sym_t RK4_int(const casadi::Function &f_cont,
              const sym_t &x, const sym_t &u, const sym_t &p, const sym_t &d, const sym_t &h)
{
    using symvec_t = std::vector<sym_t>;

    sym_t k1 = f_cont(symvec_t({x, u, p, d}))[0];
    sym_t k2 = f_cont(symvec_t({x + h / 2.0 * k1, u, p, d}))[0];
    sym_t k3 = f_cont(symvec_t({x + h / 2.0 * k2, u, p, d}))[0];
    sym_t k4 = f_cont(symvec_t({x + h * k3, u, p, d}))[0];
    sym_t x_next = x + h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    return x_next;
}
template<typename sym_t = casadi::SX>
sym_t IRK11_int(const casadi::Function &f_cont,
                const sym_t &x_next, const sym_t &x, const sym_t &u, const sym_t &p, const sym_t &d, const sym_t &h)
{
    using symvec_t = std::vector<sym_t>;
    // Implicit Euler
    return x_next -
           (x + h * f_cont(symvec_t({x, u, p, d}))[0]);
}
template<typename sym_t = casadi::SX>
sym_t IRK12_int(const casadi::Function &f_cont,
                const sym_t &x_next, const sym_t &x, const sym_t &u, const sym_t &p, const sym_t &d, const sym_t &h)
{
    using symvec_t = std::vector<sym_t>;
    // Implicit Midpoint
    return x_next -
           (x + h * f_cont(symvec_t({(x + x_next) / 2.0, u, p, d}))[0]);
}
template<typename sym_t = casadi::SX>
sym_t IRK22_int(const casadi::Function &f_cont,
                const sym_t &x_next, const sym_t &x, const sym_t &u, const sym_t &p, const sym_t &d, const sym_t &h)
{
    using symvec_t = std::vector<sym_t>;
    // Implicit Trapezoidal
    return x_next -
           (x + h / 2.0 * (f_cont(symvec_t({x, u, p, d}))[0] + f_cont(symvec_t({x_next, u, p, d}))[0]));
}

} // namespace integration
} // namespace casadi_transcription
} // namespace mpc_control_common

#endif //SRC_INTEGRATION_HPP
