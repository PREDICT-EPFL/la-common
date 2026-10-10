#ifndef SRC_USOLVERBASE_HPP
#define SRC_USOLVERBASE_HPP

#include "casadi/casadi.hpp"

namespace mpc_control_common2 {
namespace casadi_solver {

class USolverBase
{
public:
    using DM = casadi::DM;
    virtual ~USolverBase() { /* Need empty definition. 'default' does not work. */ }

    virtual bool solve()
    {
        std::cout << "solve_fn base \n";
        return false;
    }

    virtual void set_solver_opts(casadi::Dict opts, bool use_jit = false, bool expand = true)
    {
        std::cout << "set_solver_options base \n";
    }
    virtual void set_value(casadi::SX &expr, const casadi::DM &val)
    {
        std::cout << "set_value(SX) base \n";
    }
    virtual void set_value(casadi::MX &expr, const casadi::DM &val)
    {
        std::cout << "set_value(MX) base \n";
    }

    virtual void set_value()
    {
        std::cout << "set_value base \n";
    }

    void set_X_guess(const casadi::DM &X_guess) {}
    void set_U_guess(const casadi::DM &U_guess) {}

    DM get_u0() const { return {}; }
    DM get_T_opt() const { return {}; }
    DM get_U_opt() const { return {}; }
    DM get_X_opt() const { return {}; }
    DM get_p_opt() const { return {}; }
    double get_cost() const { return {}; }
};

}
}

#endif //SRC_USOLVERBASE_HPP
