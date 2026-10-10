#ifndef SRC_USOLVEROPTI_HPP
#define SRC_USOLVEROPTI_HPP

#include <memory>

#include "casadi/casadi.hpp"
#include "USolverBase.hpp"

namespace mpc_control_common2 {
namespace casadi_solver {

class USolverOpti : public USolverBase
{
public:
    explicit USolverOpti(casadi::Opti &opti_) : opti(opti_) {}

    casadi::Opti opti;
    std::shared_ptr<casadi::OptiSol> sol;

    casadi::DM T_normalized;
    casadi::MX X_sym;
    casadi::MX U_sym;
    casadi::MX tf_sym;
    std::map<std::string, casadi::MX> opt_params_sym;

    void set_solver_opts(casadi::Dict opts, bool use_jit = false, bool expand = true) override
    {
        /* Setup solver */
        if (opts.empty())
        {
            casadi::Dict jit_options = {{"flags",   "-O3"},
                                        {"verbose", true}};

            opts = casadi::Dict({{"ipopt.sb",          "no"},
                                 {"ipopt.print_level", 0},    // 5: print iterations and scaling factor
                                 {"print_time",        false},
                                 {"ipopt.max_iter",    100},
                                 {"ipopt.tol",         1e-3},

                                 {"jit",               use_jit},
                                 {"compiler",          "shell"},
                                 {"jit_options",       jit_options},
                                 {"expand",            expand}
                                });
        }
        std::cout << "Setting IPOPT solver options: " << opts << "\n";
        opti.solver("ipopt", opts);
    }

    void set_value(casadi::MX &sym_expr, const casadi::DM &val) override
    {
        opti.set_value(sym_expr, val);
//        std::cout << "sym " << sym_expr << ": " << opti.value(sym_expr) << "\n";
    }

    /* Setters for initial guesses */
    void set_X_guess(const casadi::DM &X_guess) { opti.set_initial(X_sym, X_guess); }
    void set_U_guess(const casadi::DM &U_guess) { opti.set_initial(U_sym, U_guess); }

    bool solve() override
    {
        bool solve_success{false};

        try
        {
//            std::cout << "opti status: " << opti << "\n";
            casadi::OptiSol sol_ = opti.solve();

            std::cout << "IPOPT: " << sol_.stats()["return_status"] << "\n";
            solve_success = sol_.stats()["success"];
            //casadi_assert(solve_success, "Error computing optimal input");

//            if (warmstart)
//            {
//                // Use the current solution to speed up the next optimization
//                opti.set_initial(sol_.value_variables());
//                opti.set_initial(opti.lag(), sol_.value(opti.lag()));
//            }
            sol = std::make_shared<casadi::OptiSol>(sol_);
        }
        catch (const std::exception &e)
        {
            std::cout << e.what() << "\n";
            std::cout << "X_debug: " << opti.debug().value(X_sym) << "\n";
            std::cout << "U_debug: " << opti.debug().value(U_sym) << "\n";
            //std::terminate();
        }
        return solve_success;
    }

    /* Call for OCP solve */
    DM get_u0() const { return sol->value(U_sym(casadi::Slice(), 0)); }
    DM get_T_opt() const { return sol->value(tf_sym) * T_normalized; }
    DM get_U_opt() const { return sol->value(U_sym); }
    DM get_X_opt() const { return sol->value(X_sym); }
    DM get_p_opt() const { return {}; }
    double get_cost() const { return sol->value(opti.f()).nonzeros()[0]; }
};

}
}

#endif //SRC_USOLVEROPTI_HPP
