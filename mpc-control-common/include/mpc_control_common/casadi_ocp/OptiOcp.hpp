#ifndef SRC_OPTIOCP_HPP
#define SRC_OPTIOCP_HPP

#include <memory>
#include "casadi/casadi.hpp"

namespace mpc_control_common {
namespace ocp_casadi {

template<typename return_t>
void parse_dict(const casadi::Dict &dict, const std::string &key, return_t &set_to)
{
    if (dict.find(key) != dict.end())
        set_to = static_cast<return_t>(dict.at(key));
    else
        std::cout << "Parsing options: Option '" << key << "' could not be found.\n";
}

class OptiOcp
{
public:
    using sym_t = casadi::MX;
    using DM = casadi::DM;

    OptiOcp()
    {
        // Set default options, for IPOPT
        auto opti_default_opts = casadi::Dict({
                                               {"expand",     false},
                                               {"print_time", false}
                                              });

        std::string default_solver = "ipopt";
        auto solver_default_opts = casadi::Dict({
                                                 {"max_iter",    100},
                                                 {"tol",         1e-6},
                                                 {"sb",          "no"}, // show banner
                                                 {"print_level", 0},    // 5: print iterations and scaling factor
                                                });
        auto jit_default_opts = casadi::Dict({
//                                              {"jit", false},
                                              {"compiler", "shell"},
                                              {"jit_options", casadi::Dict({
                                                              {"flags",   "-O3"},
                                                              {"verbose", true}} )}
                                              });

        set_solver(default_solver, opti_default_opts, solver_default_opts, jit_default_opts);
        set_jit(false);
    }
    void set_solver(const std::string &solver,
                    const casadi::Dict &opti_options = casadi::Dict(),
                    const casadi::Dict &solver_options = casadi::Dict(),
                    const casadi::Dict &jit_opts = casadi::Dict())
    {
        if (solver.empty())
        {
            std::cerr << "No solver name given. Nothing updated!\n";
            return;
        }
        set_solver_opts(solver, solver_options);
        set_opti_opts(opti_options);
        set_jit_opts(jit_opts);

        setup_solver();
    }
    void set_opti_opts(const casadi::Dict &opti_options)
    {
        // Write new options into solver_opts dictionary (overwrite if exists already)
        for (const auto &kv : opti_options) { opti_opts[kv.first] = kv.second; }
    }
    void set_solver_opts(const std::string &solver, const casadi::Dict &solver_options)
    {
        if (solver.empty())
        {
            std::cerr << "No solver name given. Nothing updated!\n";
            return;
        }

        if (solver_name != solver)
        {
            // Remove previous options for different solver
            std::vector<std::string> keys_to_remove;
            for (const auto &kv : solver_opts)
            {
                if (kv.first.find(solver_name) != std::string::npos) { keys_to_remove.push_back(kv.first); }
            }
            for (const auto &key : keys_to_remove) { solver_opts.erase(key); }
        }

        // Write new options into solver_opts dictionary (overwrite if exists already)
        solver_name = solver;
        for (const auto &kv : solver_options)
        {
            solver_opts[solver_name + '.' + kv.first] = kv.second;
        }
    }
    void set_jit_opts(const casadi::Dict &jit_options)
    {
        // Write new options into solver_opts dictionary (overwrite if exists already)
        for (const auto &kv : jit_options) { jit_opts[kv.first] = kv.second; }
    }
    void set_jit(const bool use_jit, const casadi::Dict &jit_options = casadi::Dict())
    {
        jit_opts["jit"] = use_jit;
        set_jit_opts(jit_options);

        setup_solver();
    }
    void setup_solver()
    {
        if (opti)
        {
            casadi::Dict opts;
            opts.insert(opti_opts.begin(), opti_opts.end());
            opts.insert(solver_opts.begin(), solver_opts.end());
            opts.insert(jit_opts.begin(), jit_opts.end());

            std::cout << "Setting " << solver_name << " with options: " << opts << "\n";
            opti->solver(solver_name, opts);
        }
    }

    /* Setters for state and control bounds */
    void set_ubx(DM ubx, DM slack_bounds = DM(), DM slack_weights = DM())
    {
        /* Fill up potentially missing entries with INF */
        ubx = DM::vertcat({DM::vec(ubx), DM::inf(X_sym.size1() - DM::vec(ubx).numel(), 1)});

        /* If not given at all, set zero slack bounds and 'symbolic' weight of 1 */
        if (slack_bounds.is_empty()) { slack_bounds = DM::zeros(ubx.size()); }
        if (slack_weights.is_empty()) { slack_weights = DM::ones(slack_bounds.size()); }
        /* Ensure column vector shape */
        slack_bounds = DM::vec(slack_bounds);
        slack_weights = DM::vec(slack_weights);

        /* Fill up potentially missing entries */
        slack_bounds = DM::vertcat({DM::vec(slack_bounds), DM::zeros(ubx.numel() - slack_bounds.numel(), 1)});
        slack_weights = DM::vertcat({DM::vec(slack_weights), DM::ones(ubx.numel() - slack_weights.numel(), 1)});

        /* Loop through all states */
        for (int ix = 0; ix < nx; ++ix)
        {
            if (ix < slack_bounds.numel() &&
                ubx(ix).nonzeros()[0] < INFINITY && slack_bounds(ix).nonzeros()[0] > 0)
            {
                /* A finite bound AND a nonzero slack_bound is specified */
                sym_t slack = opti->variable();       // Create a new slack variable

                // Set constraint on state using slack and constraint on slack
                opti->subject_to(X_sym(ix, casadi::Slice()) <= ubx(ix) + slack);
                opti->subject_to(0 <= slack <= slack_bounds(ix));

                // Add penalty for slack usage to cost
                sym_t cost = opti->f();
                sym_t slack_penalty = slack_weights(ix) * 0.5 * (slack * slack + slack);
                opti->minimize(cost + slack_penalty);

                std::cout << "State " << std::to_string(ix) << " upper bound: "
                          << std::to_string(ubx(ix).nonzeros()[0])
                          << " (+ " << std::to_string(slack_bounds(ix).nonzeros()[0]) << ")\n";
            }
            else
            {
                /* Set the hard constraint. */
                opti->subject_to(X_sym(ix, casadi::Slice()) <= ubx(ix));
                std::cout << "State " << std::to_string(ix) << " upper bound: "
                          << std::to_string(ubx(ix).nonzeros()[0]) << "\n";
            }
        }
    }
    void set_lbx(DM lbx, DM slack_bounds = DM(), DM slack_weights = DM())
    {
        /* Fill up potentially missing entries with INF */
        lbx = DM::vertcat({DM::vec(lbx), DM::inf(X_sym.size1() - DM::vec(lbx).numel(), 1)});

        /* If not given at all, set zero slack bounds and 'symbolic' weight of 1 */
        if (slack_bounds.is_empty()) { slack_bounds = DM::zeros(lbx.size()); }
        if (slack_weights.is_empty()) { slack_weights = DM::ones(slack_bounds.size()); }
        /* Ensure column vector shape */
        slack_bounds = DM::vec(slack_bounds);
        slack_weights = DM::vec(slack_weights);

        /* Fill up potentially missing entries */
        slack_bounds = DM::vertcat({DM::vec(slack_bounds), DM::zeros(lbx.numel() - slack_bounds.numel(), 1)});
        slack_weights = DM::vertcat({DM::vec(slack_weights), DM::ones(lbx.numel() - slack_weights.numel(), 1)});

        /* Loop through all states */
        for (int ix = 0; ix < nx; ++ix)
        {
            if (ix < slack_bounds.numel() &&
                lbx(ix).nonzeros()[0] < INFINITY && slack_bounds(ix).nonzeros()[0] > 0)
            {
                /* A non-zero slack bound was specified for this state. */
                sym_t slack = opti->variable();       // Create a new slack variable

                // Set constraint on state using slack and constraint on slack
                opti->subject_to(lbx(ix) - slack <= X_sym(ix, casadi::Slice()));
                opti->subject_to(0 <= slack <= slack_bounds(ix));

                // Add penalty for slack usage to cost
                sym_t cost = opti->f();
                sym_t slack_penalty = slack_weights(ix) * 0.5 * (slack * slack + slack);
                opti->minimize(cost + slack_penalty);

                std::cout << "State " << std::to_string(ix) << " lower bound: "
                          << std::to_string(lbx(ix).nonzeros()[0])
                          << " (- " << std::to_string(slack_bounds(ix).nonzeros()[0]) << ")\n";
            }
            else
            {
                /* Set the hard constraint. */
                opti->subject_to(lbx(ix) <= X_sym(ix, casadi::Slice()));
                std::cout << "State " << std::to_string(ix) << " lower bound: "
                          << std::to_string(lbx(ix).nonzeros()[0]) << "\n";
            }
        }
    }
    void set_ubu(const sym_t &ubu)
    {
        if (ubu.size2() == 1)
        {
            opti->subject_to(sym_t::vec(U_sym) <= sym_t::repmat(ubu, U_sym.size2(), 1));
        }
        else
        {
            // ubu is matrix of same size as U_sym
            opti->subject_to(sym_t::vec(U_sym) <= ubu);
        }
    }
    void set_lbu(const sym_t &lbu)
    {
        if (lbu.size2() == 1)
        {
            opti->subject_to(sym_t::repmat(lbu, U_sym.size2(), 1) <= sym_t::vec(U_sym));
        }
        else
        {
            // lbu is matrix of same size as U_sym
            opti->subject_to(lbu <= sym_t::vec(U_sym));
        }
    }

    /* Setters for initial guesses */
    virtual void set_X_guess(const DM &X_guess)
    {
        opti->set_initial(X_sym, X_guess);
    }
    virtual void set_U_guess(const DM &U_guess)
    {
        opti->set_initial(U_sym, U_guess);
    }

    void print()
    {
        opti->disp(std::cout);
        std::cout << "\n";
    }
    /* Call for OCP solve */
    bool solve(bool warm_start = true)
    {
        solve_succeeded = false;

        try
        {
//            myOptiCallback = MyOptiCallback(opti);
//            opti->callback_class(&myOptiCallback);
//            // Before solving, export NLP
//            std::cout << "=====================\n"
//                         "Before solve\n"
//                         "=====================\n";
//            print_opti_properties(opti->advanced(), opti->initial());

//            const casadi::OptiSol sol_ = opti->solve();
            const casadi::OptiSol sol_ = opti->solve_limited();

            const casadi::GenericType rs = sol_.stats()["return_status"];
            return_status = rs.is_string() ? rs.as_string() : std::to_string(rs.as_int());
            unified_return_status = sol_.stats()["unified_return_status"].as_string();
            std::cout << solver_name << ": " <<  return_status << " (" << unified_return_status << ")\n";
            solve_succeeded = sol_.stats()["success"];

            X_opt = sol_.value(X_sym);
            U_opt = sol_.value(U_sym);
            lam_g_opt = sol_.value(opti->lam_g());
            cost_opt = sol_.value(opti->f()).nonzeros()[0];
            stats = sol_.stats();
            iter_count = stats["iter_count"].as_int();

            if (warm_start)
            {
                // Use the current solution to speed up the next optimization
                opti->set_initial(X_sym, X_opt);        // X_opt is a DM, not an expression
                opti->set_initial(U_sym, U_opt);
                opti->set_initial(opti->lam_g(), lam_g_opt);
                opti->set_initial(sol_.value_variables());
            }
            solution_avail = true;
        }
        catch (const std::exception &e)
        {
            const casadi::OptiAdvanced opti_debug = opti->debug();

            const casadi::GenericType rs = opti_debug.stats()["return_status"];
            return_status = rs.is_string() ? rs.as_string() : std::to_string(rs.as_int());
            unified_return_status = opti_debug.stats()["unified_return_status"].as_string();
            std::cout << solver_name << ": " <<  return_status << " (" << unified_return_status << ")\n";

            std::cout << e.what() << "\n";
//            std::cout << "X_debug: " << opti->debug().value(X_sym) << "\n";
//            std::cout << "U_debug: " << opti->debug().value(U_sym) << "\n";

            X_opt = opti_debug.value(X_sym);
            U_opt = opti_debug.value(U_sym);
            lam_g_opt = opti_debug.value(opti->lam_g());
            cost_opt = opti_debug.value(opti->f()).nonzeros()[0];
            stats = opti_debug.stats();
            iter_count = (solver_name == "fatrop") ? -1 : stats["iter_count"].as_int();

            opti->set_initial(X_sym, X_opt);        // X_opt is a DM, not an expression
            opti->set_initial(U_sym, U_opt);
            opti->set_initial(opti->lam_g(), lam_g_opt);
            opti->set_initial(opti_debug.value_variables()); // Use unconverged solution as warm start

//            std::cout << "=====================\n"
//                         "After solve (debug)\n"
//                         "=====================\n";
//            print_opti_properties(opti_debug, opti_debug.value_variables());
        }

        return solve_succeeded;
    }
    void reset_primal_dual_vars()
    {
        using namespace casadi;
        opti->set_initial(X_sym, DM::zeros(X_sym.size()));
        opti->set_initial(U_sym, DM::zeros(U_sym.size()));
        opti->set_initial(opti->lam_g(), DM::zeros(opti->lam_g().size()));
    }
    void reset()
    {
        reset_primal_dual_vars();
        cost_opt = -999;
    }

    /* Call for OCP solve */
    DM get_u0() const
    {
        if (U_opt.is_empty()) { return {}; }
        return U_opt(casadi::Slice(), 0);
    }
    DM get_T_opt() const { return tf * T; }
    DM get_U_opt() const
    {
        if (U_opt.is_empty()) { return {}; }
        return U_opt;
    }
    DM get_X_opt() const
    {
        if (X_opt.is_empty()) { return {}; }
        return X_opt;
    }
    double get_cost() const
    {
        return cost_opt;
    }
    int get_iterations() const
    {
        return iter_count;
    }
    casadi::Dict get_stats() const
    {
        return stats;
    }

//    void print_opti_properties(const casadi::OptiAdvanced &adv, const std::vector<casadi::MX> &dat)
//    {
//        std::cout << "f: " << adv.value(adv.f(), dat) << "\n";
//        std::cout << "g: " << adv.value(adv.g(), dat) << "\n";
////        std::cout << "lam_g: " << adv.value(adv.lam_g(), dat) << "\n";
////        std::cout << "lbg: " << adv.lbg() << "\n";
//
//        sym_t jac_f_x = sym_t::jacobian(adv.f(), adv.x());
//        sym_t hes_f_xx = sym_t::hessian(adv.f(), adv.x());
//        std::cout << "jac_f_x: " <<  adv.value(jac_f_x, dat) << "\n";
//        std::cout << "hes_f_xx: " << adv.value(hes_f_xx, dat) << "\n";
//
////        sym_t jac_g_x = sym_t::jacobian(adv.g(), adv.x());
////        sym_t hes_g_xx = sym_t::hessian(adv.g(), adv.x());
////        std::cout << "jac_g_x: " <<  adv.value(jac_g_x, dat) << "\n";
////        std::cout << "hes_g_xx: " << adv.value(hes_g_xx, dat) << "\n";
//
//        std::cout << "\n";
//        std::cout << "active_symvar OPTI_VAR: " << adv.active_symvar(casadi::OPTI_VAR) << "\n";
//        std::cout << "active_values OPTI_VAR: " << adv.active_values(casadi::OPTI_VAR) << "\n";
//        std::cout << "active_symvar DUAL_G: " << adv.active_symvar(casadi::OPTI_DUAL_G) << "\n";
//        std::cout << "active_values DUAL_G: " << adv.active_values(casadi::OPTI_DUAL_G) << "\n";
//
//        casadi::DMDict dict = adv.arg();
//        for (auto it : dict)
//        {
//            std::cout << it.first << ": " << it.second << "\n";
//        }
//    }

//    class MyOptiCallback : public casadi::OptiCallback
//    {
//    public:
//        explicit MyOptiCallback(std::shared_ptr<casadi::Opti> &opti_) : m_opti(opti_) {}
//        void call(casadi_int i) override
//        {
////            std::cout << "iteration " << i << " callback. " << m_opti->debug().value(m_opti->f()) << "\n";
//        }
//        std::shared_ptr<casadi::Opti> m_opti;
//    } myOptiCallback{opti};

    casadi::Function plant_dynamics;
    int nx{0}, nu{0}, np{0}, nd{0};

    double tf{0};
    int N{0};       // Highest trajectory point index
    DM T;

    std::string solver_name{};
    casadi::Dict opti_opts;
    casadi::Dict jit_opts;
    casadi::Dict solver_opts;
    std::shared_ptr<casadi::Opti> opti;
    bool solution_avail{false};
    bool solve_succeeded{false};
    std::string return_status;
    std::string unified_return_status;

    /* Decision variables */
    sym_t X_sym;
    sym_t U_sym;
//    sym_t m_slacks_sym; // For debugging

    /* Virtual implementation functions */
    virtual void setup_discretized_trajectory_constraints() {}
    virtual sym_t lagrange_cost_impl(const sym_t &x, const sym_t &u, const sym_t &p, const sym_t &d, const sym_t &t) = 0;
    virtual sym_t mayer_cost_impl(const sym_t &x, const sym_t &p, const sym_t &d, const sym_t &tf) = 0;
    virtual sym_t cont_dynamics_impl(const sym_t &x, const sym_t &u, const sym_t &p, const sym_t &d) = 0;
    virtual sym_t eq_constraints_impl(const sym_t &x, const sym_t &u, const sym_t &p, const sym_t &d, const sym_t &t) = 0; // == 0
    virtual sym_t ineq_constraints_impl(const sym_t &x, const sym_t &u, const sym_t &p, const sym_t &d, const sym_t &t) = 0; // < 0

protected:
    void init_base(const casadi::Function &f_cont = {}, double tf_ = 1.0)
    {
        /* Setup problem dimensions */
        nx = f_cont.size1_in(0);
        nu = f_cont.size1_in(1);
        np = (f_cont.n_in() > 2 ? f_cont.size1_in(2) : 0);
        nd = (f_cont.n_in() > 3 ? f_cont.size1_in(3) : 0);
        plant_dynamics = f_cont;

        tf = tf_;

        opti = std::make_shared<casadi::Opti>();

        /* Setup solver */
        setup_solver();
    }

    DM X_opt;
    DM U_opt;
    DM lam_g_opt;
    double cost_opt{-999};
    int iter_count{-1};
    casadi::Dict stats;
};

} // namespace ocp_casadi
} // namespace mpc_common

#endif //SRC_OPTIOCP_HPP
