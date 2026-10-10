#ifndef SRC_OPTIOCPBASE_HPP
#define SRC_OPTIOCPBASE_HPP

#include <memory>
#include "casadi/casadi.hpp"

namespace ocp_casadi {

using namespace casadi;

template<typename return_t>
void parse_dict(const Dict &dict, const std::string &key, return_t &set_to)
{
    if (dict.find(key) != dict.end())
        set_to = static_cast<return_t>(dict.at(key));
    else
        std::cout << "Parsing options: Option '" << key << "' could not be found.\n";
}

class OptiOcpBase
{
public:
    using sym_t = MX;
    using symvec_t = std::vector<sym_t>;

    void set_solver_opts(Dict opts = Dict())
    {
        /* Setup solver */
        if (opts.empty())
        {
            opts = Dict({{"ipopt.print_level", 0},    // 5: print iterations and scaling factor
                         {"ipopt.max_iter",    100},
                         {"ipopt.tol",         1e-3}});
        }
        std::cout << "Setting IPOPT solver options: " << opts << "\n";
        opti->solver("ipopt", opts);
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
                opti->subject_to(X_sym(ix, Slice()) <= ubx(ix) + slack);
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
                opti->subject_to(X_sym(ix, Slice()) <= ubx(ix));
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
                opti->subject_to(lbx(ix) - slack <= X_sym(ix, Slice()));
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
                opti->subject_to(lbx(ix) <= X_sym(ix, Slice()));
                std::cout << "State " << std::to_string(ix) << " lower bound: "
                          << std::to_string(lbx(ix).nonzeros()[0]) << "\n";
            }
        }
    }
    void set_ubu(const DM &ubu)
    {
        opti->subject_to(sym_t::vec(U_sym) <= sym_t::repmat(ubu, U_sym.size2(), 1));
    }
    void set_lbu(const DM &lbu)
    {
        opti->subject_to(sym_t::repmat(lbu, U_sym.size2(), 1) <= sym_t::vec(U_sym));
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
        bool solve_success{false};

        try
        {
            OptiSol sol_ = opti->solve();

            solve_success = sol_.stats()["success"];
            //casadi_assert(solve_success, "Error computing optimal input");

            if (warm_start)
            {
                // Use the current solution to speed up the next optimization
                opti->set_initial(sol_.value_variables());
                opti->set_initial(opti->lam_g(), sol_.value(opti->lam_g()));
            }
            sol = std::make_shared<OptiSol>(sol_);
        }
        catch (const std::exception &e)
        {
            std::cout << e.what() << "\n";
        }

        return solve_success;
    }

    /* Call for OCP solve */
    DM get_u0() const { return sol->value(U_sym(Slice(), 0)); }

    /* Getters for solution parts */
    Dict get_stats() { return sol->stats(); }

    Function plant_dynamics;
    int nx{0}, nu{0}, np{0}, nd{0};

    double tf{0};
    int N{0};       // Highest trajectory point index
    DM T;

    std::shared_ptr<Opti> opti;
    std::shared_ptr<OptiSol> sol;

    /* Decision variables */
    sym_t X_sym;
    sym_t U_sym;
//    sym_t m_slacks_sym; // For debugging

    /* Virtual implementation functions */
    void setup_discretized_trajectory_constraints() {}
    sym_t lagrange_cost_impl(const sym_t &x, const sym_t &u, const sym_t &p, const sym_t &d) { return {0}; }
    sym_t mayer_cost_impl(const sym_t &x, const sym_t &p, const sym_t &d) { return {0}; }
    sym_t cont_dynamics_impl(const sym_t &x, const sym_t &u, const sym_t &p, const sym_t &d) { return {0}; }

protected:
    void init(const Function &f_cont = Function(), double tf_ = 1.0, Dict solver_options = Dict())
    {
        /* Setup problem dimensions */
        nx = f_cont.size1_in(0);
        nu = f_cont.size1_in(1);
        np = f_cont.size1_in(2);
        nd = f_cont.size1_in(3);
        plant_dynamics = f_cont;

        tf = tf_;

        opti = std::make_shared<Opti>();

        /* Setup solver */
        set_solver_opts();
    }
};

} // namespace ocp_casadi

#endif //SRC_OPTIOCPBASE_HPP
