#ifndef SRC_INVERTEDPENDULUMSIMPLEUOCP_HPP
#define SRC_INVERTEDPENDULUMSIMPLEUOCP_HPP

#include "mpc_control_common/casadi_ocp/UOcpBase.hpp"
#include "InvertedPendulum.hpp"

class InvertedPendulumSimpleUOcp : public mpc_control_common::ocp_casadi::UOcpBase
{
public:
    InvertedPendulumSimpleUOcp()
    {
        InvertedPendulum invertedPendulum("");
        casadi::Function f_cont = invertedPendulum.getNumericDynamics(false, true);

        model = f_cont;
        mpc_control_common::ocp_casadi::UOcpBase::init();

        /* Define problem-specific parameters and set default values
         * For each, a symbolic parameter will be created to change them
         * after the creation of the problem */
        params.insert({"angle_ref", 0});
        params.insert({"mayer_multiplier", 10});
        params.insert({"W_angle_err", 10});
        params.insert({"R_diag", 1});
        params.insert({"w_tf", 0});

        /* Optimization parameters with their bounds */
        opt_params.insert({"ref_offset", 0});
        opt_params_ub.insert({"ref_offset", 0});
        opt_params_lb.insert({"ref_offset", 0});

        opt_params.insert({"us", 0});
        opt_params_ub.insert({"us", 0});
        opt_params_lb.insert({"us", 0});
    }

    /* Override functions from base class --------------------------------------------------------------------------- */
    template<typename sym_t = casadi::SX>
    sym_t lagrange_cost_impl(const sym_t &x, const sym_t &u, const sym_t &p, const sym_t &d, const sym_t &t)
    {
        return get_non_control_cost(x, t) + get_control_cost(u, t);
    }

    template<typename sym_t = casadi::SX>
    sym_t mayer_cost_impl(const sym_t &xf, const sym_t &p, const sym_t &d, const sym_t &tf)
    {
        sym_t cost(0);
        cost += sym<sym_t>().params.at("mayer_multiplier") * get_non_control_cost(xf, tf);
        cost += sym<sym_t>().params.at("w_tf") * tf;
        return cost;
    }

    template<typename sym_t = casadi::SX>
    sym_t cont_dynamics_impl(const sym_t &x, const sym_t &u, const sym_t &p, const sym_t &d)
    {
        sym_t x_dot = model(std::vector<sym_t>({x, u, p, d}))[0];
        return x_dot;
    }

    template<typename sym_t = casadi::SX>
    std::vector<Constraint>
    equality_constraints_impl(const sym_t &x, const sym_t &u)
    {
        std::vector<Constraint> eq_constr;
        eq_constr.emplace_back(Constraint());
        eq_constr[0].t_from = 0;            // Relative horizon time [0 ... 1]
        eq_constr[0].expr = (x(0) - 0.2);   // Expression (== 0)

//        eq_constr.emplace_back(Constraint());
//        eq_constr[1].t_from = 0.1;          // Relative horizon time [0 ... 1]
//        eq_constr[1].t_to = 0.2;            // Relative horizon time [0 ... 1]
//        eq_constr[1].expr = (u(0) - -2);    // Expression (== 0)

        return eq_constr; // == 0
    }

    template<typename sym_t = casadi::SX>
    std::vector<Constraint>
    inequality_constraints_impl(const sym_t &x, const sym_t &u)
    {
        std::vector<Constraint> ineq_constr;
        ineq_constr.emplace_back(Constraint());
        ineq_constr[0].t_from = 0.5;          // Relative horizon time [0 ... 1]
        ineq_constr[0].expr = (-x(0) + 0.2);  // Expression (<= 0)

        ineq_constr.emplace_back(Constraint());
        ineq_constr[1].t_from = 0.1;          // Relative horizon time [0 ... 1]
        ineq_constr[1].t_to = 0.5;            // Relative horizon time [0 ... 1]
        ineq_constr[1].expr = (-u(0) - 2.8);  // Expression (<= 0)
        return ineq_constr; // <= 0
    }

    /* OCP-specific setters ----------------------------------------------------------------------------------------- */
    void set_angle_ref(double angle_ref) { params.at("angle_ref") = angle_ref; }
    void set_W_angle_ref(double W_angle_ref) { params.at("W_angle_ref") = W_angle_ref; }
    void set_R_diag(const casadi::DM &R_diag) { params.at("R_diag") = R_diag; }
    void set_mayer_multiplier(double mayer_multiplier) { params.at("mayer_multiplier") = mayer_multiplier; }
    void set_w_tf(double w_tf) { params.at("w_tf") = w_tf; }

private:
    /* Helpers for non-/control stage costs */
    template<typename sym_t = casadi::SX>
    sym_t get_non_control_cost(const sym_t &x, const sym_t &t)
    {
        sym_t non_control_cost(0);

        /* Objective: Track angle */
        sym_t angle_ref = sym<sym_t>().params.at("angle_ref") + sym<sym_t>().opt_params.at("ref_offset");
        sym_t angle_err = angle_ref - x(0);
        non_control_cost += sym<sym_t>().params.at("W_angle_err") * angle_err * angle_err;

        return non_control_cost;
    }

    template<typename sym_t = casadi::SX>
    sym_t get_control_cost(const sym_t &u, const sym_t &t)
    {
        sym_t control_cost(0);

        sym_t u_act = u + sym<sym_t>().opt_params.at("us");
        control_cost += sym_t::mtimes(u_act.T(), sym_t::mtimes(sym_t::diag(sym<sym_t>().params.at("R_diag")), u_act));

        return control_cost;
    }

};

#endif //SRC_INVERTEDPENDULUMSIMPLEUOCP_HPP
