#ifndef SRC_UOCPBASE_HPP
#define SRC_UOCPBASE_HPP

#include <memory>

#include "casadi/casadi.hpp"
#include "mpc_control_common2/casadi_transcription/UTranscriptionBase.hpp"
#include "mpc_control_common2/casadi_solver/USolverBase.hpp"

namespace mpc_control_common2 {
namespace ocp_casadi {

class UOcpBase
{
public:
    casadi::Function model;
    unsigned nx{0};
    unsigned nu{0};
    unsigned np{0};
    unsigned nd{0};

    casadi::DMDict params;

    casadi::DM u_lb, u_ub;
    casadi::DM x_lb, x_ub;
    casadi::DM x0_lb, x0_ub;
    casadi::DM xf_lb, xf_ub;

    double tf_lb{0}, tf_ub{2};
    casadi::DMDict opt_params;
    casadi::DMDict opt_params_lb, opt_params_ub;

    struct Constraint
    {
        double t_from{0}; // Relative horizon time [0 ... 1]
        double t_to{0};   // Relative horizon time [0 ... 1]
        casadi::SX expr;
    };
    std::vector<Constraint> eq_constr;
    std::vector<Constraint> ineq_constr;

    /* Discretization and solution */
    template<typename expr_t>
    struct Sym
    {
        std::map<std::string, expr_t> params;
        std::map<std::string, expr_t> opt_params;
        std::map<std::string, expr_t> opt_params_lb;
        std::map<std::string, expr_t> opt_params_ub;

        expr_t u_lb, u_ub;
        expr_t x_lb, x_ub;
        expr_t x0_lb, x0_ub;
        expr_t xf_lb, xf_ub;
        expr_t tf_lb, tf_ub;
    };
    template<typename sym_t>
    typename std::enable_if<std::is_same<sym_t, casadi::SX>::value, Sym<casadi::SX>>::type &sym() { return sym_SX; }
    template<typename sym_t>
    typename std::enable_if<std::is_same<sym_t, casadi::MX>::value, Sym<casadi::MX>>::type &sym() { return sym_MX; }

//    std::shared_ptr<mpc_control_common::casadi_transcription::UTranscriptionBase> transcription;
    double comp_time{-1};

    explicit UOcpBase(const casadi::Function &model_)
    {
        /* Dynamics */
        model = model_;
        init();
    }
    UOcpBase() = default;
    void init()
    {
        nx = model.size1_in(0);
        nu = model.size1_in(1);
        np = (model.n_in() > 2 ? model.size1_in(2) : 0);
        nd = (model.n_in() > 3 ? model.size1_in(3) : 0);

        /* OCP */
        /* Define problem-specific optimization parameters and set default values or nan if unknown */
        using namespace casadi;
        u_lb = -DM::inf(nu, 1);
        u_ub = +DM::inf(nu, 1);
        x_lb = -DM::inf(nx, 1);
        x_ub = +DM::inf(nx, 1);

        x0_lb = -DM::inf(nx, 1);
        x0_ub = +DM::inf(nx, 1);
        xf_lb = -DM::inf(nx, 1);
        xf_ub = +DM::inf(nx, 1);

        tf_lb = 1;
        tf_ub = 1;

        params.insert({"dyn_params", DM::zeros(nd, 1)});
    }

    void set_lbx(const casadi::DM &lbx) { x_lb = lbx; }
    void set_ubx(const casadi::DM &ubx) { x_ub = ubx; }
    void set_lbu(const casadi::DM &lbu) { u_lb = lbu; }
    void set_ubu(const casadi::DM &ubu) { u_ub = ubu; }
    void set_x0(const casadi::DM &x0)
    {
        x0_lb = x0(casadi::Slice(), 0);
        x0_ub = x0(casadi::Slice(), -1);
    }
    void set_xf(const casadi::DM &xf)
    {
        xf_lb = xf(casadi::Slice(), 0);
        for (unsigned i = 0; i < xf_lb.size1(); i++)
        {
            if (xf_lb(i).nonzeros()[0] == INFINITY) { xf_lb(i) = -xf_lb; }
        }

        xf_ub = xf(casadi::Slice(), -1);
    }
    void set_tf(const casadi::DM &tf)
    {
        tf_lb = tf(casadi::Slice(), 0).nonzeros()[0];
        tf_ub = tf(casadi::Slice(), -1).nonzeros()[0];
    }

    // TODO  % Setters for initial guesses --> Needs to be in Solver class in C++ implementation

//    bool solve()
//    {
//        transcription->update_parameters(this);
//       return solver->solve();
//    }

    // TODO   %
    //        % Getters for solution parts --> Needs to be in Solver class in C++ implementation
    //        %

protected:
    void set_param(const std::string &param_name, double param_value)
    {
        if (params.find(param_name) != params.end())
        {
            params.at(param_name) = param_value;
        }
        else
        {
            std::cerr << param_name << " is not a parameter of the OCP!\n";
        }
    }

private:
    Sym<casadi::MX> sym_MX;
    Sym<casadi::SX> sym_SX;

public:
    template<typename sym_t = casadi::SX>
    sym_t lagrange_cost_impl(const sym_t &x, const sym_t &u, const sym_t &p, const sym_t &d, const sym_t &t)
    {
        return 0;
    }

    template<typename sym_t = casadi::SX>
    sym_t mayer_cost_impl(const sym_t &x, const sym_t &p, const sym_t &d, const sym_t &tf)
    {
        return 0;
    }

    template<typename sym_t = casadi::SX>
    sym_t cont_dynamics_impl(const sym_t &x, const sym_t &u, const sym_t &p, const sym_t &d)
    {
        return x / 10.0;
    }

    template<typename sym_t = casadi::SX>
    std::vector<Constraint> equality_constraints_impl(const sym_t &x, const sym_t &u)
    {
        return {}; // == 0
    }

    template<typename sym_t = casadi::SX>
    std::vector<Constraint> inequality_constraints_impl(const sym_t &x, const sym_t &u)
    {
        return {}; // < 0
    }

};

}
}

#endif //SRC_UOCPBASE_HPP
