#ifndef SRC_MULTIPLESHOOTINGUOPTI_HPP
#define SRC_MULTIPLESHOOTINGUOPTI_HPP

#include <memory>

#include "casadi/casadi.hpp"
#include "mpc_control_common2/casadi_transcription/UTranscriptionBaseOpti.hpp"
#include "mpc_control_common2/casadi_solver/USolverOpti.hpp"
#include "mpc_control_common/casadi_transcription/Integration.hpp"

namespace mpc_control_common2 {
namespace casadi_transcription {

class MultipleShootingUOpti : public UTranscriptionBaseOpti
{
private:
    using MX = casadi::MX;
    using DM = casadi::DM;
    using IM = casadi::IM;
    casadi::Slice all;

public:
    unsigned N_segs{10};
    struct Config
    {
        int N{};
        DM T_segs;
        DM T;
        IM id_segs;
    } conf;

    explicit MultipleShootingUOpti(unsigned N_segs_)
    {
        if (N_segs > 0) { N_segs = N_segs_; }
    }

    template<class UOcp>
    std::shared_ptr<casadi_solver::USolverOpti> transcribe(UOcp &ocp)
    {
        setup(0, 1);

        unsigned N = conf.N;
        DM T_segs = conf.T_segs;
        DM T = conf.T;

        casadi::Opti opti = casadi::Opti();

        /* Create variables and indexing */
        const MX X_sym = opti.variable(ocp.nx, N + 1);
        const MX U_sym = opti.variable(ocp.nu, N);
        const MX tf_sym = opti.variable(1);

        ocp.template sym<MX>().u_lb = opti.parameter(ocp.nu, 1);
        ocp.template sym<MX>().u_ub = opti.parameter(ocp.nu, 1);
        ocp.template sym<MX>().x_lb = opti.parameter(ocp.nx, 1);
        ocp.template sym<MX>().x_ub = opti.parameter(ocp.nx, 1);

        ocp.template sym<MX>().x0_lb = opti.parameter(ocp.nx, 1);
        ocp.template sym<MX>().x0_ub = opti.parameter(ocp.nx, 1);
        ocp.template sym<MX>().xf_lb = opti.parameter(ocp.nx, 1);
        ocp.template sym<MX>().xf_ub = opti.parameter(ocp.nx, 1);

        ocp.template sym<MX>().tf_lb = opti.parameter(1);
        ocp.template sym<MX>().tf_ub = opti.parameter(1);

        /* Create symbolic variables for additional optimization parameters */
        ocp.template sym<MX>().opt_params = struct2sym(opti, CreateType::Variable, ocp.opt_params);

        /* Create symbolic parameters for static OCP parameters/bounds */
        ocp.template sym<MX>().params        = struct2sym(opti, CreateType::Parameter, ocp.params);
        ocp.template sym<MX>().opt_params_lb = struct2sym(opti, CreateType::Parameter, ocp.opt_params_lb);
        ocp.template sym<MX>().opt_params_ub = struct2sym(opti, CreateType::Parameter, ocp.opt_params_ub);

        const MX p = MX();
        const MX d = MX();

        casadi::Function f_cont, f_discrete;
        {
            MX x_ = MX::sym("x_", ocp.nx);
            MX u_ = MX::sym("u_", ocp.nu);
            MX p_ = MX::sym("p_", ocp.np); //std::max(1, ocp.np)) If np is zero, the input is not used in f_cont anyway
            MX d_ = MX::sym("d_", ocp.nd); //std::max(1, ocp.nd)) If nd is zero, the input is not used in f_cont anyway
            MX x_dot_ = tf_sym * ocp.cont_dynamics_impl(x_, u_, p_, d_);
            f_cont = casadi::Function("f_cont", {x_, u_, p_, d_}, {x_dot_}, casadi::Dict({{"allow_free", true}}));

            MX h_ = MX::sym("h_");
            MX x_next = mpc_control_common::transcription_casadi::integration::RK4_int(f_cont, x_, u_, p_, d_, h_);
            f_discrete = casadi::Function("f_discrete", {x_, u_, p_, d_, h_}, {x_next}, casadi::Dict({{"allow_free", true}}));
        }

        /* Objective & constraints */
        MX lagrange_cost = 0;

        for (unsigned iSeg = 0; iSeg < N; ++iSeg)
        {
            const double seg_t0 = T_segs(iSeg, 0).nonzeros()[0];
            const double seg_tf = T_segs(iSeg, -1).nonzeros()[0];

            const double h = seg_tf - seg_t0;
            MX t = tf_sym * seg_t0;

            // Normalizing to t=0...1 (seg_tf-seg_t0) is for comparability with other transcription methods like collocation
            lagrange_cost += h * ocp.lagrange_cost_impl(X_sym(all, iSeg), U_sym(all, iSeg), p, d, t);

            /* Dynamic constraints */
            opti.subject_to(X_sym(all, iSeg + 1) ==
                            f_discrete(casadi::MXVector({X_sym(all, iSeg), U_sym(all, iSeg), p, d, MX(h)}))[0]);
        }
        const MX mayer_cost = ocp.mayer_cost_impl(X_sym(all, N), p, d, tf_sym);
        // opti->subject_to( dapr(X_seg, 1) == ocp.cont_dynamics_impl(X_seg(all,-1), U_seg(all,-1)) );

        const MX cost = lagrange_cost + mayer_cost;
        opti.minimize(cost);

        /* Box constraints */
        opti.subject_to(
                MX::repmat(ocp.template sym<MX>().x_lb, 1, X_sym.size2())
                <= X_sym <=
                MX::repmat(ocp.template sym<MX>().x_ub, 1, X_sym.size2())
        );
        opti.subject_to(
                MX::repmat(ocp.template sym<MX>().u_lb, 1, U_sym.size2())
                <= U_sym <=
                MX::repmat(ocp.template sym<MX>().u_ub, 1, U_sym.size2())
        );

        /* Opt params constraints */
        if (!ocp.template sym<MX>().opt_params.empty())
        {
            const MX opt_params    = sym2vec(ocp.template sym<MX>().opt_params);
            const MX opt_params_lb = sym2vec(ocp.template sym<MX>().opt_params_lb);
            const MX opt_params_ub = sym2vec(ocp.template sym<MX>().opt_params_ub);
            opti.subject_to(opt_params_lb <= vec(opt_params) <= opt_params_ub);
        }

        /* Boundary constraints */
        opti.subject_to(ocp.template sym<MX>().x0_lb <= X_sym(all, 0) <= ocp.template sym<MX>().x0_ub);
        opti.subject_to(ocp.template sym<MX>().xf_lb <= X_sym(all, N) <= ocp.template sym<MX>().xf_ub);
        opti.subject_to(ocp.template sym<MX>().tf_lb <= tf_sym <= ocp.template sym<MX>().tf_ub);

        /* Nonlinear user constraints */
        MX user_eq_constrs = get_user_constraints(ocp, T, X_sym, U_sym);
        if (user_eq_constrs.size1() > 0) { opti.subject_to(user_eq_constrs == 0); }

        MX user_ineq_constr = get_user_constraints(ocp, T, X_sym, U_sym);
        if (user_ineq_constr.size1() > 0) { opti.subject_to(user_ineq_constr <= 0); }

        /* ---- Setup solver ---- */
        std::shared_ptr<casadi_solver::USolverOpti> solver =
                std::make_shared<casadi_solver::USolverOpti>(opti); // Copying opti, don't use hereafter

        casadi::Dict solver_opts{};
        solver->set_solver_opts(solver_opts, false);

        solver->X_sym = X_sym;
        solver->U_sym = U_sym;
        solver->tf_sym = tf_sym;
        solver->T_normalized = T;
        solver->opt_params_sym = ocp.template sym<MX>().opt_params;

        return solver;
    }

protected:
    void setup(double t0, double tf)
    {
        const int N = N_segs; // Last index

        /* Setup mesh */
        std::vector<double> t_segs_(N + 1);
        casadi::linspace(t_segs_, t0, tf);
        DM t_segs = DM(t_segs_);
        t_segs = DM::horzcat({t_segs(casadi::Slice(0, -1)), t_segs(casadi::Slice(1, t_segs.size1()))});

        const DM T_segs = t_segs;

        /* */
        const DM T = DM::vertcat({T_segs(all, 0), tf}).T();

        conf.N = N;
        conf.T_segs = T_segs;
        conf.T = T;
        std::cout << "Transcription: Multiple Shooting. N_segs: " << N_segs << "\n";

//        std::cout << "t_segs: " << t_segs << "\n";
//        std::cout << "T_segs: " << T_segs << "\n";
//        std::cout << "T: " << T << "\n";
    }

};

}
}

#endif //SRC_MULTIPLESHOOTINGUOPTI_HPP
