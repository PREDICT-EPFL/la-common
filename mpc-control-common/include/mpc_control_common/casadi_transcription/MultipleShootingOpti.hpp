#ifndef SRC_MULTIPLESHOOTINGOPTI_HPP
#define SRC_MULTIPLESHOOTINGOPTI_HPP

#include <memory>
#include "casadi/casadi.hpp"

#include "mpc_control_common/casadi_ocp/OptiOcp.hpp"
#include "mpc_control_common/casadi_transcription/Integration.hpp"

namespace mpc_control_common {
namespace transcription_casadi {

class MultipleShootingOpti
{
private:
    using MX = casadi::MX;
    using DM = casadi::DM;
    using IM = casadi::IM;
    casadi::Slice all;

public:
    int N_segs{4};

    enum Integrator
    {
        RK1, RK2a, RK2b, RK4,
        IRK11, IRK12, IRK22,
    };
    static std::string get_integrator_name(const Integrator &integrator)
    {
        switch (integrator)
        {
            case RK1:
                return "(explicit) RK1 - Forward Euler";
            case RK2a:
                return "(explicit) RK2a - Improved Euler";
            case RK2b:
                return "(explicit) RK2b - Midpoint";
            case RK4:
                return "(explicit) RK4";

            case IRK11:
                return "(implicit) IRK11 - Backward Euler";
            case IRK12:
                return "(implicit) IRK12 - Midpoint";
            case IRK22:
                return "(implicit) IRK22 - Trapezoidal";

            default:
                std::cerr << "Integrator name not found!\n";
                return "[Integrator name not found!]";
        }
    }

    struct Config
    {
        int N{};
        DM T_segs;
        DM T;
        IM id_segs;
        Integrator integrator{RK4};
    } conf;

    explicit MultipleShootingOpti(int N_segs = 10, Integrator integrator = RK4)
    {
        if (N_segs > 0) { this->N_segs = N_segs; }

        // Initialize linear grid
        std::vector<double> t_segs_(N_segs + 1);
        casadi::linspace(t_segs_, 0, 1);
        conf.T = DM(t_segs_);

        switch (integrator)
        {
            case RK1:
            case RK2a:
            case RK2b:
            case RK4:
            case IRK11:
            case IRK12:
            case IRK22:
                // Integrator exists
                conf.integrator = integrator;
                break;
            default:
                std::cerr << "Unknown integration method" << "\n";
        }
    }

    template<class OCP>
    void transcribe(OCP &ocp)
    {
        setup(0, 1);

        unsigned N = conf.N;
        DM T_segs = conf.T_segs;
        DM T = conf.T;

        const std::shared_ptr<casadi::Opti> opti = ocp.opti;

        /* Create variables and indexing */
        const MX X_sym = opti->variable(ocp.nx, N + 1);
        const MX U_sym = opti->variable(ocp.nu, N);
        const MX p = MX();
        const MX d = MX();

        casadi::Function f_cont;
        {
            MX x_ = MX::sym("x_", ocp.nx);
            MX u_ = MX::sym("u_", ocp.nu);
            MX p_ = MX::sym("p_", ocp.np); //std::max(1, ocp.np)); // If np is zero, p is not used in f_cont anyway
            MX d_ = MX::sym("d_", ocp.nd); //std::max(1, ocp.nd)); // If nd is zero, d is not used in f_cont anyway
            MX x_dot_ = MX(ocp.tf) * ocp.cont_dynamics_impl(x_, u_, p_, d_);
            f_cont = casadi::Function("f_cont", {x_, u_, p_, d_}, {x_dot_});
        }
        casadi::Function dyn_constr_fun;
        {
            MX xp_ = MX::sym("xp_", ocp.nx);
            MX x_ = MX::sym("x_", ocp.nx);
            MX u_ = MX::sym("u_", ocp.nu);
            MX p_ = MX::sym("p_", ocp.np);
            MX d_ = MX::sym("d_", ocp.nd);
            MX h_ = MX::sym("h_");

            MX dyn_constr_expr;
            switch (conf.integrator)
            {
                using namespace integration;

                case RK1:
                    dyn_constr_expr = xp_ - RK11_int(f_cont, x_, u_, p_, d_, h_);
                    break;
                case RK2a:
                    dyn_constr_expr = xp_ - RK2a_int(f_cont, x_, u_, p_, d_, h_);
                    break;
                case RK2b:
                    dyn_constr_expr = xp_ - RK2b_int(f_cont, x_, u_, p_, d_, h_);
                    break;
                case RK4:
                    dyn_constr_expr = xp_ - RK4_int(f_cont, x_, u_, p_, d_, h_);
                    break;

                case IRK11:
                    dyn_constr_expr = IRK11_int(f_cont, xp_, x_, u_, p_, d_, h_);
                    break;
                case IRK12:
                    dyn_constr_expr = IRK12_int(f_cont, xp_, x_, u_, p_, d_, h_);
                    break;
                case IRK22:
                    dyn_constr_expr = IRK22_int(f_cont, xp_, x_, u_, p_, d_, h_);
                    break;

                default:
                    std::cerr << "No Integrator set!\n";
            }
            dyn_constr_fun = casadi::Function("dyn_constr_fun", {xp_, x_, u_, p_, d_, h_}, {dyn_constr_expr});
        }

        /* Objective & constraints */
        MX lagrange_cost = 0;

        for (unsigned iSeg = 0; iSeg < N; ++iSeg)
        {
            const double seg_t0 = T_segs(iSeg, 0).nonzeros()[0];
            const double seg_tf = T_segs(iSeg, -1).nonzeros()[0];

            const double h = seg_tf - seg_t0;
            const double t = ocp.tf * seg_t0;

            // Normalizing to t=0...1 (seg_tf-seg_t0) is for comparability with other transcription methods like collocation
            lagrange_cost += h * ocp.lagrange_cost_impl(X_sym(all, iSeg), U_sym(all, iSeg), p, d, t);

            /* Dynamic constraints */
            opti->subject_to(0 == dyn_constr_fun(
                    casadi::MXVector({X_sym(all, iSeg + 1), X_sym(all, iSeg), U_sym(all, iSeg), p, d, MX(h)}))[0]);

            /* Nonlinear constraints */
            MX eq_constr = ocp.eq_constraints_impl(X_sym(all, iSeg), U_sym(all, iSeg), p, d, t);
            if (eq_constr.size1() > 0) { opti->subject_to(eq_constr == 0); }

            MX ineq_constr = ocp.ineq_constraints_impl(X_sym(all, iSeg), U_sym(all, iSeg), p, d, t);
            if (ineq_constr.size1() > 0) { opti->subject_to(ineq_constr <= 0); }
        }
        const MX mayer_cost = ocp.mayer_cost_impl(X_sym(all, N), p, d, ocp.tf);

        MX cost = lagrange_cost + mayer_cost;
        opti->minimize(cost);

        /* */
        ocp.X_sym = X_sym;
        ocp.U_sym = U_sym;
        ocp.T = T;

        ocp.setup_discretized_trajectory_constraints();
    }

//    void resample_trajectory(const DM &Xcol, double dt, DM &Xn, DM &Tn)
//    {
//        const DM id_segs = conf.id_segs;
//        const DM T_segs = conf.T_segs;
//
//        const IM segs_n_ = DM::ceil((T_segs(all, T_segs.size2() - 1) - T_segs(all, 0)) / dt);
//        const IM segs_n = IM::vertcat({0, IM::cumsum(segs_n_)});
//        const IM segs_idn = IM::horzcat({segs_n(casadi::Slice(0, -1)), segs_n(casadi::Slice(1, segs_n.size1()))});
//
//        const casadi_int n = segs_idn(segs_idn.size1() - 1, segs_idn.size2() - 1).nonzeros()[0];
//        Tn = DM::zeros(1, n + 1);
//        Xn = DM::zeros(Xcol.size1(), n + 1);
//
//        for (int iSeg = 0; iSeg < T_segs.size1(); ++iSeg)
//        {
//            const IM id_seg = id_segs(iSeg, all);
//            const IM idn_seg = segs_idn(iSeg, all);
//
//            const double t0_seg = T_segs(iSeg, 0).nonzeros()[0];
//            const double tf_seg = T_segs(iSeg, T_segs.size2() - 1).nonzeros()[0];
//
//            const DM X_seg = Xcol(all, casadi::Slice(id_seg(0).nonzeros()[0], id_seg(1).nonzeros()[0] + 1));
//
//            std::vector<double> Tn_seg(segs_n_(iSeg).nonzeros()[0] + 1);
//            casadi::linspace(Tn_seg, t0_seg, tf_seg);
//
//            const DM Tau_seg = t2tau(Tn_seg, t0_seg, tf_seg);
//            DM Xn_seg = DM::zeros(Xcol.size1(), segs_n_(iSeg).nonzeros()[0] + 1);
//            for (int i = 0; i < Xn_seg.size2(); i++)
//            {
//                Xn_seg(all, i) = apr(X_seg, Tau_seg(i).nonzeros()[0]);
//            }
//
//            Tn(casadi::Slice(idn_seg(0).nonzeros()[0], idn_seg(1).nonzeros()[0] + 1)) = Tn_seg;
//            Xn(all, casadi::Slice(idn_seg(0).nonzeros()[0], idn_seg(1).nonzeros()[0] + 1)) = Xn_seg;
//        }
//
//    }

private:
    void setup(double t0, double tf)
    {
        std::cout << "Transcription: Multiple Shooting. N_segs: " << N_segs
                  << " (Integration: " << get_integrator_name(conf.integrator) << ")\n";

        const int N = N_segs; // Last index

        if (conf.T.is_empty())
        {
            /* Setup mesh */
            std::vector<double> t_segs_(N + 1);
            casadi::linspace(t_segs_, t0, tf);
            DM t_segs = DM(t_segs_);
            conf.T = t_segs;
        }
        DM T = conf.T;
        if (T.size2() > T.size1()) { T = T.T(); } // Use as column vector

        DM T_segs = DM::horzcat({T(casadi::Slice(0, -1)), T(casadi::Slice(1, T.size1()))});

        if (T.size1() > T.size2()) { T = T.T(); } // Save as row vector

        conf.N = N;
        conf.T_segs = T_segs;
        conf.T = T;

//        std::cout << "t_segs: " << t_segs << "\n";
//        std::cout << "T_segs: " << T_segs << "\n";
//        std::cout << "T: " << T << "\n";
    }

};

} // namespace casadi_transcription
} // namespace mpc_control_common

#endif //SRC_MULTIPLESHOOTINGOPTI_HPP
