#ifndef SRC_RADAUCOLLOCATIONOPTI_HPP
#define SRC_RADAUCOLLOCATIONOPTI_HPP

#include <memory>

#include "casadi/casadi.hpp"
#include "mpc_control_common/casadi_ocp/OptiOcp.hpp"

namespace mpc_control_common {
namespace transcription_casadi {

class RadauCollocationOpti
{
private:
    using MX = casadi::MX;
    using DM = casadi::DM;
    using IM = casadi::IM;
    casadi::Slice all;

public:
    int N_segs{4};
    DM Tau = DM::horzcat({0, 0.1398, 0.4164, 0.7232, 0.9429, 1.0000});

    struct Config
    {
        int D_poly{};
        int N{};
        DM T_segs;
        DM T;
        IM id_segs;
    } conf;

    explicit RadauCollocationOpti(int N_segs = 1, int D_poly = 5)
    {
        if (N_segs > 0) { this->N_segs = N_segs; }

        if (D_poly > 0)
        {
            DM points = 1 - DM(casadi::collocation_points(D_poly, "radau"));
            this->Tau = DM::vertcat({points(casadi::Slice(-1, 0, -1), all), points(0, all), 1}).T();
            std::cout << "Radau Tau: " << this->Tau << "\n";
        }
    }

    void test(double tf)
    {
        setup(0, tf);

        std::cout << "D: " << dapr_mat(Tau(casadi::Slice(0, -1)).nonzeros()) << "\n";
        std::cout << "I: " << intapr_mat() << "\n";
    }

    template<class OCP>
    void transcribe(OCP &ocp)
    {
        setup(0, 1);

        const int N = conf.N;
        std::vector<int> idx_collocated(conf.D_poly);
        casadi::linspace(idx_collocated, 0, conf.D_poly - 1);
        const DM id_segs = conf.id_segs;
        const DM T_segs = conf.T_segs;
        const DM T = conf.T;

        const std::shared_ptr<casadi::Opti> opti = ocp.opti;

        /* Create variables and indexing */
        const MX X_sym = opti->variable(ocp.nx, N + 1);
        const MX U_sym = opti->variable(ocp.nu, N + 1);
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

        /* Objective & constraints */
        MX lagrange_cost = 0;

        const DM D = dapr_mat(Tau(idx_collocated).nonzeros());
        const DM I = intapr_mat();

        for (int iSeg = 0; iSeg < N_segs; ++iSeg)
        {
            const double seg_t0 = T_segs(iSeg, 0).nonzeros()[0];
            const double seg_tf = T_segs(iSeg, -1).nonzeros()[0];

            const std::vector<double> id_seg_ = id_segs(iSeg, all).nonzeros();
            const std::vector<int> id_seg(id_seg_.begin(), id_seg_.end());
            const MX X_seg = X_sym(all, casadi::Slice(id_seg[0], id_seg[1] + 1));
            const MX U_seg = U_sym(all, casadi::Slice(id_seg[0], id_seg[1] + 1));

            MX lagrange_cost_coll = MX(1, X_seg.size2());
            double t;
            for (int i = 0; i < X_seg.size2(); i++)
            {
                t = ocp.tf * T_segs(iSeg, i).nonzeros()[0];
                lagrange_cost_coll(i) = ocp.lagrange_cost_impl(X_seg(all, i), U_seg(all, i), p, d, t);
            }
            lagrange_cost += (seg_tf - seg_t0) / 2.0 * MX::mtimes(
                    I(I.size1() - 1, all),
                    lagrange_cost_coll(casadi::Slice(0, -1)).T()
            );
            // lagrange_cost_coll = ocp.lagrange_cost_impl( X_seg, U_seg, T_segs(iSeg,all) );

            /* Dynamic constraints */
            MX cont_dynamics_seg = MX(ocp.nx, idx_collocated.size());
            for (const int i: idx_collocated)
            {
                cont_dynamics_seg(all, i) = f_cont(casadi::MXVector({X_seg(all, i), U_seg(all, i), p, d}))[0];

                /* Nonlinear constraints */
                t = ocp.tf * T_segs(iSeg, i).nonzeros()[0];
                MX eq_constr = ocp.eq_constraints_impl(X_sym(all, iSeg), U_sym(all, iSeg), p, d, t);
                if (eq_constr.size1() > 0) { opti->subject_to(eq_constr == 0); }

                MX ineq_constr = ocp.ineq_constraints_impl(X_sym(all, iSeg), U_sym(all, iSeg), p, d, t);
                if (ineq_constr.size1() > 0) { opti->subject_to(ineq_constr <= 0); }
            }
            opti->subject_to(2.0 / (seg_tf - seg_t0) * MX::mtimes(D, X_seg.T()).T()
                             == cont_dynamics_seg);
            // opti->subject_to( 2/(seg_tf-seg_t0) * MX::mtimes(D, X_seg.T() ).T() == ocp.cont_dynamics_impl(X_seg(all,idx_collocated), U_seg(all,idx_collocated)) );
        }
        const MX mayer_cost = ocp.mayer_cost_impl(X_sym(all, N), p, d, ocp.tf);

        MX cost = lagrange_cost + mayer_cost;
        opti->minimize(cost);

        /* */
        //ocp.opti = opti;
        ocp.X_sym = X_sym;
        ocp.U_sym = U_sym;
        ocp.T = T;

        ocp.setup_discretized_trajectory_constraints();
    }

    void resample_trajectory(const DM &Xcol, double dt, DM &Xn, DM &Tn)
    {
        const DM id_segs = conf.id_segs;
        const DM T_segs = conf.T_segs;

        const IM segs_n_ = DM::ceil((T_segs(all, T_segs.size2() - 1) - T_segs(all, 0)) / dt);
        const IM segs_n = IM::vertcat({0, IM::cumsum(segs_n_)});
        const IM segs_idn = IM::horzcat({segs_n(casadi::Slice(0, -1)), segs_n(casadi::Slice(1, segs_n.size1()))});

        const casadi_int n = segs_idn(segs_idn.size1() - 1, segs_idn.size2() - 1).nonzeros()[0];
        Tn = DM::zeros(1, n + 1);
        Xn = DM::zeros(Xcol.size1(), n + 1);

        for (int iSeg = 0; iSeg < T_segs.size1(); ++iSeg)
        {
            const IM id_seg = id_segs(iSeg, all);
            const IM idn_seg = segs_idn(iSeg, all);

            const double t0_seg = T_segs(iSeg, 0).nonzeros()[0];
            const double tf_seg = T_segs(iSeg, T_segs.size2() - 1).nonzeros()[0];

            const DM X_seg = Xcol(all, casadi::Slice(id_seg(0).nonzeros()[0], id_seg(1).nonzeros()[0] + 1));

            std::vector<double> Tn_seg(segs_n_(iSeg).nonzeros()[0] + 1);
            casadi::linspace(Tn_seg, t0_seg, tf_seg);

            const DM Tau_seg = t2tau(Tn_seg, t0_seg, tf_seg);
            DM Xn_seg = DM::zeros(Xcol.size1(), segs_n_(iSeg).nonzeros()[0] + 1);
            for (int i = 0; i < Xn_seg.size2(); i++)
            {
                Xn_seg(all, i) = apr(X_seg, Tau_seg(i).nonzeros()[0]);
            }

            Tn(casadi::Slice(idn_seg(0).nonzeros()[0], idn_seg(1).nonzeros()[0] + 1)) = Tn_seg;
            Xn(all, casadi::Slice(idn_seg(0).nonzeros()[0], idn_seg(1).nonzeros()[0] + 1)) = Xn_seg;
        }

    }

private:
    void setup(double t0, double tf)
    {
        DM Tau_ = Tau;
        if (Tau_(0).nonzeros()[0] >= 0.0)
            Tau_ = 2.0 * Tau_ - 1.0;

        const int D_poly = Tau_.size2() - 1;
        const int N = D_poly * N_segs; // Last index

        /* Setup mesh */
        std::vector<double> t_segs_(N_segs + 1);
        casadi::linspace(t_segs_, t0, tf);
        DM t_segs = DM(t_segs_);
        t_segs = DM::horzcat({t_segs(casadi::Slice(0, -1)), t_segs(casadi::Slice(1, t_segs.size1()))});

        const DM T_segs = tau2t(Tau_, t_segs(all, 0), t_segs(all, 1));

        std::vector<int> id_segs_(N / D_poly + 1);
        casadi::linspace(id_segs_, 0, N);
        IM id_segs(id_segs_);
        id_segs = DM::horzcat({id_segs(casadi::Slice(0, -1)), id_segs(casadi::Slice(1, id_segs.size1()))});

        /* */
        const DM T = DM::horzcat({t0, DM::reshape(T_segs(all, casadi::Slice(1, T_segs.size2())).T(), 1, N)});

        Tau = Tau_;
        conf.D_poly = D_poly;
        conf.N = N;
        conf.T_segs = T_segs;
        conf.T = T;
        conf.id_segs = id_segs;
        std::cout << "Transcription: Radau Collocation. N_segs: " << N_segs << ", Poly order: " << D_poly << ", Tau: "
                  << Tau << "\n";

//        std::cout << "t_segs: " << t_segs << "\n";
//        std::cout << "T_segs: " << T_segs << "\n";
//        std::cout << "id_segs: " << id_segs << "\n";
    }

    // Matrices
    DM dapr_mat(const std::vector<double> &tau_eval)
    {
        DM D = DM::zeros(tau_eval.size(), conf.D_poly + 1);

        for (unsigned i = 0; i < tau_eval.size(); i++)
        {
            for (int j = 0; j < conf.D_poly + 1; j++)
            {
                D(i, j) = dL(j, tau_eval[i]);
            }
        }
        return D;
    }
    DM intapr_mat()
    {
        const DM D = dapr_mat(Tau(casadi::Slice(0, -1)).nonzeros());
        const DM I = DM::inv(D(all, casadi::Slice(1, D.size2())));
        return I;
    }

    // Lagrange interpolation on segment (mesh interval), X: values at mesh points
    template<typename sym_t>
    sym_t apr(const sym_t &X, double tau_eval)
    {
        sym_t x = sym_t::zeros(X.size1(), 1);
        for (int j = 0; j < conf.D_poly + 1; j++)
        {
            x += sym_t::times(X(all, j), L(j, tau_eval));
        }
        return x;
    }

    template<typename sym_t>
    sym_t dapr(const sym_t &X, double tau_eval)
    {
        sym_t x = sym_t::zeros(X.size1(), 1);
        for (int j = 0; j < conf.D_poly + 1; j++)
        {
            x += sym_t::times(X(all, j), dL(j, tau_eval));
        }
        return x;
    }

    // Lagrange polynomials
    double L(int j, double tau_eval) const
    {
        std::vector<int> loop_range;
        for (int i = 0; i < conf.D_poly + 1; i++)
        {
            if (i != j) loop_range.push_back(i);
        }

        double L = 1;
        for (const int l: loop_range)
        {
            L *= static_cast<double>((tau_eval - Tau(l)) / (Tau(j) - Tau(l)));
        }
        return L;
    }
    double dL(int j, double tau_eval) const
    {
        std::vector<int> loop_range;
        for (int i = 0; i < conf.D_poly + 1; i++)
        {
            if (i != j) loop_range.push_back(i);
        }

        double dL = 0;
        for (const int i: loop_range)
        {
            double L = 1;
            for (const int l: loop_range)
            {
                if (l != i)
                    L *= static_cast<double>((tau_eval - Tau(l)) / (Tau(j) - Tau(l)));
            }
            dL += 1.0 / static_cast<double>(Tau(j) - Tau(i)) * L;
        }
        return dL;
    }

    // Time transformation
    static DM tau2t(const DM &tau, const DM &t0, const DM &tf)
    {
        return DM::mtimes((tf - t0) / 2.0, tau) + (tf + t0) / 2.0;
    }
    static DM t2tau(const DM &t, double t0, double tf)
    {
        return 2.0 * (t - t0) / (tf - t0) - 1.0;
    }
};

} // namespace casadi_transcription
} // namespace mpc_control_common

#endif //SRC_RADAUCOLLOCATIONOPTI_HPP
