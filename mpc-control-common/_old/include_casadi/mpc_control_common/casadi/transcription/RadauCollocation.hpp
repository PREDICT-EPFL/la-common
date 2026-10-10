#ifndef SRC_RADAUCOLLOCATION_HPP
#define SRC_RADAUCOLLOCATION_HPP

#include <memory>

#include "mpc_control_common/casadi/OptiOcpBase.hpp"
#include "TranscriptionBase.hpp"

namespace transcription {

using namespace ocp_casadi;

class RadauCollocation
{
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

    explicit RadauCollocation(int N_segs = -1, int D_poly = -1)
    {
        if (N_segs > 0)
            this->N_segs = N_segs;

        if (D_poly > 0)
        {
            DM points = 1 - DM(casadi::collocation_points(D_poly, "radau"));
            this->Tau = DM::vertcat({points(Slice(-1, 0, -1), Slice()), points(0, Slice()), 1}).T();
            std::cout << "Radau Tau: " << this->Tau << "\n";
        }
    }

    void test(double tf)
    {
        setup(0, tf);

        std::cout << "D: " << dapr_mat(Tau(Slice(0, -1)).nonzeros()) << "\n";
        std::cout << "I: " << intapr_mat() << "\n";
    }

    template<class OCP>
    void transcribe(OCP &ocp)
    {
        setup(0, ocp.tf);

        const int N = conf.N;
        std::vector<int> idx_collocated(conf.D_poly);
        casadi::linspace(idx_collocated, 0, conf.D_poly - 1);
        const DM id_segs = conf.id_segs;
        const DM T_segs = conf.T_segs;
        const DM T = conf.T;

        const std::shared_ptr<casadi::Opti> opti = ocp.opti;

        /* Create variables and indexing */
        const sym_t X_sym = opti->variable(ocp.nx, N);
        const sym_t U_sym = opti->variable(ocp.nu, N);
        const sym_t p = sym_t();
        const sym_t d = sym_t();

        /* Objective & constraints */
        sym_t lagrange_cost = 0;

        const DM D = dapr_mat(Tau(idx_collocated).nonzeros());
        const DM I = intapr_mat();

        for (int iSeg = 0; iSeg < N_segs; ++iSeg)
        {
            const double seg_t0 = T_segs(iSeg, 0).nonzeros()[0];
            const double seg_tf = T_segs(iSeg, -1).nonzeros()[0];

            const std::vector<double> id_seg_ = id_segs(iSeg, Slice()).nonzeros();
            const std::vector<int> id_seg(id_seg_.begin(), id_seg_.end());
            const sym_t X_seg = X_sym(Slice(), Slice(id_seg[0], id_seg[1] + 1));
            const sym_t U_seg = U_sym(Slice(), Slice(id_seg[0], id_seg[1] + 1));

            sym_t lagrange_cost_coll = sym_t(1, X_seg.size2());
            double t;
            for (int i = 0; i < X_seg.size2(); i++)
            {
                t = T_segs(iSeg, i).nonzeros()[0];
                lagrange_cost_coll(i) = ocp.lagrange_cost_impl(X_seg(Slice(), i), U_seg(Slice(), i), p, d);
            }
            lagrange_cost += (seg_tf - seg_t0) / 2.0 * sym_t::mtimes(
                    I(I.size1() - 1, Slice()),
                    lagrange_cost_coll(Slice(0, -1)).T()
            );
            // lagrange_cost_coll = ocp.lagrange_cost_impl( X_seg, U_seg, T_segs(iSeg,Slice()) );

            /* Dynamic constraints */
            sym_t cont_dynamics_seg = sym_t(ocp.nx, idx_collocated.size());
            for (const int i: idx_collocated)
            {
                cont_dynamics_seg(Slice(), i) = ocp.cont_dynamics_impl(X_seg(Slice(), i), U_seg(Slice(), i), p, d);
            }
            opti->subject_to(2.0 / (seg_tf - seg_t0) * sym_t::mtimes(D, X_seg.T()).T()
                             == cont_dynamics_seg);
            // opti->subject_to( 2/(seg_tf-seg_t0) * sym_t::mtimes(D, X_seg.T() ).T() == ocp.cont_dynamics_impl(X_seg(Slice(),idx_collocated), U_seg(Slice(),idx_collocated)) );
        }
        const sym_t mayer_cost = ocp.mayer_cost_impl(X_sym(Slice(), N - 1), p, d);
        // opti->subject_to( dapr(X_seg, 1) == ocp.cont_dynamics_impl(X_seg(Slice(),-1), U_seg(Slice(),-1)) );

        sym_t cost = lagrange_cost + mayer_cost;
        opti->minimize(cost);

        /* */
        ocp.opti = opti;
        ocp.X_sym = X_sym;
        ocp.U_sym = U_sym;
        ocp.T = T;

        ocp.setup_discretized_trajectory_constraints();
    }

    void resample_trajectory(const DM &Xcol, double dt, DM &Xn, DM &Tn)
    {
        const DM id_segs = conf.id_segs;
        const DM T_segs = conf.T_segs;

        const IM segs_n_ = DM::ceil((T_segs(Slice(), T_segs.size2() - 1) - T_segs(Slice(), 0)) / dt);
        const IM segs_n = IM::vertcat({0, IM::cumsum(segs_n_)});
        const IM segs_idn = IM::horzcat({segs_n(Slice(0, -1)), segs_n(Slice(1, segs_n.size1()))});

        const casadi_int n = segs_idn(segs_idn.size1() - 1, segs_idn.size2() - 1).nonzeros()[0];
        Tn = DM::zeros(1, n + 1);
        Xn = DM::zeros(Xcol.size1(), n + 1);

        for (int iSeg = 0; iSeg < T_segs.size1(); ++iSeg)
        {
            const IM id_seg = id_segs(iSeg, Slice());
            const IM idn_seg = segs_idn(iSeg, Slice());

            const double t0_seg = T_segs(iSeg, 0).nonzeros()[0];
            const double tf_seg = T_segs(iSeg, T_segs.size2() - 1).nonzeros()[0];

            const DM X_seg = Xcol(Slice(), Slice(id_seg(0).nonzeros()[0], id_seg(1).nonzeros()[0] + 1));

            std::vector<double> Tn_seg(segs_n_(iSeg).nonzeros()[0] + 1);
            casadi::linspace(Tn_seg, t0_seg, tf_seg);

            const DM Tau_seg = t2tau(Tn_seg, t0_seg, tf_seg);
            DM Xn_seg = DM::zeros(Xcol.size1(), segs_n_(iSeg).nonzeros()[0] + 1);
            for (int i = 0; i < Xn_seg.size2(); i++)
            {
                Xn_seg(Slice(), i) = apr(X_seg, Tau_seg(i).nonzeros()[0]);
            }

            Tn(Slice(idn_seg(0).nonzeros()[0], idn_seg(1).nonzeros()[0] + 1)) = Tn_seg;
            Xn(Slice(), Slice(idn_seg(0).nonzeros()[0], idn_seg(1).nonzeros()[0] + 1)) = Xn_seg;
        }

    }

private:
    void setup(double t0, double tf)
    {
        DM Tau_ = Tau;
        if (Tau_(0).nonzeros()[0] >= 0.0)
            Tau_ = 2.0 * Tau_ - 1.0;

        const int D_poly = Tau_.size2() - 1;
        const int N = D_poly * N_segs + 1;

        /* Setup mesh */
        std::vector<double> t_segs_(N_segs + 1);
        casadi::linspace(t_segs_, t0, tf);
        DM t_segs = DM(t_segs_);
        t_segs = DM::horzcat({t_segs(Slice(0, -1)), t_segs(Slice(1, t_segs.size1()))});

        const DM T_segs = tau2t(Tau_, t_segs(Slice(), 0), t_segs(Slice(), 1));

        std::vector<int> id_segs_((N - 1) / D_poly + 1);
        casadi::linspace(id_segs_, 0, N - 1);
        IM id_segs(id_segs_);
        id_segs = DM::horzcat({id_segs(Slice(0, -1)), id_segs(Slice(1, id_segs.size1()))});

        /* */
        const DM T = DM::horzcat({t0, DM::reshape(T_segs(Slice(), Slice(1, T_segs.size2())).T(), 1, N - 1)});

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

        for (int i = 0; i < tau_eval.size(); i++)
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
        const DM D = dapr_mat(Tau(Slice(0, -1)).nonzeros());
        const DM I = DM::inv(D(Slice(), Slice(1, D.size2())));
        return I;
    }

    // Lagrange interpolation on segment (mesh interval), X: values at mesh points
    template<typename Derived>
    Derived apr(const Derived &X, double tau_eval)
    {
        Derived x = Derived::zeros(X.size1(), 1);
        for (int j = 0; j < conf.D_poly + 1; j++)
        {
            x += Derived::times(X(Slice(), j), L(j, tau_eval));
        }
        return x;
    }
    sym_t dapr(const sym_t &X, double tau_eval)
    {
        sym_t x = sym_t::zeros(X.size1(), 1);
        for (int j = 0; j < conf.D_poly + 1; j++)
        {
            x += sym_t::times(X(Slice(), j), dL(j, tau_eval));
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


}

#endif //SRC_RADAUCOLLOCATION_HPP
