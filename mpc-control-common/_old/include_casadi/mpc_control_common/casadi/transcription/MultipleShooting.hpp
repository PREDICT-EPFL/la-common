#ifndef SRC_MULTIPLESHOOTING_HPP
#define SRC_MULTIPLESHOOTING_HPP

#include "TranscriptionBase.hpp"

namespace transcription {

class MultipleShooting : public TranscriptionBase
{
public:
    bool use_cos_grid{false};
    void init(const Dict &options)
    {
        /* Parse control objectives */
        parse_dict(options, "N", N);
        parse_dict(options, "use_cos_grid", use_cos_grid);

        init_base();

        std::cout << "Transcription: Multiple Shooting - N=" << N << ", use_cos_grid=" << use_cos_grid << "\n";
    }

    DM get_TU_opt() { return DM::vertcat({T(Slice(0, -1)), sol->value(U_sym)}); }
    DM get_TX_opt() { return DM::vertcat({T, sol->value(X_sym)}); }
    DM get_TU(double dt)
    {
        DM Topt = T(Slice(0, -1));
        DM Uopt = sol->value(U_sym);
        return sample_and_hold(Uopt, Topt, dt);
    }
    DM get_TX(double dt)
    {
        DM Topt = T;
        DM Xopt = sol->value(X_sym);
        return sample_and_hold(Xopt, Topt, dt);
    }
protected:
    void create_problem_vars_impl() override
    {
        /* Generate state trajectory time grid */
        if (use_cos_grid)
        {
            T = tf * (1.0 - cos(pi / 2.0 * DM::linspace(0, 1, N + 1).T()));
        }
        else
        {
            T = tf * DM::linspace(0, 1, N + 1).T();
        }

        /* Decision variables */
        X_sym = opti->variable(nx, N + 1); // State trajectory variables (initial state + N points)
        U_sym = opti->variable(nu, N);     // Control trajectory variables
    }
    void setup_cost_impl() override
    {
        sym_t cost = 0;
        DM hs = DM::diff(T);  // Step durations
        double h_mean = sum(hs.nonzeros()) / hs.numel();
        for (int k = 0; k < N; ++k)
        {
            // Stage cost (Lagrange term)
            cost += hs(k) / h_mean * lagrange_cost_impl(X_sym(Slice(), k), U_sym(Slice(), k), sym_t(), sym_t());
        }
        // Terminal cost (Meyer term)
        cost += mayer_cost_impl(X_sym(Slice(), N), sym_t(), sym_t());

        opti->minimize(cost);
    }
    void setup_dyn_constraints_impl() override
    {
        DM hs = DM::diff(T);
        for (int k = 0; k < N; ++k)
        {
            opti->subject_to(
                    X_sym(Slice(), k + 1) == f_discr(X_sym(Slice(), k), U_sym(Slice(), k), sym_t(), sym_t(), hs(k))
            );
        }
    }

private:
    sym_t RK2a(const sym_t &x, const sym_t &u, const sym_t &p, const sym_t &d, const sym_t &h, const Function &f)
    {
        /* Improved Euler's method */
        sym_t k1 = cont_dynamics_impl(x, u, p, d);
        sym_t k2 = cont_dynamics_impl(x + h / 2 * k1, u, p, d);
        sym_t x_next = x + h * k2;
        return x_next;
    }
    sym_t RK2b(const sym_t &x, const sym_t &u, const sym_t &p, const sym_t &d, const sym_t &h)
    {
        /* Heun's method */
        sym_t k1 = cont_dynamics_impl(x, u, p, d);
        sym_t k2 = cont_dynamics_impl(x + h * k1, u, p, d);
        sym_t x_next = x + h / 2 * (k1 + k2);
        return x_next;
    }
    sym_t RK4(const sym_t &x, const sym_t &u, const sym_t &p, const sym_t &d, const sym_t &h)
    {
        sym_t k1 = cont_dynamics_impl(x, u, p, d);
        sym_t k2 = cont_dynamics_impl(x + h / 2 * k1, u, p, d);
        sym_t k3 = cont_dynamics_impl(x + h / 2 * k2, u, p, d);
        sym_t k4 = cont_dynamics_impl(x + h * k3, u, p, d);
        sym_t x_next = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
        return x_next;
    }

    sym_t f_discr(const sym_t &x, const sym_t &u, const sym_t &p, const sym_t &d, const sym_t &h)
    {
//        sym_t x_next = RK2a(x, u, p, d, h);
//        sym_t x_next = RK2b(x, u, p, d, h);
        sym_t x_next = RK4(x, u, p, d, h);
        return x_next;
    }
};

}

#endif //SRC_MULTIPLESHOOTING_HPP
