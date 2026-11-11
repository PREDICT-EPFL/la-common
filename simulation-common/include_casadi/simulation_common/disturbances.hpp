#ifndef DISTURBANCES_HPP
#define DISTURBANCES_HPP

/*
 * Disturbances (namespace disturbance): turbulence (DrydenTurbulence, KinematicTurbulence) and white-noise velocity kicks (generate_vel_kicks).
 */

#include <cmath>
#include <random>
#include <casadi/casadi.hpp>

namespace disturbance
{
    using namespace casadi;

    class DrydenTurbulence
    {
        /*
         * Dryden turbulence (MIL-F-8785C, continuous form; the same turbulence as MIL-HDBK-1797, which has L_w = h / 2, L_v = L_u / 2 and 2 L in the filters) as a shaping filter driven by white noise (intensity pi, see step), exactly discretized (ZOH of the
         * white noise input over the step, matrix exponential in closed form). Output: gust (u, v, w) in the frame of the caller (m/s).
         *   H_u = sigma_u sqrt(2 L_u / (pi V)) / (1 + (L_u / V) s)
         *   H_v, H_w = sigma sqrt(L / (pi V)) (1 + sqrt(3) (L / V) s) / (1 + (L / V) s)^2
         * Low altitude model at the height h = height, W20 = speed_at_20ft (the standard formulas take h in ft; here the lengths are converted to m):
         *   L_w = h, L_u = L_v = h / (0.177 + 0.000823 h)^1.2, sigma_w = 0.1 W20, sigma_u = sigma_v = sigma_w / (0.177 + 0.000823 h)^0.4
         * V is the speed at which the vehicle sweeps through the frozen turbulence field (Taylor hypothesis).
         */
    public:
        double sigma_u_, sigma_v_, sigma_w_; // Turbulence intensities (m/s)
        double L_u_, L_v_, L_w_;             // Turbulence scale lengths (m)
        double V_;                           // Airspeed through the turbulence (m/s)

        /* speed_at_20ft: wind speed at 20 ft (m/s) (W20 above), height: altitude (m) (h above), V: airspeed (m/s) */
        DrydenTurbulence(const double speed_at_20ft, const double height, const double V) : V_(V)
        {
            const double ft = 0.3048;     // m per ft
            const double height_ft = height / ft; // Altitude (ft)
            const double f = 0.177 + 0.000823 * height_ft;
            L_w_ = height;
            L_u_ = L_v_ = height / std::pow(f, 1.2);
            sigma_w_ = 0.1 * speed_at_20ft;
            sigma_u_ = sigma_v_ = sigma_w_ / std::pow(f, 0.4);
            x_u_ = 0;
            x_v_ = DM::zeros(2, 1);
            x_w_ = DM::zeros(2, 1);
        }

        DM step(const double dt, std::mt19937 &rng)
        {
            std::normal_distribution<double> n01(0.0, 1.0);
            /*
             * White noise held over the step (ZOH): E[n(t) n(t')] = pi delta(t - t'), i.e. sqrt(pi) N(0,1) / sqrt(dt). The factor pi is the normalization
             * of the one-sided PSD of the Dryden spectra (the filters above have the output variance sigma^2 / pi for the noise of unit intensity)
             */
            const double sqrt_dt = std::sqrt(dt);
            const double n_u = std::sqrt(M_PI) * n01(rng) / sqrt_dt;
            const double n_v = std::sqrt(M_PI) * n01(rng) / sqrt_dt;
            const double n_w = std::sqrt(M_PI) * n01(rng) / sqrt_dt;

            /* u: x' = -a x + a K n, y = x (first order), a = V / L_u, K = sigma_u sqrt(2 L_u / (pi V)) */
            const double a_u = V_ / L_u_;
            const double K_u = sigma_u_ * std::sqrt(2 * L_u_ / (M_PI * V_));
            const double phi_u = std::exp(-a_u * dt);
            x_u_ = phi_u * x_u_ + K_u * (1 - phi_u) * n_u;

            /* v, w: x1' = x2, x2' = -a^2 x1 - 2 a x2 + n, y = K a^2 (x1 + sqrt(3) x2 / a) (second order, double pole -a), a = V / L, K = sigma sqrt(L / (pi V)) */
            const double a_v = V_ / L_v_;
            const double K_v = sigma_v_ * std::sqrt(L_v_ / (M_PI * V_));
            const double a_w = V_ / L_w_;
            const double K_w = sigma_w_ * std::sqrt(L_w_ / (M_PI * V_));
            step_second_order(x_v_, a_v, n_v, dt);
            step_second_order(x_w_, a_w, n_w, dt);

            return DM::vertcat({DM(x_u_), get_output_second_order(x_v_, a_v, K_v), get_output_second_order(x_w_, a_w, K_w)});
        }

    private:
        double x_u_;
        DM x_v_, x_w_;

        static void step_second_order(DM &x, const double a, const double n, const double dt)
        {
            /* A = [0, 1; -a^2, -2a] = -a I + N, N = [a, 1; -a^2, -a], N^2 = 0: Phi = exp(-a dt) (I + N dt), Gamma = A^-1 (Phi - I) B, B = [0; 1] */
            const DM A = DM::blockcat({{DM(0), DM(1)}, {DM(-a * a), DM(-2 * a)}});
            const DM N = DM::blockcat({{DM(a), DM(1)}, {DM(-a * a), DM(-a)}});
            const DM Phi = std::exp(-a * dt) * (DM::eye(2) + N * dt);
            const DM Gamma = mtimes(inv(A), mtimes(Phi - DM::eye(2), DM({0.0, 1.0})));
            x = mtimes(Phi, x) + Gamma * n;
        }
        static DM get_output_second_order(const DM &x, const double a, const double K)
        {
            return K * a * a * (x(0) + std::sqrt(3.0) / a * x(1));
        }
    };

    class KinematicTurbulence
    {
        /*
         * Atmospheric turbulence as a frozen random wind field that can be evaluated at any position and time:
         *
         *     n_dv_turb (m/s) = get_gust(position (NED) (m), time (s))   (wind velocity offset due to the turbulence, NED)
         *
         * Why a field and not a filter (DrydenTurbulence)? Each wing asks for the gust at its own position, so the two wings (and the hub) see
         * different but correlated gusts: close positions get similar gusts, far positions get independent ones. The vehicle moves through the field.
         *
         * The method is the "kinematic simulation" of Fung et al. (1992). In short:
         *   1. Strength and size of the turbulence: the intensities sigma (m/s) and the scale lengths L (m) follow from the wind speed at 20 ft (speed_at_20ft)
         *      and the altitude (height), using the low-altitude model of MIL-F-8785C (same formulas as DrydenTurbulence).
         *   2. The field is a sum of N_modes plane waves (random Fourier modes). Wave number k_n = 2 pi / wavelength: small k = large slow eddies,
         *      large k = small eddies. The k_n are log-spaced from large eddies (several times L_hor) down to small ones (dx_min).
         *   3. Each mode n gets a weight w_n (how much energy it carries). The weights follow the von Karman energy spectrum E(k):
         *      little energy in the largest eddies, a peak near k_e = 0.7468 / L_hor, and a decay ~ k^(-5/3) (Kolmogorov) for the small eddies.
         *   4. Each mode has a random direction khat_n (unit vector) and two random amplitude vectors a_n, b_n (perpendicular to khat_n, which keeps
         *      the unscaled field divergence-free: air is incompressible). Mode n contributes a_n cos(phase_n) + b_n sin(phase_n) to the wind.
         *   5. The phase of a mode is  phase_n = k_n khat_n . (x - U t) + omega_n t:
         *      - x - U t: the whole pattern is carried along by the mean wind U (frozen turbulence, Taylor hypothesis). For U = 0 the pattern stands
         *        still and the vehicle only sees it change because it moves through it.
         *      - omega_n t: slow evolution of the eddies themselves (small eddies change faster, omega_n = lambda sqrt(k_n^3 E_dim(k_n))).
         *   6. The sum has unit variance per component by construction (sum of w_n = 3 / 2); the diagonal matrix S = diag(sigma_hor, sigma_hor, sigma_ver)
         *      scales it to the intensities. S makes the field anisotropic (sigma_ver < sigma_hor), which slightly breaks the exact
         *      divergence-freeness; acceptable for a simulation.
         *
         * Deviations from Dryden: von Karman spectrum instead of the Dryden filter spectra, and one common scale length L_hor for the spectrum shape
         * (the MIL-F-8785C vertical scale length L_w = height is not used). The altitude is fixed at construction (the field does not change when the vehicle climbs).
         * The random numbers are drawn only in the constructor, so the field is deterministic afterwards (same rng seed = same field).
         * Frames: NED, the field is aligned with the NED axes: north and east are the horizontal and down is the vertical component.
         */
    public:
        double sigma_hor_, sigma_ver_; // Turbulence intensities of the horizontal (north, east) and the vertical (down) component (m/s); MIL-F-8785C: sigma_u = sigma_v and sigma_w
        double L_hor_;                 // Turbulence scale length (m), used for the whole field; MIL-F-8785C: L_u = L_v

        /*
         * speed_at_20ft: wind speed at 20 ft (m/s); the intensity knob (light turbulence: 15 kt = 7.7 m/s)
         * height: altitude (m); n_U_mean: mean wind (NED) (m/s), advects the field
         * N_modes: number of Fourier modes (more = smoother spectrum, slower get_gust); dx_min: smallest eddy size resolved (m)
         */
        KinematicTurbulence(const double speed_at_20ft, const double height, const DM &n_U_mean, std::mt19937 &rng, const int N_modes = 200, const double dx_min = 1.0)
            : n_U_mean_(n_U_mean), k_(DM::zeros(3, N_modes)), a_(DM::zeros(3, N_modes)), b_(DM::zeros(3, N_modes)), omega_(DM::zeros(N_modes, 1))
        {
            /* Step 1: intensities and scale lengths */
            set_intensities_and_scale_lengths(speed_at_20ft, height);
            S_ = DM::diag(DM({sigma_hor_, sigma_hor_, sigma_ver_}));

            /*
             * Step 2: wave number grid, log-spaced: k_n = k_min r^n, n = 0..N_modes-1, so that the last one is k_max.
             * k_min = 0.05 k_e (eddies 20x larger than the energy peak, they carry almost nothing), k_max = 2 pi / dx_min (wavelength dx_min)
             */
            const double k_e = 0.7468 / L_hor_; // Wave number of the von Karman energy peak (1/m)
            const double k_min = 0.05 * k_e;
            const double k_max = 2 * M_PI / dx_min;
            const double r = std::pow(k_max / k_min, 1.0 / (N_modes - 1)); // Ratio of consecutive wave numbers (-)

            auto get_k = [&](const int n) { return k_min * std::pow(r, n); };
            auto get_dk = [&](const int n) { return get_k(n) * (std::sqrt(r) - 1 / std::sqrt(r)); }; // Width of the log-spaced cell around k_n (1/m)

            /* Step 3: mode weights w_n ~ E(k_n) dk_n (energy in the cell of mode n), normalized below to sum(w_n) = 3 / 2 */
            auto get_unnormalized_weight = [&](const int n) { return get_von_karman_spectrum(get_k(n) / k_e) * get_dk(n); };
            double weight_sum = 0;
            for (int n = 0; n < N_modes; ++n) { weight_sum += get_unnormalized_weight(n); }

            /* Steps 4 and 5: random direction, amplitudes and frequency of each mode */
            const double lambda = 0.5; // Eddy turnover rate factor (-)
            std::normal_distribution<double> n01(0.0, 1.0);
            for (int n = 0; n < N_modes; ++n)
            {
                const double w = get_unnormalized_weight(n) * 1.5 / weight_sum; // Mode weight, sum(w_n) = 3 / 2 (total energy of unit variance in 3 components)
                const DM khat = get_random_unit_vector(rng, n01);               // Propagation direction of the mode (-)
                k_(Slice(), n) = get_k(n) * khat;
                /* |a_n|^2 = |b_n|^2 = 2 w_n: cos^2 and sin^2 average to 1 / 2, so mode n adds the variance (|a_n|^2 + |b_n|^2) / 2 = 2 w_n and all modes together 3 (1 per component) */
                a_(Slice(), n) = std::sqrt(2 * w) * get_random_perpendicular_unit_vector(khat, rng, n01);
                b_(Slice(), n) = std::sqrt(2 * w) * get_random_perpendicular_unit_vector(khat, rng, n01);

                const double E_dim = sigma_hor_ * sigma_hor_ * w / get_dk(n); // Dimensional spectrum value E(k_n) ((m/s)^2 / (1/m)); only sigma_hor is used for the time scales
                omega_(n) = lambda * std::sqrt(std::pow(get_k(n), 3) * E_dim);
            }
        }

        /* Wind velocity offset n_dv_turb (NED) (m/s) due to the turbulence at the position n_pos (NED) (m) and time t (s) */
        DM get_gust(const DM &n_pos, const double t) const
        {
            /* Phase of each mode (N_modes x 1): k_n . (x - U t) + omega_n t */
            const DM phase = mtimes(k_.T(), n_pos - n_U_mean_ * t) + omega_ * t;
            /* Sum of all modes (3 x 1, unit variance per component), then scaled to the intensities */
            return mtimes(S_, mtimes(a_, cos(phase)) + mtimes(b_, sin(phase)));
        }

    private:
        DM n_U_mean_; // Mean wind that advects the field (NED) (m/s)
        DM S_;        // Intensity scaling diag(sigma_hor, sigma_hor, sigma_ver) (m/s), 3 x 3
        DM k_;        // Wave vectors k_n khat_n (1/m), 3 x N_modes
        DM a_, b_;    // Mode amplitudes of the cos and sin part (-), 3 x N_modes
        DM omega_;    // Mode frequencies (rad/s), N_modes x 1

        /* Low-altitude model of MIL-F-8785C (the standard formulas take the altitude in ft; the lengths are scale-free, so they come out in m for an altitude in m) */
        void set_intensities_and_scale_lengths(const double speed_at_20ft, const double height)
        {
            const double height_ft = height / 0.3048; // Altitude (ft)
            const double f = 0.177 + 0.000823 * height_ft;
            L_hor_ = height / std::pow(f, 1.2);               // L_u = L_v (L_w = height is not needed)
            sigma_ver_ = 0.1 * speed_at_20ft;                 // sigma_w
            sigma_hor_ = sigma_ver_ / std::pow(f, 0.4);       // sigma_u = sigma_v
        }

        /* Von Karman energy spectrum shape (not normalized) E ~ kappa^4 / (1 + kappa^2)^(17/6) with kappa = k / k_e (-): ~ k^4 for large eddies, ~ k^(-5/3) for small eddies */
        static double get_von_karman_spectrum(const double kappa)
        {
            return std::pow(kappa, 4) / std::pow(1 + kappa * kappa, 17.0 / 6.0);
        }

        /* Uniformly distributed direction: normalized Gaussian vector (-) */
        static DM get_random_unit_vector(std::mt19937 &rng, std::normal_distribution<double> &n01)
        {
            const DM g = DM({n01(rng), n01(rng), n01(rng)});
            return g / DM::norm_2(g);
        }

        /* Random unit vector perpendicular to khat (-): remove the khat component of a random vector (Gram-Schmidt) and normalize */
        static DM get_random_perpendicular_unit_vector(const DM &khat, std::mt19937 &rng, std::normal_distribution<double> &n01)
        {
            const DM g = get_random_unit_vector(rng, n01);
            const DM p = g - DM::dot(g, khat) * khat;
            return p / DM::norm_2(p);
        }
    };

    /* White-noise velocity kicks (vel, omg) of one body for the steps 0..N: 6 x (N + 1), sigma_v ((m/s)/sqrt(s)), sigma_w ((rad/s)/sqrt(s)) */
    static DM generate_vel_kicks(int N, double dt, double sigma_v, double sigma_w, unsigned seed)
    {
        /*
         * dv = sigma_v sqrt(dt) N3(0,1) (m/s), domega = sigma_w sqrt(dt) N3(0,1) (rad/s), body frame FRD; zeros if both sigmas are 0.
         * Rows: [vel (3), omg (3)], added to the body velocities [v_b; omega_b] of body n; use a different seed per body.
         * The kicks slightly violate c_dot = 0; the Baumgarte stabilization corrects it.
         */
        DM kicks = DM::zeros(6, N + 1); // Rows: vel (3), omg (3)
        if (!(sigma_v > 0 || sigma_w > 0)) { return kicks; }

        std::mt19937 rng(seed);
        std::normal_distribution<double> n01(0.0, 1.0);
        const double sqrt_dt = std::sqrt(dt);
        for (int k = 0; k <= N; ++k)
        {
            for (int i = 0; i < 3; ++i) { kicks(i, k) = sigma_v * sqrt_dt * n01(rng); }     // Velocity
            for (int i = 0; i < 3; ++i) { kicks(3 + i, k) = sigma_w * sqrt_dt * n01(rng); } // Angular velocity
        }
        return kicks;
    }
}

#endif // DISTURBANCES_HPP
