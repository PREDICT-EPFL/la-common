#ifndef SRC__DELAYCOMPENSATION_HPP
#define SRC__DELAYCOMPENSATION_HPP

#include "mpc_control_racing/RacingOcpParams.hpp"

namespace mpc_control_racing {

class DelayCompensator
{
protected:
    virtual Eigen::VectorXd cont_dynamics_impl(const Eigen::VectorXd &x, const Eigen::VectorXd &u)
    {
        (void) u;
        Eigen::VectorXd x_dot = x;
        x_dot.setZero();
        return x_dot;
    }
    virtual Eigen::VectorXd discr_dynamics_impl(const Eigen::VectorXd &x, const Eigen::VectorXd &u)
    {
        (void) u;
        return x;
    }

public:
    enum DynamicsMode
    {
        UNDEFINED, CONTINUOUS, DISCRETE
    };

    explicit DelayCompensator(const DynamicsMode &dynamicsMode, double dt = 0.01) :
            m_dynamics_mode(dynamicsMode),
            m_dt(dt)
    {
        std::cout << "DelayCompensator. Dynamics mode: " << m_dynamics_mode << ", dt: " << dt << "\n";
    }

    void set_delay(double delay) { m_delay = delay; }
    virtual Eigen::VectorXd compose_delayed_state(const RacingOcpParams::RacingState &state,
                                                  const RacingOcpParams::StateIndex &state_index,
                                                  const Eigen::VectorXd &x_0)
    {
        (void) state;
        (void) state_index;
        (void) x_0;
        return {};
    }
    Eigen::VectorXd predict_state(const Eigen::VectorXd &x0, const Eigen::VectorXd &u, double t0 = 0, double tf = 0)
    {
        if (tf <= t0) { tf = t0 + m_delay; }
//        std::cout << "Compensate " << (tf - t0) * 1e3 << "ms delay from x0: " << x0.transpose() << " with u: " << u.transpose() << "\n";
//        const std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

        Eigen::VectorXd x_pred = x0;
        switch (m_dynamics_mode)
        {
            case CONTINUOUS:
            {
                x_pred = simulate_continuous(x0, u, t0, tf);
                break;
            }
            case DISCRETE:
            {
                x_pred = simulate_discrete(x0, u, t0, tf);
                break;
            }
            default:
            {
                std::cerr << "Undefined dynamics mode\n";
            }
        }

//        const std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
//        long comp_time_us = std::chrono::duration_cast<std::chrono::microseconds >(toc - tic).count();
//        std::cout << " (" <<  comp_time_us / 1e3 << "ms)\n";

        return x_pred;
    }
    virtual Eigen::VectorXd get_ocp_state(const Eigen::VectorXd &x_pred,
                                          const RacingOcpParams::StateIndex &state_index,
                                          const Eigen::VectorXd &x_0)
    {
        (void) x_pred;
        (void) state_index;
        (void) x_0;
        return {};
    }
protected:
    DynamicsMode m_dynamics_mode{UNDEFINED};
    const double m_dt{0.01};
    double m_delay{0.02};

    Eigen::VectorXd simulate_continuous(const Eigen::VectorXd &x0, const Eigen::VectorXd &u, double t0, double tf)
    {
        const double sim_duration = tf - t0;
        const unsigned N = std::ceil(sim_duration / m_dt);

        Eigen::VectorXd x_sim = x0;
        for (unsigned k = 0; k < N; ++k)
        {
            x_sim += cont_dynamics_impl(x_sim, u) * m_dt;
        }
        return x_sim;
    }
    Eigen::VectorXd simulate_discrete(const Eigen::VectorXd &x0, const Eigen::VectorXd &u, double t0, double tf)
    {
        const double sim_duration = tf - t0;
        const unsigned N = std::ceil(sim_duration / m_dt);

        Eigen::MatrixXd X_sim(x0.rows(), N);
        X_sim.setZero();
        X_sim.col(0) = x0;

        Eigen::VectorXd x_sim = x0;
        for (unsigned k = 0; k < N; ++k)
        {
            x_sim = discr_dynamics_impl(x_sim, u);
            X_sim.col(k) = x_sim;
        }
//        std::cout << "X_sim: " << X_sim << "\n";
        return x_sim;
    }
};

}

#endif //SRC__DELAYCOMPENSATION_HPP
