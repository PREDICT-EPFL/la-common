#ifndef SRC_MPCBASE_HPP
#define SRC_MPCBASE_HPP

#include <chrono>
#include <Eigen/Core>

#include "mpc_control_common/MpcUtils.hpp"

#define DEBUG_DELAYCOMP false

namespace mpc_control {

class MpcBase
{
public:
    ~MpcBase()
    {
        durationLogger.printStatistics();
    }
    virtual void init() { mpc_utils::print_base("init"); }
    virtual void diagnose() { mpc_utils::print_base("diagnose"); }

    void set_state(const Eigen::VectorXd &state)
    {
        m_state_c = state;
        m_state_usedCounter = 0;
    }
    void run_once()
    {
        Eigen::VectorXd initial_state = m_state_c;
        predict_initial_state(initial_state); // Delay compensation
        compute_control(initial_state);
        if (evaluate_solution())
        {
            store_solution_impl();
        }
    }
    void run()
    {
        std::cout << std::fixed << std::setprecision(1);
        while (ok())
        {
            std::cout << "\n----------------------------------------------------------------------------\n";
            init();

            std::cout << "[PolyMPC Control]: Running at " << p_rate << " Hz ...\n";
            while (ok() && !m_re_init)
            {
                update_in();
                /* Check whether available state should be used for an optimization */
                if (m_state_usedCounter <= p_reuse_state_max)
                {
                    /* If it is used several times, correct its capture time, so that the optimization result based on
                     * this state won't be in the past when applied to the plant */
                    if (m_state_usedCounter > 0)
                    {
                        std::cout << "\nReusing state: " << m_state_usedCounter + 1 << " time(s)\n";
                        m_tc_state += (1.0 / p_rate);
                    }
                    run_once();
                    update_out();
                    ++m_state_usedCounter;
                }
                sleep();        // Sleep remaining time to match rate
            }
            if (m_re_init) std::cout << "Re-init MPC.\n";
            m_re_init = false;
        }
    }
    void test(const Eigen::VectorXd &x0)
    {
        std::cout << "Test...\n";

        init();

        /* IN */
        set_state(x0);

        run_once();

        /* OUT */
        std::cout << "State trajectory: " << get_state_trajectory() << "\n"
                  << "Control Trajectory: " << get_control_trajectory() << "\n";
    }

    virtual int get_solve_status() const { return mpc_utils::print_base<int>("get_solve_status"); }
    virtual bool solve_status_is_valid() const { return mpc_utils::print_base("solve_status_is_valid"); }
    virtual bool control_contains_nans() const { return mpc_utils::print_base("control_contains_nans"); }
    virtual const Eigen::VectorXd &get_control() const = 0;
    virtual const Eigen::MatrixXd &get_control_trajectory() const = 0;
    virtual const Eigen::MatrixXd &get_state_trajectory() const = 0;
    virtual double get_cost() const { return mpc_utils::print_base<double>("get_cost"); }

protected:
    std::string p_model_params_dir{getenv("HOME") + std::string("/")};
//    std::string get_model_params_path() const
//    {
//        /* Read in kiteparams name from kitefile and construct kiteparams path */
//        const std::string kitefile_path = getenv("HOME") + std::string("/mpc_kitefile.txt");
//        std::string kiteparams_name;
//        std::ifstream kite_params_file(kitefile_path, std::ios::in);
//        kite_params_file >> kiteparams_name;
//        std::string kiteparams_path = p_model_params_dir + kiteparams_name + ".yaml";
//        return kiteparams_path;
//    }

    virtual void update_in() { std::cout << "MpcBase::update_in()\n"; }
    virtual void update_out()
    {
        std::cout << "State trajectory: " << get_state_trajectory() << "\n"
                  << "Control Trajectory: " << get_control_trajectory() << "\n";
    }
    virtual void compute_control_impl(const Eigen::VectorXd &x0) { mpc_utils::print_base("compute_control_impl"); }
    virtual void store_solution_impl() { mpc_utils::print_base("store_solution_impl"); }
    virtual void predict_initial_state(const Eigen::VectorXd &x0)
    {
        if (p_delay_capt_apply == 0) return; // If there is no delay, don't do anything

        const double ta = m_tc_state + p_delay_capt_apply;
        predict_initial_state_impl(x0, m_tc_state,
                                   ta); // Application specific implementation for computing virtual states
    }
    virtual void predict_initial_state_impl(const Eigen::VectorXd &x0, double tc, double ta)
    {
        std::cout << "predict_initial_state_impl()\n";
    }

    void compute_control(const Eigen::VectorXd &x0)
    {
        using namespace std::chrono;
        const steady_clock::time_point start = steady_clock::now();
        compute_control_impl(x0);
        const steady_clock::time_point stop = steady_clock::now();
        const auto duration = duration_cast<microseconds>(stop - start);
        m_comp_time_us = duration.count() + simulate_computation_delay(duration.count());
        durationLogger.log(m_comp_time_us);
    }
    bool evaluate_solution()
    {
        m_solution_isValid = false;

        if (control_contains_nans())
        {
            std::cout << "Control contains nans. Terminate.\n\n\n";
            exit(EXIT_FAILURE);
        }
        else if (solve_status_is_valid())
        {
            m_solution_isValid = true;
        }

        m_solution_cost = get_cost();
        if (m_solution_isValid)
        {
            m_control_traj = get_control_trajectory();
            m_state_traj = get_state_trajectory();
            return true; // Return true if solution is valid
        }
        return false; // Return false if solution is not valid
    }

    bool m_re_init{false};
public:
    double p_rate{50};
protected:
    virtual inline bool ok() const { return true; }
    virtual inline void sleep() { mpc_utils::chrono_sleep(std::chrono::microseconds(static_cast<long>(1e6 / p_rate))); }

    /* Delay compensation */
    double p_delay_capt_apply{0};                  // Delay between capturing state from and applying control to plant

    int p_simulated_computation_delay_us{0};
    virtual inline long simulate_computation_delay(const long &real_comp_time_us)
    {
        /* If there is remaining (simulated) computation time, wait it to pass and return its duration */
        if (p_simulated_computation_delay_us > real_comp_time_us)
        {
            const long remaining_us = p_simulated_computation_delay_us - real_comp_time_us;
            mpc_utils::chrono_sleep(std::chrono::microseconds(remaining_us));
            return remaining_us;
        }
        return 0;
    }

    /* Data */
    unsigned p_reuse_state_max{2};                       // Maximum number of usages of same state
    unsigned m_state_usedCounter{p_reuse_state_max + 1}; // Count how many times same state is used
    Eigen::VectorXd m_state_c;                           // 'c'aptured state
    double m_tc_state{-1};                               // 'c'apture time of state

    /* Solution */
    bool m_solution_isValid{false};
    long m_comp_time_us{-1};
    double m_solution_cost{-1};
    Eigen::MatrixXd m_control_traj;
    Eigen::MatrixXd m_state_traj;

    mpc_utils::DurationLogger<mpc_utils::MicroSeconds, mpc_utils::MilliSeconds> durationLogger;
};

} // namespace mpc_control

#endif //SRC_MPCBASE_HPP