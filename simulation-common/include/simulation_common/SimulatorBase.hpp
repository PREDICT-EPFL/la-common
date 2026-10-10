#ifndef SRC_SIMBASE_HPP
#define SRC_SIMBASE_HPP

#include <memory>
#include <iostream>
#include <iomanip>

#include <vector>
#include <Eigen/Core>
#include "sim_engine/SimEngineBase.hpp"
#include "SimulationUtils.hpp"

namespace simulation {

inline std::vector<double> EigenXd2vec(const Eigen::VectorXd &eigen_vec)
{
    return {eigen_vec.data(), eigen_vec.data() + eigen_vec.size()};
}
inline Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned> vec2EigenXd(const std::vector<double> &vec)
{
    return {vec.data(), static_cast<int>(vec.size())};
}

class SimulatorBase
{
public:
    void set_rate(double rate)
    {
        p_rate.set_rate(rate);
        if (m_VERBOSE >= PrintLevel1) std::cout << "Rate set to " << rate << " Hz\n";
        update_sim_dt();
    }
    void set_sim_speed(double sim_speed)
    {
        if (sim_speed > 20.0)
        {
            std::cout << "Sim speed cannot exceed 20! ";
            sim_speed = 20.0;
        }
        else if (sim_speed <= 0.0)
        {
            std::cout << "Sim speed must be positive! ";
            sim_speed = 1.0;
        }
        p_sim_speed = sim_speed;
        if (m_VERBOSE >= PrintLevel1) std::cout << "Sim speed set to " << sim_speed << "\n";
        update_sim_dt();
    }
    void reset_sim_time() { m_sim_time = 0; }

    void set_state(const Eigen::VectorXd &state) { m_state = state; }
    void set_control(const Eigen::VectorXd &control) { m_control = control; }
    double get_sim_time() const { return m_sim_time; }
    const Eigen::VectorXd &get_state() const { return m_state; }
    const Eigen::VectorXd &get_control() const { return m_control; }
    const Eigen::VectorXd &get_output() const { return m_output; }

    bool run_once()
    {
        update_sim_time();      // New simulation time
        update_in_impl();       // Data in
        const bool ok = simulate_once_impl();   // Simulation step
        update_out_impl();      // Data out

        return ok;
    }
    void run(double final_sim_time = -1)
    {
        print_init();
        init_impl();

        std::cout << std::fixed << std::setprecision(1);
        std::cout << "----------------------------------------------------------------------------\n";
        std::cout << "[Simulator]: Running at " << p_rate.get_rate() << " Hz";
        if (final_sim_time > 0.0) std::cout << " to final sim time " << final_sim_time << " s";
        std::cout << " ...\n";

        while (ok())
        {
            if (!run_once()) break; // Execute computations for one simulation step
            if (final_sim_time > 0.0 && m_sim_time >= final_sim_time)
            {
                print_final_time();
                break;
            }
            sleep();                // Sleep remaining time to match rate
        }
    }
    void test(unsigned n_iterations = 1)
    {
        print_init();
        init_impl();

        std::cout << std::fixed << std::setprecision(1);
        std::cout << "----------------------------------------------------------------------------\n";
        std::cout << "[Simulator]: Running at " << p_rate.get_rate() << " Hz"
                  << " for " << n_iterations << " iteration(s) ...\n";

        for (unsigned i = 0; i < n_iterations; ++i)
        {
            run_once();             // Execute computations for one simulation step
            sleep();                // Sleep remaining time to match rate
        }
    }

protected:
    SimulatorBase() = default;
    ~SimulatorBase() = default;

    /** Functions to be specified by the user in child class **/
    virtual void init_impl() { /* Empty */ }
    virtual void update_in_impl() { /* Empty */ }
    virtual void update_out_impl() { /* Empty */ }
    virtual bool simulate_once_impl()
    {
        std::cout << "SimulatorBase::simulate_once_impl(): Override me! And potentially also:\n"
                  << "init_impl()\n" << "update_in_impl()\n" << "update_out_impl()\n" << "\n";
        return true;
    }

    /** Functions optionally specified by the user **/
    virtual inline bool ok() { return true; }
    virtual inline void sleep() { p_rate.sleep(); }

    /* Print helpers */
    void print_init()
    {
        std::cout << std::fixed << std::setprecision(1);
        std::cout << " \n"
                  << "============================================================================\n"
                  << "Simulator @ " << get_rate() << " Hz, Sim speed: " << p_sim_speed << "x\n";
    }
    static void print_final_time()
    {
        std::cout << "\n"
                  << "--------------------------------------\n"
                     "Final time reached. End of simulation.\n"
                  << "--------------------------------------\n";
    }

    /** Normal functions **/
    double get_rate() const { return p_rate.get_rate(); }
    double get_sim_dt() const { return p_sim_dt; }
    void update_sim_time(double elapsed_time)
    {
        m_sim_time += elapsed_time;
        if (m_VERBOSE >= PrintLevel2) std::cout << "[sim_time " << std::round(m_sim_time * 10000) / 10 << " ms]\n";
    }
    void update_sim_time()
    {
        update_sim_time(get_sim_dt());
    }

    std::shared_ptr<simulation::engine::SimEngineBase> m_sim_engine;
    double m_sim_time{0};
    Eigen::VectorXd m_state;
    Eigen::VectorXd m_control;
    Eigen::VectorXd m_params;
    Eigen::VectorXd m_output;
    Eigen::VectorXd m_dyn_out;

    /* Debug helpers */
    enum PrintLevel
    {
        PrintLevel0, // Nothing
        PrintLevel1, // One-time settings before run
        PrintLevel2  // Continuous prints during run
    };
    const static int m_VERBOSE = PrintLevel0;
    template<typename T = bool>
    static T print_base(const std::string &title)
    {
        std::cout << "Base::" << title << "\n";
        return {};
    }

private:
    sim_utils::RateKeeper p_rate{100};
    double p_sim_speed{1.0};
    double p_sim_dt{0.01};
    void update_sim_dt()
    {
        p_sim_dt = std::round((p_sim_speed / p_rate.get_rate()) * 1e6) / 1e6;  // Round to 1 us
        if (m_VERBOSE >= PrintLevel1) std::cout << "update_sim_dt: " << p_sim_dt << "\n";
    }
};

}

#endif //SRC_SIMBASE_HPP
