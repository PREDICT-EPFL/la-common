#ifndef SRC_CONTROLBASE_HPP
#define SRC_CONTROLBASE_HPP

#include "ControlUtils.hpp"
#include "ControlTypes.hpp"

namespace control_common {

class ControlBase
{
public:
    double get_rate() const { return p_rate.get_rate(); }
    void set_rate(const double rate)
    {
        p_rate.set_rate(rate);
        if (m_VERBOSE >= PrintLevel1) std::cout << "Rate set to " << rate << " Hz\n";
    }

    void init() { init_impl(); }

    void run_once()
    {
        if (m_VERBOSE >= PrintLevel2) { std::cout << "run_once\n"; }
        update_in();
        compute_control();
        update_out();
    }
    void run()
    {
        std::cout << "\n----------------------------------------------------------------------------\n";
        std::cout << "[Controller]: Running at " << p_rate.get_rate() << " Hz\n";
        init();

        while (ok())
        {
            run_once();
            sleep();
        }
    }
    void test()
    {
        std::cout << "Test...\n";
        init();
        test_impl();
        print_test_res_impl();
    }

    /* Wraps compute_control_impl() and logs computation time */
    void compute_control()
    {
        using namespace std::chrono;
        const steady_clock::time_point start = steady_clock::now();
        compute_control_impl();
        const steady_clock::time_point stop = steady_clock::now();
        const auto duration = duration_cast<microseconds>(stop - start);
        m_comp_time_us = duration.count();
        durationLogger.log(static_cast<double>(m_comp_time_us));
    }

protected:
    ControlBase() = default;
    ~ControlBase() { durationLogger.printStatistics(); }

    long m_comp_time_us{-1};
    utils::DurationLogger<utils::TimeUnit::MicroSeconds, utils::TimeUnit::MilliSeconds> durationLogger;

    /** Functions to be specified by the user in child class **/
    virtual void init_impl() { /* Empty */ }
    virtual void update_in() { print_base("update_in()"); }
    virtual void update_out() { print_base("update_out()"); }
    virtual void compute_control_impl() { print_base("compute_control_impl"); }

    /** Functions optionally specified by the user **/
    virtual inline bool ok() { return true; }
    virtual inline void sleep() { p_rate.sleep(); }

    virtual void test_impl() { print_base("test_impl()"); }
    virtual void print_test_res_impl() {}

    /** Activation **/
    utils::ActiveParser activeParser;
    bool main_control_isActive{false};
    bool main_control_wasActive{false};

    /** UI **/
    static void printStatus(const std::string &text)
    {
        if (text.substr(0, 1) == "\n")
        {
            std::cout << "\n" << "[ STATUS ]: " << text.substr(1, text.length() - 1) << "\n";
        }
        else
        {
            std::cout << "[ STATUS ]: " << text << "\n";
        }
    }
    static void printModeChange(const std::string &text)
    {
        if (text.substr(0, 1) == "\n")
        {
            std::cout << "\n" << "[ MODE CHANGE ]: " << text.substr(1, text.length() - 1) << "\n";
        }
        else
        {
            std::cout << "[ MODE CHANGE ]: " << text << "\n";
        }
    }

    /* Debug helpers */
    enum PrintLevel
    {
        PrintLevel0, // Nothing
        PrintLevel1, // One-time settings before run
        PrintLevel2  // Continuous prints during run
    };
    static const int m_VERBOSE = PrintLevel0;
    template<typename T = bool>
    static T print_base(const std::string &title)
    {
        std::cout << "Base::" << title << "\n";
        return {};
    }

protected:
    utils::RateKeeper p_rate{100}; // 100 Hz default
};

} // namespace control_common

#endif //SRC_CONTROLBASE_HPP
