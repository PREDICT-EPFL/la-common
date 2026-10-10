#ifndef SRC_CONTROLUTILS_HPP
#define SRC_CONTROLUTILS_HPP

#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>

namespace control_common {
namespace utils {

template<typename Rep, typename Period>
void chrono_sleep(const std::chrono::duration<Rep, Period> &duration)
{
    std::this_thread::sleep_for(duration);
}

class RateKeeper
{
public:
    explicit RateKeeper(double reference_rate_hz)
    {
        set_rate(reference_rate_hz);
        set_start();
    }
    void set_cycle_duration_microseconds(long duration_us)
    {
        if (duration_us > 0)
        {
            ref_duration_us = duration_us;
        }
        else { std::cerr << "Cycle duration must be > 0\n"; }
    }
    long get_cycle_duration_microseconds() const { return ref_duration_us; }
    void set_rate(double reference_rate_hz)
    {
        if (reference_rate_hz > 0)
        {
            ref_duration_us = static_cast<long>(1e6 / reference_rate_hz);
        }
        else { std::cerr << "Rate must must be > 0\n"; }
    }
    double get_rate() const
    {
        return 1e6 / static_cast<double>(ref_duration_us);
    }

    void set_start()
    {
        m_cycle_time = std::chrono::steady_clock::now();
    }
    void sleep()
    {
        using namespace std::chrono;
        const steady_clock::time_point t_now = steady_clock::now();
        const auto duration_passed_us = duration_cast<microseconds>(t_now - m_cycle_time);

        if (ref_duration_us > duration_passed_us.count())
        {
            const unsigned long sleep_duration_us = ref_duration_us - duration_passed_us.count();
//            std::cout << "duration_passed: " << duration_passed.count() << " ns, "
//                      << "sleep_duration: " << sleep_duration_ns << " ns\n";
            chrono_sleep(microseconds(sleep_duration_us));
        }
        else
        {
//            std::cout << "overdue by " << duration_passed_ns.count() - ref_duration_ns << "ns\n";
        }

        m_cycle_time = steady_clock::now();
    }

private:
    long ref_duration_us{10000}; // 10 ms // 100 Hz
    std::chrono::steady_clock::time_point m_cycle_time;
};

enum TimeUnit
{
    Seconds, MilliSeconds, MicroSeconds
};
template<int time_unit_in = MicroSeconds, int time_unit_out = MilliSeconds>
struct DurationLogger
{
public:

    void log(double delay)
    {
        if (n_measurements == 0) { delay_min = delay_max = delay; }
        delay_sum += delay;
        delay_avg = delay_sum / ++n_measurements;
        delay_max = std::max(delay_max, delay);
        delay_min = std::min(delay_min, delay);
    };
    void printStatistics()
    {
        std::string unit_out_str;
        double scaling{9999999};
        if (time_unit_out == MicroSeconds)
        {
            unit_out_str = "us";
            if (time_unit_in == MicroSeconds)
                scaling = 1;
            else if (time_unit_in == MilliSeconds)
                scaling = 1e3;
            else
                scaling = 1e6;
        }
        else if (time_unit_out == Seconds)
        {
            unit_out_str = "s";
            if (time_unit_in == MicroSeconds)
                scaling = 1e-6;
            else if (time_unit_in == MilliSeconds)
                scaling = 1e-3;
            else
                scaling = 1;
        }
        else // time_unit_out == MilliSeconds
        {
            unit_out_str = "ms";
            if (time_unit_in == MicroSeconds)
                scaling = 1e-3;
            else if (time_unit_in == MilliSeconds)
                scaling = 1;
            else // time_unit_in == Seconds
                scaling = 1e3;
        }

        std::cout << "\nComputation statistics [" << unit_out_str << "]: "
                  << "\nmax: " << std::setw(3) << delay_max * scaling
                  << "\navg: " << std::setw(3) << delay_avg * scaling
                  << "\nmin: " << std::setw(3) << delay_min * scaling
                  << "\n";
    };
    void reset()
    {
        n_measurements = 0;
        delay_sum = delay_avg = delay_max = delay_min = 0;
    }
private:
    unsigned n_measurements{0};
    double delay_sum{0};
    double delay_avg{0};
    double delay_max{0};
    double delay_min{0};
};

class ActiveParser
{
public:
    enum ActivationMode { TAP, HOLD } activation_mode{HOLD};

    void set_activation_mode(ActivationMode activationMode)
    {
        switch (activationMode)
        {
            case TAP:
            case HOLD:
                // OK
                break;
            default:
                activationMode = HOLD;
                std::cout << "ActiveParser: Unknown activation mode. Using HOLD as default\n";
        }
        activation_mode = activationMode;
    }

    enum Compare
    {
        GREATER, SMALLER, EQUAL
    };
    template<typename T = bool>
    bool wants_active_hold(T active_switch, Compare comparison_operator, const T &value)
    {
        bool wants_active = press(active_switch, comparison_operator, value);
        wanted_active = wants_active;
        return wants_active;
    }
    template<typename T = bool>
    bool wants_active_tap(T active_switch, Compare comparison_operator, const T &value)
    {
        bool wants_active = press(active_switch, comparison_operator, value);
        bool return_value = (wants_active && !wanted_active);
        wanted_active = wants_active;
        return return_value;
    }

    template<typename T = bool>
    bool wants_active(T active_switch, Compare comparison_operator, const T &value)
    {
        if (activation_mode == TAP)
        {
            return wants_active_tap<T>(active_switch, comparison_operator, value);
        }
        else
        {
            return wants_active_hold<T>(active_switch, comparison_operator, value);
        }
    }

private:
    bool wanted_active{false};

    template<typename T = bool>
    bool press(T active_switch, Compare comparison_operator, const T &value)
    {
        bool wants_active{false};

        /* User condition for active */
        switch (comparison_operator)
        {
            case GREATER:
                wants_active = (active_switch > value);
                break;
            case SMALLER:
                wants_active = (active_switch < value);
                break;
            case EQUAL:
                wants_active = (active_switch == value);
                break;
            default:
                wants_active = false;
        }
        return wants_active;
    }
};

}
}

#endif //SRC_CONTROLUTILS_HPP
