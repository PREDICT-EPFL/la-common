#ifndef SRC_MPCUTILS_HPP
#define SRC_MPCUTILS_HPP

#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>

namespace mpc_utils {

template<typename T = bool>
static T print_base(const std::string &title)
{
    std::cout << "Base::" << title << "\n";
    return {};
}

template<typename Rep, typename Period>
inline void chrono_sleep(const std::chrono::duration<Rep, Period> &duration)
{
    std::this_thread::sleep_for(duration);
}

enum TimeUnit
{
    Seconds, MilliSeconds, MicroSeconds
};
template<int time_unit_in = MicroSeconds, int time_unit_out = MilliSeconds>
struct DurationLogger
{
public:

    void log(const double &delay)
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

} // namespace mpc_utils

#endif //SRC_MPCUTILS_HPP
