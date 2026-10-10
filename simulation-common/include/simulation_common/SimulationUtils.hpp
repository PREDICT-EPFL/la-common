#ifndef SRC_SIMULATIONUTILS_HPP
#define SRC_SIMULATIONUTILS_HPP

#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>

namespace sim_utils {

template<typename Rep, typename Period>
inline void chrono_sleep(const std::chrono::duration<Rep, Period> &duration)
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

} // namespace control

#endif //SRC_SIMULATIONUTILS_HPP
