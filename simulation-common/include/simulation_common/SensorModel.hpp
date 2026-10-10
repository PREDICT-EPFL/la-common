#ifndef SRC_SENSORMODEL_HPP
#define SRC_SENSORMODEL_HPP

#include <random>
#include <array>
#include <Eigen/Core>

namespace sensor_models {

namespace random_gen {

class UniformRandGen
{
public:
    UniformRandGen() { distribution = std::uniform_real_distribution<double>(0, 1); }
    double getValue() { return distribution(generator); }
    double getValueBetween(const double &lower_bound, const double &upper_bound)
    {
        return lower_bound + distribution(generator) * (upper_bound - lower_bound);
    }
private:
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution;
};

class WhiteNoiseGen
{
public:
    WhiteNoiseGen() { distribution = std::normal_distribution<double>(0, 1); }
    double getValue() { return distribution(generator); }
    double getValueBetween(double lower, double upper)
    { // Returns samples from the normal distribution, s.t. lower/higher match with -/+ 2 * standard error
        const double mean = 0.5 * (lower + upper);
        return mean + distribution(generator) * (upper - mean) / 2.0;
    }
private:
    std::default_random_engine generator;
    std::normal_distribution<double> distribution;
};

} // namespace random_gen

class SimpleSensor
{
public:
    explicit SimpleSensor(double variance = 0, double bias = 0)
    {
        setup(variance, bias);
    }
    void setup(double variance, double bias)
    {
        p_var = variance;
        p_bias = bias;
    }
    double get_value(double value_true)
    {
        return value_true + p_var * m_whiteNoiseGen.getValue() + p_bias;
    }

private:
    random_gen::WhiteNoiseGen m_whiteNoiseGen{};
    double p_var{0};
    double p_bias{0};
};

class SimpleSensor3D
{
public:
    explicit SimpleSensor3D(double var_x = 0, double var_y = 0, double var_z = 0,
                            double bias_x = 0, double bias_y = 0, double bias_z = 0)
    {
        setup(var_x, var_y, var_z, bias_x, bias_y, bias_z);
    }
    void setup(double var_x, double var_y, double var_z,
               double bias_x = 0, double bias_y = 0, double bias_z = 0)
    {
        sens_xyz[0].setup(var_x, bias_x);
        sens_xyz[1].setup(var_y, bias_y);
        sens_xyz[2].setup(var_z, bias_z);
    }
    Eigen::Vector3d get_noisy(const Eigen::Vector3d &value_true)
    {
        return {sens_xyz[0].get_value(value_true(0)),
                sens_xyz[1].get_value(value_true(1)),
                sens_xyz[2].get_value(value_true(2))};
    }
private:
    std::array<SimpleSensor, 3> sens_xyz;
};

}

#endif //SRC_SENSORMODEL_HPP
