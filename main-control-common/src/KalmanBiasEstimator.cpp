#include "main_control_common/KalmanBiasEstimator.hpp"
#include <cmath>
#include <stdexcept>

using namespace control_common::estimation;

KalmanBiasEstimator::KalmanBiasEstimator(double process_noise_Q,
                                         double measurement_noise_R,
                                         double initial_mean,
                                         double initial_variance)
    : Q_(process_noise_Q)
    , R_(measurement_noise_R)
    , mu_(initial_mean)
    , sigma2_(initial_variance)
{
    if (R_ <= 0.0)
        throw std::invalid_argument("measurement_noise_R must be positive");
    if (Q_ < 0.0)
        throw std::invalid_argument("process_noise_Q must be non-negative");
    if (sigma2_ < 0.0)
        throw std::invalid_argument("initial_variance must be non-negative");
}

void KalmanBiasEstimator::update(double e_y)
{
    // --- Predict ---
    // Mean is unchanged (bias random walk has zero mean increment)
    sigma2_ += Q_;

    // --- Update ---
    const double K = sigma2_ / (sigma2_ + R_);   // Kalman gain
    mu_     += K * (e_y - mu_);
    sigma2_ *= (1.0 - K);
}

void KalmanBiasEstimator::reset(double mean, double variance)
{
    mu_     = mean;
    sigma2_ = variance;
}

double KalmanBiasEstimator::stddev() const
{
    return std::sqrt(sigma2_);
}
