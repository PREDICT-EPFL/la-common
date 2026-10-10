#ifndef SRC_SCALARBIASESTIMATOR_HPP
#define SRC_SCALARBIASESTIMATOR_HPP

namespace control_common {
namespace estimation {

/**
 * KalmanBiasEstimator (Written by Claude.ai)
 *
 * Estimates a slowly-drifting scalar bias from noisy measurements.
 *
 * Model:
 *   b_{k+1} = b_k + w_k,   w_k ~ N(0, Q)   (process)
 *   e_y_k   = b_k + v_k,   v_k ~ N(0, R)   (measurement)
 */
class KalmanBiasEstimator
{
public:
    /**
     * @param process_noise_Q     How much the bias is expected to drift per step.
     *                            Q = 0  -> bias is constant.
     *                            Q > 0  -> bias can drift; sets steady-state variance floor.
     * @param measurement_noise_R Variance of the noise on e_y.
     * @param initial_mean        Prior mean of the bias (default 0).
     * @param initial_variance    Prior variance / confidence in initial_mean (default 1).
     */
    KalmanBiasEstimator(double process_noise_Q,
                        double measurement_noise_R,
                        double initial_mean     = 0.0,
                        double initial_variance = 1.0);

    /**
     * Feed one new output-error sample and update the estimate.
     * @param e_y  Measured output error: r_corrected - y
     */
    void update(double e_y);

    /** Reset state to given mean and variance. */
    void reset(double mean = 0.0, double variance = 1.0);

    double mean()     const { return mu_;     }
    double variance() const { return sigma2_; }
    double stddev()   const;

private:
    double Q_;       // process noise variance
    double R_;       // measurement noise variance
    double mu_;      // current mean estimate
    double sigma2_;  // current variance estimate
};


}
}

#endif //SRC_SCALARBIASESTIMATOR_HPP