#ifndef SRC_CONTROLENGINEBASE_HPP
#define SRC_CONTROLENGINEBASE_HPP

#include <Eigen/Core>

namespace mpc_control {
namespace engine {

class ControlEngineBase
{
public:
    ControlEngineBase() = default;

    virtual void set_state_bounds(const Eigen::VectorXd &x_lb, const Eigen::VectorXd &x_ub) = 0;
    virtual void set_input_bounds(const Eigen::VectorXd &u_lb, const Eigen::VectorXd &u_ub) = 0;

    virtual void set_initial_guess(const Eigen::VectorXd &X_guess) = 0;

    virtual void compute_control(const Eigen::VectorXd &x0) = 0;

    virtual void get_input() const = 0;
    virtual void get_input_trajectory() const = 0;
    virtual void get_state_trajectory() const = 0;
};

} // namespace engine
} // namespace mpc_control

#endif //SRC_CONTROLENGINEBASE_HPP
