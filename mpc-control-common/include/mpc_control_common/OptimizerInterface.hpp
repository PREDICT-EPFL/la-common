#ifndef SRC_OPTIMIZERINTERFACE_HPP
#define SRC_OPTIMIZERINTERFACE_HPP

#include <Eigen/Core>

namespace mpc_control_common {

class OptimizerInterface
{
public:
    OptimizerInterface() = default;
    virtual ~OptimizerInterface() = default;

    virtual bool init() { return true; } // optional
    virtual void reset() {} // optional

    virtual void set_initial_state(const Eigen::VectorXd &x0) = 0;
    virtual bool compute_control() = 0;

    virtual Eigen::VectorXd get_control() const = 0;
    virtual Eigen::VectorXd get_time_grid() const = 0;
    virtual Eigen::MatrixXd get_control_trajectory() const = 0;
    virtual Eigen::MatrixXd get_state_trajectory() const = 0;
    virtual Eigen::VectorXd get_opt_param() const { return Eigen::VectorXd(); } // optional
    virtual double get_cost() const { return 0; } // optional
    virtual int get_no_iterations() const { return 0; } // optional
    virtual int get_no_iterations_low() const { return 0; } // optional
};

}

#endif //SRC_OPTIMIZERINTERFACE_HPP
