#ifndef SRC_MPCINTERFACE_HPP
#define SRC_MPCINTERFACE_HPP

#include <Eigen/Core>

class MpcInterface
{
public:
    virtual void set_state(const Eigen::VectorXd &x0) = 0;
    virtual void compute_control(const Eigen::VectorXd &x0 = Eigen::VectorXd()) = 0;
    virtual const Eigen::VectorXd &get_control() const = 0;

private:

};

#endif //SRC_MPCINTERFACE_HPP
