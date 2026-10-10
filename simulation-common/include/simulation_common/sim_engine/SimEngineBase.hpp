#ifndef SRC_SIMENGINEBASE_HPP
#define SRC_SIMENGINEBASE_HPP

#include <iostream>
#include <Eigen/Core>

namespace simulation {
namespace engine {

class SimEngineBase
{
public:
    SimEngineBase() = default;
    virtual ~SimEngineBase() = default;

    virtual void initialize(const Eigen::VectorXd &init_state,
                            const Eigen::VectorXd &default_control,
                            const Eigen::VectorXd &default_dyn_params)
    {
        (void) init_state;
        (void) default_control;
        (void) default_dyn_params;
        std::cout << "SimEngineBase::initialize(): Override me!\n";
    }
    virtual void set_state(const Eigen::VectorXd &state)
    {
        (void) state;
        std::cout << "SimEngineBase::set_state(): Override me!\n";
    }
    virtual void set_control(const Eigen::VectorXd &control)
    {
        (void) control;
        std::cout << "SimEngineBase::set_control(): Override me!\n";
    }
    virtual void set_params(const Eigen::VectorXd &params)
    {
        (void) params;
        std::cout << "SimEngineBase::set_params(): Override me!\n";
    }
    virtual void simulate_one_step() { std::cout << "SimEngineBase::simulate_one_step(): Override me!\n"; }
    virtual void simulate_one_step(const Eigen::VectorXd &control)
    {
        (void) control;
        std::cout << "SimEngineBase::simulate_one_step(Eigen::VectorXd): Override me!\n";
    }
    virtual void simulate_one_step(const Eigen::VectorXd &state, const Eigen::VectorXd &control)
    {
        (void) state;
        (void) control;
        std::cout << "SimEngineBase::simulate_one_step(Eigen::VectorXd, Eigen::VectorXd): Override me!\n";
    }
    virtual Eigen::VectorXd get_state()
    {
        std::cout << "SimEngineBase::get_state(): Override me!\n";
        return {};
    }
    virtual Eigen::VectorXd get_control()
    {
        std::cout << "SimEngineBase::get_control(): Override me!\n";
        return {};
    }
    virtual Eigen::VectorXd get_params()
    {
        std::cout << "SimEngineBase::get_params(): Override me!\n";
        return {};
    }
    virtual Eigen::VectorXd get_output()
    {
        std::cout << "SimEngineBase::get_output(): Override me!\n";
        return {};
    }
    virtual Eigen::VectorXd get_dyn_out()
    {
        std::cout << "SimEngineBase::get_dyn_out(): Override me!\n";
        return {};
    }
    virtual Eigen::VectorXd get_dyn_out(const Eigen::VectorXd &control)
    {
        (void) control;
        std::cout << "SimEngineBase::get_dyn_out(): Override me!\n";
        return {};
    }
};

} // namespace engine
} // namespace simulation

#endif //SRC_SIMENGINEBASE_HPP
