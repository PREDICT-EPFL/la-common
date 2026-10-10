#ifndef SRC_SIMENGINEEIGEN_HPP
#define SRC_SIMENGINEEIGEN_HPP

#include <Eigen/Core>
#include <boost/numeric/odeint.hpp>

#include "simulation_common/sim_engine/SimEngineBase.hpp"

namespace simulation {
namespace engine {

/** Eigen + Boost ODEint Simualator Engine **/
template<typename Plant>
class EigenSimEngine : public SimEngineBase
{
public:
    using Scalar = typename Plant::Scalar;
    using State = typename Plant::State;
    using Control = typename Plant::Input;
    using Param = typename Plant::Param;
    using Output = typename Plant::Output;
    using DynOut = typename Plant::DynOut;

    explicit EigenSimEngine(const Scalar &sim_dt)
    {
        dt = static_cast<Scalar>(sim_dt);
        m_state.setZero();
    }
    void initialize(const Eigen::VectorXd &init_state,
                    const Eigen::VectorXd &default_control,
                    const Eigen::VectorXd &default_dyn_params) override
    {
        //std::cout << "EigenSimEngine/initialize: \ninit state: " << init_state << "\ndefault_control: " << default_control << "\ndefault_dyn_params: " << default_dyn_params << "\n";
        m_state = init_state;
        plant.set_input(default_control);
        plant.set_param(default_dyn_params);
    }
    void set_state(const Eigen::VectorXd &state) override
    {
        m_state = state;
    }
    void set_control(const Eigen::VectorXd &control) override
    {
        Control m_control = control;
        plant.set_input(m_control);
    }
    void set_params(const Eigen::VectorXd &params_) override
    {
        Param m_param = params_;
        plant.set_param(m_param);
    }
    void simulate_one_step() override
    {
        m_integrator.do_step(plant, m_state, 0.0, dt);
    }
    void simulate_one_step(const Eigen::VectorXd &control) override
    {
        plant.set_input(control);
        simulate_one_step();
    }
    void simulate_one_step(const Eigen::VectorXd &state, const Eigen::VectorXd &control) override
    {
        m_state = state;
        simulate_one_step(control);
    }
    void simulate_one_step(const Eigen::VectorXd &state, const Eigen::VectorXd &control, double dt)
    {
        plant.set_input(control);
        m_integrator.do_step(plant, m_state, 0.0, dt);
    }
    Eigen::VectorXd get_state() override { return m_state; }
    Eigen::VectorXd get_control() override { return plant.get_input(); }
    Eigen::VectorXd get_params() override { return plant.get_param(); }
    Eigen::VectorXd get_output() override { return plant.output(m_state); }
    Eigen::VectorXd get_dyn_out() override
    {
        // If get_output() is called after simulate_one_step, we use the new state here already, which is technically wrong
        return plant.eval_dyn_out(m_state);
    }
    Eigen::VectorXd get_dyn_out(const Eigen::VectorXd &control) override
    {
        //set_control(control);
        // If get_output() is called after simulate_one_step, we use the new state here already, which is technically wrong
        return plant.eval_dyn_out(m_state, control);
    }

    void set_dt(double dt_) { dt = dt_; }

    Plant plant;
private:
    State m_state;

    using rkd5 = boost::numeric::odeint::runge_kutta_dopri5<State, Scalar, State, Scalar, boost::numeric::odeint::vector_space_algebra>;
    rkd5 m_integrator;

    Scalar dt{0.01};
};

}
}

#endif //SRC_SIMENGINEEIGEN_HPP
