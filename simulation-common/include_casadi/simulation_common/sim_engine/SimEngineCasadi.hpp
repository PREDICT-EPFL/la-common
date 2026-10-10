#ifndef SRC_SIMENGINECASADI_HPP
#define SRC_SIMENGINECASADI_HPP

#include <memory>

#include "casadi/casadi.hpp"
#include "simulation_common/sim_engine/SimEngineBase.hpp"
#include "casadi_integration.hpp"

#include "simulation_common/eigen_casadi_conversion.hpp"

namespace simulation {
namespace engine_casadi {

/** Casadi + Integrator Simulator Engine **/
using namespace casadi;

class SimEngineCasadi : public engine::SimEngineBase
{
public:
    SimEngineCasadi() : SimEngineBase() {}
    SimEngineCasadi(const Function &dynamics_func, const double &sim_dt,
                    const Function &output_func = Function(),
                    const Function &dyn_out_func = Function())
    {
        setup_solver(dynamics_func, sim_dt);
        m_output_func = output_func;
        m_dyn_out_func = dyn_out_func;
    }
    ~SimEngineCasadi() override = default;

    void initialize(const Eigen::VectorXd &init_state,
                    const Eigen::VectorXd &default_control,
                    const Eigen::VectorXd &default_dyn_params) override
    {
        m_state = eigen_casadi_conversion::toDM(init_state);
        m_control = eigen_casadi_conversion::toDM(default_control);
        m_params = eigen_casadi_conversion::toDM(default_dyn_params);
    }
    void set_state(const Eigen::VectorXd &state) override { m_state = eigen_casadi_conversion::toDM(state); }
    void set_control(const Eigen::VectorXd &control) override { m_control = eigen_casadi_conversion::toDM(control); }
    void set_params(const Eigen::VectorXd &params) override { m_params = eigen_casadi_conversion::toDM(params); }
    void simulate_one_step() override
    {
        m_state = m_solver_ptr->solve(m_state, m_control, m_params, p_dt);
    }
    void simulate_one_step(const Eigen::VectorXd &control) override
    {
        m_control = eigen_casadi_conversion::toDM(control);
        simulate_one_step();
    }
    void simulate_one_step(const Eigen::VectorXd &state, const Eigen::VectorXd &control) override
    {
        m_state = eigen_casadi_conversion::toDM(state);
        simulate_one_step(control);
    }
    Eigen::VectorXd get_state() override { return eigen_casadi_conversion::toEigenX(m_state); }
    Eigen::VectorXd get_control() override { return eigen_casadi_conversion::toEigenX(m_control); }
    Eigen::VectorXd get_output() override
    {
        if (m_output_func.n_in() == 2)
        {
            return eigen_casadi_conversion::toEigenX(m_output_func(DMVector{m_state, m_control})[0]);
        }
        else
        {
            return eigen_casadi_conversion::toEigenX(m_output_func(DMVector{m_state, m_control, m_params})[0]);
        }
    }
    Eigen::VectorXd get_dyn_out() override
    {
        if (m_dyn_out_func.n_in() == 2)
        {
            return eigen_casadi_conversion::toEigenX(m_dyn_out_func(DMVector{m_state, m_control})[0]);
        }
        else
        {
            return eigen_casadi_conversion::toEigenX(m_dyn_out_func(DMVector{m_state, m_control, m_params})[0]);
        }
    }
    Eigen::VectorXd get_dyn_out(const Eigen::VectorXd &control) override
    {
        set_control(control);
        return get_dyn_out();
    }
private:
    DM m_state;
    DM m_control;
    DM m_params;
    Function m_output_func;
    Function m_dyn_out_func;

    double p_dt{0.01};
    std::shared_ptr<casadi_integration::ODESolver> m_solver_ptr;
    void setup_solver(const Function &dynamics_func, const double &sim_dt)
    {
        p_dt = sim_dt;
        Dict solver_params({//{"tf",     sim_dt}, // Not supported in newer CVODES version
                            {"tol",    1e-6},
                            {"method", casadi_integration::CVODES}});
        m_solver_ptr = std::make_shared<casadi_integration::ODESolver>(dynamics_func, solver_params);
    }
};

} // namespace engine
} // namespace simulation

#endif //SRC_SIMENGINECASADI_HPP
