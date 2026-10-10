#ifndef SRC_CONTROLENGINECASADI_HPP
#define SRC_CONTROLENGINECASADI_HPP

#include <memory>

#include "mpc_control_common/control_engine/ControlEngineBase.hpp"
#include "casadi/casadi.hpp"

namespace mpc_control {
namespace engine {

template<typename CasadiOCP>
class ControlEngineCasadi : public ControlEngineBase
{
public:
    void set_state_bounds(const DM &lbx, const DM &ubx) override
    {
        m_controller->set_state_box_constraints(lbx, ubx);
    }
    void set_control_bounds(const DM &lbu, const DM &ubu) override
    {
        m_controller->set_control_box_constraints(lbu, ubu);
    }

    void compute_control(const DM &x0) override
    {
        m_controller->computeControl(x0);
    }

    DM get_control() const override { return m_controller->getOptimalControl()(Slice(), 0); }
    DM get_control_trajectory() const override { return m_controller->getOptimalControl(); }
    DM get_state_trajectory() const override { return m_controller->getOptimalTrajectory(); }

    std::shared_ptr<CasadiOCP> m_controller;
    CasadiOCP &controller() { return m_controller; }
};

} // namespace engine
} // namespace mpc_control

#endif //SRC_CONTROLENGINECASADI_HPP
