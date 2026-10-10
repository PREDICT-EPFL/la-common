#ifndef SRC_ODEINTPLANT_HPP
#define SRC_ODEINTPLANT_HPP

namespace odeint_model {

template<typename DynamicsModel>
class OdeIntPlant : public DynamicsModel
{
public:
    OdeIntPlant()
    {
        m_input.setZero();
        m_param.setZero();
    }

    // Redefine for visibility
    using Scalar = typename DynamicsModel::Scalar;
    using State = typename DynamicsModel::State;
    using Input = typename DynamicsModel::Input;
    using Param = typename DynamicsModel::Param;
    using Output = typename DynamicsModel::Output;
    using DynOut = typename DynamicsModel::DynOut;

    Input m_input;
    void set_input(const Input &input) { m_input = input; }
    const Input &get_input() { return m_input; }

    Param m_param;
    void set_param(const Param &param) { m_param = param; }
    const Param &get_param() { return m_param; }

    void operator()(const State &x, State &x_dot, const Scalar &t) const noexcept
    {
        (void) t; // Silence parameter unused warning
        DynOut dyn_out;
        x_dot = DynamicsModel::template dynamics<Scalar>(x, m_input, m_param, dyn_out);
    }
    DynOut eval_dyn_out(const State &x)
    {
        return eval_dyn_out(x, m_input);
    }
    DynOut eval_dyn_out(const State &x, const Input &u)
    {
        DynOut dyn_out;
        DynamicsModel::template dynamics<Scalar>(x, u, m_param, dyn_out);
        return dyn_out;
    }
    Output output(const State &x)
    {
        return DynamicsModel::template output<Scalar>(x, m_input, m_param);
    }
    Output output(const State &x, const Input &u)
    {
        return DynamicsModel::template output<Scalar>(x, u, m_param);
    }
};

}

#endif //SRC_ODEINTPLANT_HPP
