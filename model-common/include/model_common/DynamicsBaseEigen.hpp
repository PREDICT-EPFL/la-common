#ifndef SRC_MODELINGBASEEIGEN_HPP
#define SRC_MODELINGBASEEIGEN_HPP

#include <iostream>
#include <vector>

#include <Eigen/Core>
#include <unsupported/Eigen/AutoDiff>
#include "eigen_autodiff_fix.hpp"

namespace model_common {
namespace eigen_model {

template<typename Scalar_, int nx_, int nu_ = 0, int np_ = 0, int ny_ = 0, int nz_ = 0>
class DynamicsBase
{
public:
    static const int nx = nx_;
    static const int nu = nu_;
    static const int np = np_;
    static const int ny = ny_;
    static const int nz = nz_;

    /* Templated types */
    template<typename scalar_t> using state_t = Eigen::Matrix<scalar_t, nx, 1>;
    template<typename scalar_t> using input_t = Eigen::Matrix<scalar_t, nu, 1>;
    template<typename scalar_t> using param_t = Eigen::Matrix<scalar_t, np, 1>;
    template<typename scalar_t> using output_t = Eigen::Matrix<scalar_t, ny, 1>;
    template<typename scalar_t> using dynout_t = Eigen::Matrix<scalar_t, nz, 1>;

    template<typename T> using constref_t = const Eigen::Ref<const T>;
    template<typename T> using ref_t = Eigen::Ref<T>;

    /* Scalar types */
    using Scalar = Scalar_;
    using State = state_t<Scalar>;
    using Input = input_t<Scalar>;
    using Param = param_t<Scalar>;
    using Output = output_t<Scalar>;
    using DynOut = dynout_t<Scalar>;
    using SystemMat = Eigen::Matrix<Scalar, nx, nx>;
    using ControlMat = Eigen::Matrix<Scalar, nx, nu>;

    /* AD (nx+nu) types */
    using Derivates = Eigen::Matrix<Scalar, nx + nu, 1>;
    using AdScalar = Eigen::AutoDiffScalar<Derivates>;
    using AdState = state_t<AdScalar>;
    using AdInput = input_t<AdScalar>;
    using AdParam = param_t<AdScalar>;
    using AdOutput = output_t<AdScalar>;
    using AdDynOut = dynout_t<AdScalar>;

    DynamicsBase()
    {
        x_trim_ubound.setZero();
        x_trim_lbound.setZero();
        u_trim_ubound.setZero();
        u_trim_lbound.setZero();
        u_ubound.setZero();
        u_lbound.setZero();
        x_default.setZero();
        x_trim.setZero();
        u_trim.setZero();
    }

    State get_default_initial_state() const { return x_default; }
    std::vector<Scalar> get_default_initial_state_vec() const
    {
        return std::vector<Scalar>(x_default.data(), x_default.data() + x_default.size());
    }

    void set_const_params(const Eigen::VectorXd &const_params) { (void) const_params; }
    Eigen::VectorXd get_const_params() const { return {}; }

    template<typename scalar_t>
    state_t<scalar_t> dynamics(const Eigen::Ref<const state_t<scalar_t>> &state,
                               const Eigen::Ref<const input_t<scalar_t>> &input,
                               const Eigen::Ref<const param_t<scalar_t>> &param,
                               Eigen::Ref<dynout_t<scalar_t>> dyn_out) const
    {
        (void) state;
        (void) input;
        (void) param;
        (void) dyn_out;
        std::cout << "DynamicsBase::dynamics(): Hide me!\n";
    }

    template<typename scalar_t>
    output_t<scalar_t> output(const Eigen::Ref<const state_t<scalar_t>> &state,
                              const Eigen::Ref<const input_t<scalar_t>> &input,
                              const Eigen::Ref<const param_t<scalar_t>> &param) const
    {
        (void) state;
        (void) input;
        (void) param;
        std::cout << "DynamicsBase::output(): Hide me!\n";
    }

    void linearize_at(Eigen::Matrix<Scalar, nx, nx> &A, Eigen::Matrix<Scalar, nx, nu> &B,
                      const State &state, const Input &control, const Param &param = Param())
    {
        // Set linearization point and seed derivatives
        const int NXU = nx + nu;
        AdState x0 = state;
        AdInput u0 = control;
        int derivative_idx = 0;
        for (int ix = 0; ix < nx; ++ix)
            x0(ix).derivatives() = Eigen::Matrix<Scalar, NXU, 1>::Unit(NXU, derivative_idx++);
        for (int iu = 0; iu < nu; ++iu)
            u0(iu).derivatives() = Eigen::Matrix<Scalar, NXU, 1>::Unit(NXU, derivative_idx++);

        // Propagate through the dynamics function
        AdParam p = param;
        AdOutput dyn_out; // Dummy
        AdState x_dot = dynamics(x0, u0, p, dyn_out);

        // Obtain Jacobian and segment it in df_dx (=A) and df_du (=B)
        Eigen::Matrix<Scalar, nx, NXU> df_dxu;
        for (int ix = 0; ix < nx; ++ix)
            df_dxu.row(ix) = x_dot(ix).derivatives();

        A = df_dxu.template block<nx, nx>(0, 0);
        B = df_dxu.template block<nx, nu>(0, nx);
        //std::cout << "A (=df_dx): \n" << A << "\n";
        //std::cout << "B (=df_du): \n" << B << "\n";
    }

    State get_trim_state() { return x_trim; }
    Input get_trim_control() { return u_trim; }
    void trim(const Param &param, const Eigen::Matrix<Scalar, nx + nu, 1> &xu0,
              const Input &LBU = {}, const Input &UBU = {},
              const State &LBX = {}, const State &UBX = {}) {}

    Input u_ubound;
    Input u_lbound;

    State x_trim_ubound;
    State x_trim_lbound;
    Input u_trim_ubound;
    Input u_trim_lbound;

protected:
    State x_default;
    State x_trim;
    Input u_trim;
};

} //namespace eigen_model
} //namespace flight_model

#endif //SRC_MODELINGBASEEIGEN_HPP
