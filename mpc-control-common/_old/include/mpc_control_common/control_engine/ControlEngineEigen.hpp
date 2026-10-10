#ifndef SRC_CONTROLENGINEEIGEN_HPP
#define SRC_CONTROLENGINEEIGEN_HPP

#include "ControlEngineBase.hpp"

template<typename EigenMPC>
class EigenControlEngine : public ControlEngineBase, public EigenMPC
{
public:
    using state_t = typename EigenMPC::state_t;
    using control_t = typename EigenMPC::control_t;

    void set_state_bounds(const DM &lbx, const DM &ubx) override
    {
        state_t lbx_, ubx_;
        toEigen(lbx, lbx_);
        toEigen(ubx, ubx_);
        this->state_bounds(lbx_, ubx_);
    }
    void set_control_bounds(const DM &lbu, const DM &ubu) override
    {
        control_t lbu_, ubu_;
        toEigen(lbu, lbu_);
        toEigen(ubu, ubu_);
        this->control_bounds(lbu_, ubu_);
    }
    void set_initial_guess(const DM &X) override
    {
        DM X_guess_ = X;
        if (X_guess_.size2() < 2)
        {
            X_guess_ = DM::repmat(X_guess_, 1, EigenMPC::num_nodes);
        }

        Eigen::Matrix<typename EigenMPC::scalar_t, EigenMPC::nx, EigenMPC::num_nodes> X_guess;
        toEigen(X_guess_, X_guess);
//        this->x_guess_reshaped(X_guess);
        this->x_guess(eigen_casadi_conversion::reshape<EigenMPC::nx * EigenMPC::num_nodes, 1>(X_guess));
    }

    void compute_control(const DM &x0) override
    {
        state_t x0_;
        toEigen(x0, x0_);
        this->initial_conditions(x0_);
        this->solver().solve();
        solve_status = this->info().status.value;
//        std::cout << "tf: " << this->solution_p()(0) << "\n";
    }

    DM get_control() const override { return toDM(this->solution_u_at(0)); }
    DM get_control_at(double t) const { return toDM(this->solution_u_at(t)); }

    DM get_time_trajectory() const { return toDM(this->time_grid.transpose()); }
    DM get_control_trajectory() const override { return toDM(this->solution_u_reshaped()); }
    DM get_state_trajectory() const override { return toDM(this->solution_x_reshaped()); }

    DM get_time_trajectory_resampled(double Ts) const { return toDM(this->time_grid_resampled(Ts).transpose()); }
    DM get_state_trajectory_resampled(double Ts) const { return toDM(this->solution_x_resampled(Ts)); }
    DM get_control_trajectory_resampled(double Ts) const { return toDM(this->solution_u_resampled(Ts)); }
};

#endif //SRC_CONTROLENGINEEIGEN_HPP
