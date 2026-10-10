#ifndef SRC_CASADI_INTEGRATION_HPP
#define SRC_CASADI_INTEGRATION_HPP

#include "casadi/casadi.hpp"

namespace casadi_integration {

enum IntType { RK4, CVODES };

/** Solve ODE of the form : xdot = f(x, u, p) */
class ODESolver
{
public:
    explicit ODESolver(const casadi::Function &f_dyn, const casadi::Dict &params = casadi::Dict())
    {
        using namespace casadi;

        /** define default values for parameters */
        Tolerance = 1e-8;
        MaxIter = 300;
        dT = 1;
        Method = CVODES; //{RK4}

        Parameters["method"] = Method;
        Parameters["tf"] = dT;
        Parameters["max_iter"] = MaxIter;
        Parameters["tol"] = Tolerance;

        /** set user defined parameters */
        bool use_default_params{false};
        if (params.empty())
            use_default_params = true;
        else
            updateParams(params);

        /** @todo: revise parameters */
        dT = Parameters["tf"];
        /** Define integration scheme here */
        /** space dimensionality */
        nx = static_cast<int>(f_dyn.numel_in(0));
        nu = static_cast<int>(f_dyn.numel_in(1));
        if (f_dyn.n_in() > 2)
            np = static_cast<int>(f_dyn.numel_in(2));

        SX x = SX::sym("x", nx);
        SX u = SX::sym("u", nu);
        SX p = SX::sym("p", np);
        SX dt = SX::sym("dt");
        SX x_dot;
        if (np > 0)
            x_dot = f_dyn(SXVector{x, u, p})[0];
        else
            x_dot = f_dyn(SXVector{x, u})[0];

        dyn_fun = casadi::Function("dyn_numeric", {x, u, p}, {x_dot});

        SXDict ode = {{"x",   x},
                      {"p",   SX::vertcat({u, dt, p})},
                      {"ode", dt * x_dot}};
        Dict opts = {//{"tf",            1.0},    // Not supported in newer CVODES version
                     {"abstol",        Tolerance},
                     {"max_num_steps", MaxIter}};

        /** initialization of integration methods */
        Method = Parameters["method"];
        switch (Method)
        {
            case RK4:
                std::cout << "RK4 solver ";
                break;
            case CVODES:
                std::cout << "CVODES solver ";
                cvodes_integrator = casadi::integrator("CVODES_INT", "cvodes", ode, 0, dT, opts);
                break;
            default:
                std::cerr << "################## Unknown method: " << Method << " ##################\n";
                break;
        }
        if (use_default_params)
            std::cout << "with default parameters: " << Parameters << "\n";
        else
            std::cout << "with specified parameters: " << Parameters << "\n";

    }

    casadi::DM solve(const casadi::DM &x0, const casadi::DM &u, const casadi::DM &p, const double &dt)
    {
        casadi::DM solution;
        Method = Parameters["method"];
        dT = Parameters["tf"];
        switch (Method)
        {
            case RK4:
                solution = rk4_solve(x0, u, p, dt);
                break;
            case CVODES:
                solution = cvodes_solve(x0, u, p, dt);
                break;
            default:
                break;
        }

        return solution;
    }
    casadi::DM solve(const casadi::DM &x0, const casadi::DM &u, const double &dt)
    {
        casadi::DM p = casadi::DM::zeros(np); // dummy input
        return solve(x0, u, p, dt);
    }

    void updateParams(const casadi::Dict &params)
    {
        for (const auto &param: params)
        {
            if (Parameters.count(param.first) > 0)
                Parameters[param.first] = param.second;
            else
                std::cout << "Unknown parameter: " << param.first << "\n";
        }
    }
    casadi::Dict getParams() { return Parameters; }
    int dim_x() const { return nx; }
    int dim_u() const { return nu; }
    int dim_p() const { return np; }

private:
    /** right hand side of the ODE : f(x, u, p)*/
    casadi::Function dyn_fun;
    casadi::Dict Parameters;
    int nx{0}, nu{0}, np{0};

    /** numerical options */
    double dT;
    double Tolerance;
    int Method;
    int MaxIter;

    /** RK4 */
    casadi::DM rk4_solve(const casadi::DM &x0, const casadi::DM &u, const casadi::DM &p, const casadi::DM &dt)
    {
        using namespace casadi;

        DMVector res = dyn_fun(DMVector{x0, u, p});
        DM k1 = res[0];
        res = dyn_fun(DMVector{x0 + 0.5 * dt * k1, u, p});
        DM k2 = res[0];
        res = dyn_fun(DMVector{x0 + 0.5 * dt * k2, u, p});
        DM k3 = res[0];
        res = dyn_fun(DMVector{x0 + dt * k3, u, p});
        DM k4 = res[0];

        return x0 + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
    }

    /** CVODES */
    casadi::DM cvodes_solve(const casadi::DM &x0, const casadi::DM &u, const casadi::DM &p, const casadi::DM &dt)
    {
        using namespace casadi;

        DMDict out;
        try
        {
            DMDict args = {{"x0", x0},
                           {"p",  DM::vertcat({u, dt, p})}};
            out = cvodes_integrator(args);
        }
        catch (std::exception &e)
        {
            std::cout << "At  state x0: " << x0 << "\n";
            std::cout << "At control u: " << u << "\n";
            std::cout << "At   param p: " << p << "\n";
            std::cout << "At        dt: " << dt << "\n";
            std::cout << "CVODES exception " << e.what() << "\n";
        }

        return out["xf"];
    }
    casadi::Function cvodes_integrator;
};

} // namespace casadi_integration

#endif //SRC_CASADI_INTEGRATION_HPP
