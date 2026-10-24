#ifndef SRC_DYNAMICSBASECASADI_HPP
#define SRC_DYNAMICSBASECASADI_HPP

#include <iostream>
#include <cstdlib>
#include <vector>

#include <casadi/casadi.hpp>
#include "model_common/common_utils.hpp"

namespace model_common {
namespace casadi_model {

class DynamicsBase
{
public:
    DynamicsBase() = default;

    // Recommended parameters to be handed to any child class
    explicit DynamicsBase(const std::string &filename,
                          const std::map<std::string, double> &static_params = {},
                          const std::vector<std::string> &const_params = {},
                          const std::vector<std::string> &opt_params = {})
    {
        (void) filename;
        (void) static_params;
        (void) const_params;
        (void) opt_params;
    }
    ~DynamicsBase() = default;

    casadi::DM getDefaultInitialState() const { return x_default; }

    casadi::Function getNumericDynamics(bool needed_inputs_only = false, bool show_model_setup = false)
    {
        if (output_mappings.find("dynamics") != output_mappings.end())
            output_mappings.at("dynamics") = this->StateDot;
        else
            output_mappings.insert({"dynamics", this->StateDot});

        casadi::Function numDynamics = getNumericMapping("dynamics", needed_inputs_only, show_model_setup);

        if (show_model_setup)
            std::cout << "dynamicsFunc: " << numDynamics << "\n";
        return numDynamics;
    }
    casadi::Function getNumericOutput(bool needed_inputs_only = false, bool show_model_setup = false)
    {
        if (output_mappings.find("output") != output_mappings.end())
            output_mappings.at("output") = this->Output;
        else
            output_mappings.insert({"output", this->Output});

        casadi::Function numOutput = getNumericMapping("output", needed_inputs_only, show_model_setup);

        if (show_model_setup)
            std::cout << "outputFunc: " << numOutput << "\n";
        return numOutput;
    }
    casadi::Function getNumericMapping(const std::string &output_name, bool needed_inputs_only = false,
                                       bool show_model_setup = true)
    {
        if (output_mappings.find(output_name) != output_mappings.end())
        {
            /* Output exists under this name, create output function for it */
            return getOutputFunction(output_name, output_mappings.at(output_name), needed_inputs_only,
                                     show_model_setup);
        }
        else
        {
            std::cout << "Output '" << output_name << "' could not be found." << "\n";
            std::cout << "Available outputs are:\n";
            for (const auto &output_mapping: output_mappings)
            {
                std::cout << output_mapping.first << "\n";
            }
            exit(EXIT_FAILURE);
        }
    }

    casadi::DM getTrimState()
    {
        if (x_trim.is_empty())
        {
            std::cout << "Warning: Need to do trim calculation first. Run trim() member function to specify "
                         "state and control bounds and initial guess. Running unbounded trim() with zeros guess "
                         "(only works if the dynamics function does not depend on parameters) ...\n";
            trim(casadi::SX(), casadi::DM());
        }
        return x_trim;
    }
    casadi::DM getTrimControl()
    {
        if (u_trim.is_empty())
        {
            std::cout << "Warning: Need to do trim calculation first. Run trim() member function to specify "
                         "state and control bounds and initial guess. Running unbounded trim() with zeros guess "
                         "(only works if the dynamics function does not depend on parameters) ...\n";
            trim(casadi::SX(), casadi::DM());
        }
        return u_trim;
    }
    void trim(const casadi::SX &dyn_params,
              casadi::DM xu0 = casadi::DM(),
              casadi::DM LBU = casadi::DM(), casadi::DM UBU = casadi::DM(),
              casadi::DM LBX = casadi::DM(), casadi::DM UBX = casadi::DM()) // By value because we need copies anyway
    {
        using namespace casadi;

        std::cout << "Running trim calculation ...\n";
        const int nx = this->State.size1();
        const int nu = this->Control.size1();
        const SX varx = SX::sym("varx", nx);
        const SX varu = SX::sym("varu", nu);

        Function dynamicsFunc = this->getNumericDynamics();
        DM opt_params_dummy;
        SX dyn = dynamicsFunc(SXVector({varx, varu, opt_params_dummy, dyn_params}))[0];

        SXDict problem;
        problem["x"] = SX::vertcat({varx, varu});
        problem["f"] = dot(dyn, dyn);

        Dict OPTS;
        OPTS["ipopt.linear_solver"] = "mumps";
        OPTS["ipopt.print_level"] = 0;
        OPTS["ipopt.tol"] = 1e-6;
        OPTS["ipopt.acceptable_tol"] = 1e-6;
        OPTS["ipopt.warm_start_init_point"] = "yes";
        const Function solver = nlpsol("solver", "ipopt", problem, OPTS);
//    const Function solver = qpsol("qp_solver", "qpoases", QP);

        const DM default_bound = DM::inf();
        if (xu0.is_empty()) xu0 = DM::zeros(nx + nu, 1);
        if (UBX.is_empty()) UBX = DM::ones(nx, 1) * default_bound;
        if (LBX.is_empty()) LBX = -DM::ones(nx, 1) * default_bound;
        if (UBU.is_empty()) UBU = DM::ones(nu, 1) * default_bound;
        if (LBU.is_empty()) LBU = -DM::ones(nu, 1) * default_bound;

        DMDict ARG;
        ARG["ubx"] = DM::vertcat({UBX, UBU});
        ARG["lbx"] = DM::vertcat({LBX, LBU});
        ARG["x0"] = xu0;

        DMDict NLP_res = solver(ARG);
        DM NLP_x_opt = NLP_res.at("x");

        this->x_trim = NLP_x_opt(Slice(0, nx));
        this->u_trim = NLP_x_opt(Slice(nx, nx + nu));
//    std::cout << "Trim state, control: " << x_trim << ", " << u_trim << "\n";
    }

    casadi::DM u_ubound;
    casadi::DM u_lbound;

    casadi::DM x_trim_ubound;
    casadi::DM x_trim_lbound;
    casadi::DM u_trim_lbound;
    casadi::DM u_trim_ubound;

    casadi::SX getSymbolicState() const { return this->State; }
    casadi::SX getSymbolicControl() const { return this->Control; }
    casadi::SX getSymbolicDynamics() const { return this->StateDot; }
    casadi::SX getSymbolicOptParameters() const { return this->OptParameters; }
    casadi::SX getSymbolicConstParameters() const { return this->ConstParameters; }

protected:
    casadi::DM x_trim;
    casadi::DM u_trim;
    casadi::DM x_default;

    casadi::SX State;
    casadi::SX Control;
    casadi::SX StateDot;
    casadi::SX Output;

    casadi::SX OptParameters;
    casadi::SX ConstParameters;
    casadi::SX SymStaticParameters;
    casadi::SX ValStaticParameters;

    /* Outputs */
    std::map<std::string, casadi::SX> output_mappings;
    casadi::Function getOutputFunction(const std::string &function_name,
                                       const casadi::SX &output,
                                       bool needed_inputs_only = false,
                                       bool show_model_setup = true)
    {
        if (show_model_setup && firstQuery)
        {
            firstQuery = false;
            std::cout << "Model setup:\n"
                      << "State       : " << this->State << "\n"
                      << "Control     : " << this->Control << "\n"
                      << "optParams   : " << this->OptParameters << "\n"
                      << "constParams : " << this->ConstParameters << "\n"
                      << "staticParams: " << this->SymStaticParameters << "\n";
        }

        using namespace casadi;

        /* Substitute static model parameters by their values */
        SX output_ = SX::substitute(output, SymStaticParameters, SX(ValStaticParameters));
        if (needed_inputs_only && OptParameters.is_empty() && ConstParameters.is_empty())
        {
            return Function(function_name, {State, Control}, {output_});
        }
        else if (needed_inputs_only && OptParameters.is_empty())
        {
            return Function(function_name, {State, Control, ConstParameters}, {output_});
        }
        else if (needed_inputs_only && ConstParameters.is_empty())
        {
            return Function(function_name, {State, Control, OptParameters}, {output_});
        }
        else
        {
            return Function(function_name, {State, Control, OptParameters, ConstParameters}, {output_});
        }
    }
    void sort_model_parameters(const casadi::SX &model_params,
                               const std::vector<std::string> &opt_params,
                               const std::vector<std::string> &const_params,
                               const std::map<std::string, double> &default_params,
                               const std::map<std::string, double> &static_params,
                               const std::string &config_filepath)
    {
        YAML::Node config;
        bool YAML_avail{false};
        try
        {
            config = YAML::LoadFile(config_filepath);
            YAML_avail = true;
        } catch (...)
        {
            std::cout << "Note: No valid YAML file given. (Filepath: " << config_filepath << ")\n";
        }

        /* Loop through parameters,
         * if they appear in opt_params list, make them optimizable,
         * if they appear in dyn_params list, make them dynamic, but not optimizable,
         * otherwise make them static and read them from default_params or config file. */
        double numerical_value{0};

        for (int i = 0; i < model_params.numel(); ++i)
        {
            std::string param_name = model_params->at(i).name();

            if (std::find(opt_params.begin(), opt_params.end(), param_name) != opt_params.end())
            {
                /* Parameters is optimizable */
                OptParameters = casadi::SX::vertcat({OptParameters, model_params->at(i)});
            }
            else if (std::find(const_params.begin(), const_params.end(), param_name) != const_params.end())
            {
                /* Parameter is const during optimization */
                ConstParameters = casadi::SX::vertcat({ConstParameters, model_params->at(i)});
            }
            else if (static_params.find(param_name) != static_params.end())
            {
                /* Static parameter is given by the user */
                SymStaticParameters = casadi::SX::vertcat({SymStaticParameters, model_params->at(i)});
                ValStaticParameters = casadi::DM::vertcat({ValStaticParameters, static_params.at(param_name)});
            }
            else if (YAML_avail && model_common::get_as_double(config, param_name, numerical_value))
            {
                /* Load parameter value from config file */
                SymStaticParameters = casadi::SX::vertcat({SymStaticParameters, model_params->at(i)});
                ValStaticParameters = casadi::DM::vertcat({ValStaticParameters, numerical_value});
            }
            else if (default_params.find(param_name) != default_params.end())
            {
                /* Default parameter value */
                SymStaticParameters = casadi::SX::vertcat({SymStaticParameters, model_params->at(i)});
                ValStaticParameters = casadi::DM::vertcat({ValStaticParameters, default_params.at(param_name)});
            }
            else
            {
                std::cerr << "Param " << param_name << " could not be loaded\n";
                exit(EXIT_FAILURE);
            }
        }
    }
private:
    bool firstQuery{true};
};

} //namespace casadi_model
} //namespace flight_model

#endif //SRC_DYNAMICSBASECASADI_HPP
