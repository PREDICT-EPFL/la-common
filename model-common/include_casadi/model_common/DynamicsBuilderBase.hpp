#ifndef SRC_DYNAMICSBUILDERBASE_HPP
#define SRC_DYNAMICSBUILDERBASE_HPP

#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <algorithm>
#include <cstdlib>

#include <casadi/casadi.hpp>
#include <yaml-cpp/yaml.h>
#include "model_common/common_utils.hpp"

namespace model_common {
namespace casadi_model {

class DynamicsBuilderBase
{
public:
    virtual ~DynamicsBuilderBase() = default;

    casadi::Function get_dynamics_function(const std::string &yaml_filepath = "",
                                           const std::map<std::string, double> &static_params = {},
                                           const std::vector<std::string> &const_params = {},
                                           const std::vector<std::string> &opt_params = {},
                                           bool minimal_interface = false,
                                           bool print_function_signature = false) const
    {
        return build_function("dynamics", sym_.x_dot, yaml_filepath, static_params, const_params, opt_params,
                              minimal_interface, print_function_signature);
    }

    casadi::Function get_output_function(const std::string &yaml_filepath = "",
                                         const std::map<std::string, double> &static_params = {},
                                         const std::vector<std::string> &const_params = {},
                                         const std::vector<std::string> &opt_params = {},
                                         bool minimal_interface = false,
                                         bool print_function_signature = false) const
    {
        return build_function("output", sym_.y, yaml_filepath, static_params, const_params, opt_params,
                              minimal_interface, print_function_signature);
    }

    casadi::Function get_mapping_function(const std::string &map_name,
                                          const std::string &yaml_filepath = "",
                                          const std::map<std::string, double> &static_params = {},
                                          const std::vector<std::string> &const_params = {},
                                          const std::vector<std::string> &opt_params = {},
                                          bool minimal_interface = false,
                                          bool print_function_signature = false) const
    {
        if (output_mappings_.find(map_name) != output_mappings_.end())
        {
            /* Output exists under this name, create function for it */
            return build_function(map_name, output_mappings_.at(map_name), yaml_filepath, static_params,
                                  const_params, opt_params, minimal_interface, print_function_signature);
        }
        else
        {
            std::cout << "Output '" << map_name << "' could not be found." << "\n";
            std::cout << "Available outputs are:\n";
            for (const auto &output_mapping: output_mappings_)
            {
                std::cout << output_mapping.first << "\n";
            }
            exit(EXIT_FAILURE);
        }
    }

    std::map<std::string, double> get_param_values(const std::string &yaml_filepath = "",
                                                   const std::map<std::string, double> &static_params = {}) const
    {
        /*
         * Numeric value of every model parameter (name in sym_.p) outside the graph: from static_params, otherwise
         * from the YAML file (if one is given). Parameters that are found in neither are left out.
         */
        YAML::Node config;
        bool yaml_loaded = false;
        if (!yaml_filepath.empty())
        {
            try
            {
                config = YAML::LoadFile(yaml_filepath);
                yaml_loaded = true;
            }
            catch (const YAML::Exception &e)
            {
                std::cerr << "Could not load YAML file " << yaml_filepath << ": " << e.what() << "\n";
            }
        }

        std::map<std::string, double> values;
        for (int i = 0; i < sym_.p.numel(); ++i)
        {
            std::string name = sym_.p->at(i).name();
            double value = 0;
            auto static_it = static_params.find(name);
            if (static_it != static_params.end())
            {
                values[name] = static_it->second;
            }
            else if (yaml_loaded && get_as_double(config, name, value))
            {
                values[name] = value;
            }
        }
        return values;
    }

protected:
    struct SymbolicExpressions
    {
        casadi::SX x, u, p, x_dot, y;
    } sym_;

    std::map<std::string, casadi::SX> output_mappings_;

    casadi::Function build_function(const std::string &name,
                                    const casadi::SX &sym_return,
                                    const std::string &yaml_filepath = "",
                                    const std::map<std::string, double> &static_params = {},
                                    const std::vector<std::string> &const_params = {},
                                    const std::vector<std::string> &opt_params = {},
                                    bool minimal_interface = false,
                                    bool print_function_signature = false) const
    {
        if (yaml_filepath.empty() && static_params.empty() && const_params.empty() && opt_params.empty())
        {
            return casadi::Function(name, {sym_.x, sym_.u, sym_.p}, {sym_return});
        }

        /*
         * Parameters are identified by name only. const_params / opt_params stay symbolic (in the order given),
         * every other parameter is substituted into sym_return as a number: static_params take precedence over the
         * YAML file (if one is given). Parameters that are neither in static_params nor in the YAML file are
         * appended to the const parameters (in the order of sym_.p).
         */
        YAML::Node config;
        bool yaml_loaded = false;
        if (!yaml_filepath.empty())
        {
            try
            {
                config = YAML::LoadFile(yaml_filepath);
                yaml_loaded = true;
            }
            catch (const YAML::Exception &e)
            {
                std::cerr << "Could not load YAML file " << yaml_filepath << ": " << e.what() << "\n";
            }
        }

        /* Model parameters by name */
        std::map<std::string, casadi::SX> p_by_name;
        for (int i = 0; i < sym_.p.numel(); ++i) { p_by_name[sym_.p->at(i).name()] = sym_.p->at(i); }

        std::vector<std::string> params_not_found;
        std::vector<std::string> params_ambiguous;
        std::vector<std::string> params_not_set;

        /* Collect opt and const params */
        std::vector<casadi::SX> p_opt_vec;
        std::vector<casadi::SX> p_const_vec;
        for (const std::string &name: opt_params)
        {
            if (p_by_name.find(name) == p_by_name.end())
            {
                params_not_found.push_back(name);
                continue;
            }
            p_opt_vec.push_back(p_by_name[name]);
        }
        for (const std::string &name: const_params)
        {
            if (p_by_name.find(name) == p_by_name.end())
            {
                params_not_found.push_back(name);
                continue;
            }
            p_const_vec.push_back(p_by_name[name]);
        }

        /* Static params that are not model parameters */
        for (const auto &kv: static_params)
        {
            if (p_by_name.find(kv.first) == p_by_name.end())
            {
                params_not_found.push_back(kv.first);
            }
        }

        /* Collect params that are (wrongly) in two categories */
        for (const std::string &name: const_params)
        {
            if (std::find(opt_params.begin(), opt_params.end(), name) != opt_params.end())
            {
                params_ambiguous.push_back(name);
            }
        }
        for (const auto &kv: static_params)
        {
            if (std::find(opt_params.begin(), opt_params.end(), kv.first) != opt_params.end() ||
                std::find(const_params.begin(), const_params.end(), kv.first) != const_params.end())
            {
                params_ambiguous.push_back(kv.first);
            }
        }

        /* Static parameters: values from static_params, otherwise from the YAML file */
        std::vector<casadi::SX> static_sym;
        std::vector<double> static_val;
        for (int i = 0; i < sym_.p.numel(); ++i)
        {
            std::string name = sym_.p->at(i).name();
            if (std::find(opt_params.begin(), opt_params.end(), name) != opt_params.end() ||
                std::find(const_params.begin(), const_params.end(), name) != const_params.end()) { continue; }

            double value = 0;
            auto static_it = static_params.find(name);
            if (static_it != static_params.end())
            {
                value = static_it->second;
            }
            else if (!yaml_loaded || !get_as_double(config, name, value))
            {
                params_not_set.push_back(name);
                continue;
            }

            static_sym.push_back(sym_.p->at(i));
            static_val.push_back(value);
        }

        /* Throw errors if needed */
        if (!params_not_found.empty())
        {
            for (const std::string &name: params_not_found)
            {
                std::cerr << "Parameter '" << name << "' does not occur in the model\n";
            }
        }
        if (!params_ambiguous.empty())
        {
            for (const std::string &name: params_ambiguous)
            {
                std::cerr << "Parameter '" << name << "' appears in more than one of {static, const, opt}_params\n";
            }
        }
        if (!params_not_found.empty() || !params_ambiguous.empty()) { exit(EXIT_FAILURE); }
        if (!params_not_set.empty())
        {
            /* Params are not categorized/not available from YAML --> Add them to const params and tell the user. */
            std::cout << "Note: the following parameters are neither in static_params nor in the YAML file and were "
                         "appended to ConstParam:\n";
            for (const std::string &name: params_not_set)
            {
                std::cout << "  " << name << " (ConstParam index " << p_const_vec.size() << ")\n";
                p_const_vec.push_back(p_by_name[name]);
            }
        }

        casadi::SX p_opt = casadi::SX::vertcat(p_opt_vec);
        casadi::SX p_const = casadi::SX::vertcat(p_const_vec);
        casadi::SX sym_return_ = casadi::SX::substitute(sym_return, casadi::SX::vertcat(static_sym), casadi::SX(static_val));

        casadi::Function return_fn;
        if (minimal_interface && p_opt_vec.empty() && p_const_vec.empty())
        {
            return_fn = casadi::Function(name, {sym_.x, sym_.u}, {sym_return_});
        }
        else if (minimal_interface && p_opt_vec.empty())
        {
            return_fn = casadi::Function(name, {sym_.x, sym_.u, p_const}, {sym_return_});
        }
        else if (minimal_interface && p_const_vec.empty())
        {
            return_fn = casadi::Function(name, {sym_.x, sym_.u, p_opt}, {sym_return_});
        }
        else
        {
            return_fn = casadi::Function(name, {sym_.x, sym_.u, p_opt, p_const}, {sym_return_});
        }

        if (print_function_signature)
        {
            std::cout << "Function " << return_fn << ":\n"
                      << "State       : " << sym_.x << "\n"
                      << "Input       : " << sym_.u << "\n"
                      << "OptParam    : " << p_opt_vec << "\n"
                      << "ConstParam  : " << p_const_vec << "\n"
                      << "StaticParam : " << static_sym << "\n";
        }
        return return_fn;
    }
};

} // namespace casadi_model
} // namespace model_common

#endif //SRC_DYNAMICSBUILDERBASE_HPP
