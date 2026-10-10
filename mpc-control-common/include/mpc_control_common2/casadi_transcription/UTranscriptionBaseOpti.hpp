#ifndef SRC_UTRANSCRIPTIONBASEOPTI_HPP
#define SRC_UTRANSCRIPTIONBASEOPTI_HPP

#include "casadi/casadi.hpp"
#include "UTranscriptionBase.hpp"
#include "mpc_control_common2/casadi_solver/USolverOpti.hpp"

namespace mpc_control_common2 {
namespace casadi_transcription {

class UTranscriptionBaseOpti : public UTranscriptionBase
{
public:
    // Solver specific
    template<class UOcp, class SolverPtr>
    void update_parameters(UOcp &ocp, SolverPtr solver)
    {
        using casadi::MX;

        // Loop through parameter/bounds structs and update values in solver parameters

        // Update OCP bounds
        solver->set_value(ocp.template sym<MX>().u_lb, replace_inf(ocp.u_lb));
        solver->set_value(ocp.template sym<MX>().u_ub, replace_inf(ocp.u_ub));
        solver->set_value(ocp.template sym<MX>().x_lb, replace_inf(ocp.x_lb));
        solver->set_value(ocp.template sym<MX>().x_ub, replace_inf(ocp.x_ub));

        solver->set_value(ocp.template sym<MX>().x0_lb, replace_inf(ocp.x0_lb));
        solver->set_value(ocp.template sym<MX>().x0_ub, replace_inf(ocp.x0_ub));
        solver->set_value(ocp.template sym<MX>().xf_lb, replace_inf(ocp.xf_lb));
        solver->set_value(ocp.template sym<MX>().xf_ub, replace_inf(ocp.xf_ub));

        solver->set_value(ocp.template sym<MX>().tf_lb, replace_inf(ocp.tf_lb));
        solver->set_value(ocp.template sym<MX>().tf_ub, replace_inf(ocp.tf_ub));

        // Update opt_param bounds
        if (!ocp.template sym<MX>().opt_params.empty())
        {
            const std::vector<std::string> names_lb = get_key_names(ocp.opt_params_lb);
            for (const auto &name: names_lb)
            {
                solver->set_value(ocp.template sym<MX>().opt_params_lb.at(name),
                                  replace_inf(ocp.opt_params_lb.at(name)));
            }

            const std::vector<std::string> names_ub = get_key_names(ocp.opt_params_ub);
            for (const auto &name: names_ub)
            {
                solver->set_value(ocp.template sym<MX>().opt_params_ub.at(name),
                                  replace_inf(ocp.opt_params_ub.at(name)));
            }
        }
        // Update OCP parameters
        if (!ocp.params.empty())
        {
            const std::vector<std::string> names = get_key_names(ocp.params);
            for (const auto &name: names)
            {
                solver->set_value(ocp.template sym<MX>().params.at(name),
                                  replace_inf(ocp.params.at(name)));
            }
        }
    }

protected:
    enum CreateType { Variable, Parameter };
    static std::map<std::string, casadi::MX> struct2sym(casadi::Opti &opti, CreateType type, const casadi::DMDict &dict)
    {
        std::map<std::string, casadi::MX> out_map{};
        if (type == CreateType::Parameter)
        {
            for (const auto &var: dict)
            {
                out_map.insert({var.first, opti.parameter(var.second.size1(), var.second.size2())});
            }
        }
        else /* Variable */
        {
            for (const auto &var: dict)
            {
                out_map.insert({var.first, opti.variable(var.second.size1(), var.second.size2())});
            }
        }
        return out_map;
    }

    static casadi::MX sym2vec(const std::map<std::string, casadi::MX> &sym_map)
    {
        casadi::MX sym_vec{};
        for (const std::pair<const std::string, casadi::MX> &sym: sym_map)
        {
            sym_vec = casadi::MX::veccat({sym_vec, sym.second});
        }
        return sym_vec;
    }

private:
    static casadi::DM replace_inf(casadi::DM mat)
    {
        const double Inff = 9999999;
        for (auto &elem: mat.nonzeros())
        {
            if (std::abs(elem) == casadi::inf)
            {
                elem = Inff * casadi::sign(elem);
            }
        }
        return mat;
    }

    template<typename T>
    std::vector<std::string> get_key_names(const std::map<std::string, T> &map)
    {
        std::vector<std::string> names;
        for (const auto &elem : map)
        {
            names.template emplace_back(elem.first);
        }
        return names;
    }
};

}
}

#endif //SRC_UTRANSCRIPTIONBASEOPTI_HPP
