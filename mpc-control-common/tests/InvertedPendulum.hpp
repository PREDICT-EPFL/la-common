#ifndef SRC_INVERTEDPENDULUM_HPP
#define SRC_INVERTEDPENDULUM_HPP

#include "model_common/DynamicsBaseCasadi.hpp"

class InvertedPendulum : public model_common::casadi_model::DynamicsBase
{
public:
    InvertedPendulum(const std::string &filename,
                     const std::map<std::string, double> &static_params = {},
                     const std::vector<std::string> &dyn_params = {},
                     const std::vector<std::string> &opt_params = {}) :
            model_common::casadi_model::DynamicsBase()
    {
        std::cout << "InvertedPendulum\n";
        using casadi::SX;

        /** ============================================================================================================ **/
        /** Model parameters **/
        SX model_params;
        std::map<std::string, double> default_params;

        const double g = 9.806;   // gravitational acceleration [m/s2] [WGS84]
        const double l = 0.5;
        const double m = 0.15;
        const double b = 0.1;

        /** ============================================================================================================ **/
        /** Start of model **/
        SX debugSX;

        /** State variables **/
        SX theta = SX::sym("theta");
        SX theta_dot = SX::sym("theta_dot");

        /** Control variables **/
        SX T = SX::sym("T");

        SX dynamics = SX::vertcat({theta_dot,
                                   (m * g * l * sin(theta) - b * theta_dot + T) / (m * l * l)});

        /** End of model **/
        /** ============================================================================================================ **/

        /** Categorize parameters **/
        sort_model_parameters(model_params, opt_params, dyn_params, default_params, static_params, filename);

        /** State, Dynamics, Control **/
        SX state = SX::vertcat({theta, theta_dot});
        SX input = SX::vertcat({T});

        this->State = state;
        this->Control = input;
        this->StateDot = dynamics;
        this->Output = state;

        /** Output mappings **/
        /* Full output (all in one vector) NOTE THAT debugSX only participates in vector if defined
         * anywhere else than initial declaration */
        SX dyn_out = SX::vertcat({debugSX});

        output_mappings.insert({
                                       {"debugSX", debugSX}
                               });
    }

};

#endif //SRC_INVERTEDPENDULUM_HPP
