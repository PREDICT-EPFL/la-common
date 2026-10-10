#ifndef SRC_PARAMSBASEEIGEN_HPP
#define SRC_PARAMSBASEEIGEN_HPP

#include <iostream>
#include <vector>

#include "model_common/common_utils.hpp"

namespace model_common {
namespace eigen_model {

template<typename param_t = double>
struct ParamsBase
{
public:
    using StaticParam = param_t;
    virtual void load_params_from_yaml(const std::string &yaml_filepath)
    {
        (void) yaml_filepath;
        std::cout << "load_params_from_yaml(): Implement me!" << "\n";
    }
};

} //namespace eigen_model
} //namespace model_common

#endif //SRC_PARAMSBASEEIGEN_HPP
