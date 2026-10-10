#ifndef SRC_MODEL_COMMON_UTILS_HPP
#define SRC_MODEL_COMMON_UTILS_HPP

#include <iostream>
#include <cassert>
#include <cmath>

#include <yaml-cpp/yaml.h>

namespace model_common {

inline double get_value(const YAML::Node &node, const std::string &name)
{
    double property_value = 0.0;
    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
    {
        std::string key = it->first.as<std::string>();
        switch (it->second.Type())
        {
            case YAML::NodeType::Scalar:
                if (key == name)
                {
                    property_value = it->second.as<double>();
                    return property_value;
                }
                break;
            case YAML::NodeType::Map:
                /** iterate over the map */
                for (auto iter = it->second.begin(); iter != it->second.end(); ++iter)
                {
                    if ((iter->first).as<std::string>() == name)
                    {
                        property_value = iter->second.as<double>();
                        return property_value;
                    }
                }
                break;
            case YAML::NodeType::Null:
                std::cout << "real lox \n";
                break;
            default :
                std::cout << "lox \n";
                break;
        }
    }
    std::cerr << "Warning: YAML does not contain " << name << "\n";
//    assert(false);
    return property_value;
}

inline bool get_as_double(const YAML::Node &node, const std::string &name, double &value_write)
{
    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
    {
        std::string key = it->first.as<std::string>();
        switch (it->second.Type())
        {
            case YAML::NodeType::Scalar:
                if (key == name)
                {
                    value_write = it->second.as<double>();
                    return true;
                }
                break;
            case YAML::NodeType::Map:
                /** iterate over the map */
                for (auto iter = it->second.begin(); iter != it->second.end(); ++iter)
                {
                    if ((iter->first).as<std::string>() == name)
                    {
                        value_write = iter->second.as<double>();
                        return true;
                    }
                }
                break;
            case YAML::NodeType::Null:
                std::cout << "real lox \n";
                break;
            default :
                std::cout << "lox \n";
                break;
        }
    }
    return false; // Success / modified
}

} // namespace model_common

#endif //SRC_MODEL_COMMON_UTILS_HPP
