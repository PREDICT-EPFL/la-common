#ifndef SRC_PARAMINTERFACEBASE_HPP
#define SRC_PARAMINTERFACEBASE_HPP

#include <string>

class ParamInterfaceBase
{
public:
    /** To check if there is an update **/
    virtual bool needs_update() { return false; }

    /** To inform an update **/
    virtual bool needs_update(bool update) { return false; }

    virtual bool get_param(const std::string &param_name, double& param) { return false; }

    virtual bool set_param(const std::string &param_name, const double& param) { return false; }
};

#endif //SRC_PARAMINTERFACEBASE_HPP
