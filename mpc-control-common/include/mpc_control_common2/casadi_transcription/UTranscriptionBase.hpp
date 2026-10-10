#ifndef SRC_UTRANSCRIPTIONBASE_HPP
#define SRC_UTRANSCRIPTIONBASE_HPP

#include "casadi/casadi.hpp"

namespace mpc_control_common2 {
namespace casadi_transcription {

class UTranscriptionBase
{
public:
    template<class UOcp>
    void update_parameters(UOcp &ocp)
    {
        std::cout << "update_parameters(ocp) base\n";
        double test =1 ;
    }

protected:
    template<typename Ocp, typename sym_t>
    sym_t get_user_constraints(const Ocp &ocp, const casadi::DM &T, const sym_t &X_sym, const sym_t &U_sym)
    {
        sym_t constr;
// #TODO: Unfinished
        return constr;
    }

};

}
}

#endif //SRC_UTRANSCRIPTIONBASE_HPP
