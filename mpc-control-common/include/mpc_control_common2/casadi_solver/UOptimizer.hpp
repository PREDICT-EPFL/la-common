#ifndef SRC_UOPTIMIZER_HPP
#define SRC_UOPTIMIZER_HPP

#include <memory>
#include "casadi/casadi.hpp"

template<typename Ocp, typename Transcription, typename Solver>
class UOptimizer
{
public:
    using DM = casadi::DM;

    UOptimizer(Ocp &ocp_, Transcription &transcription_) :
            ocp(ocp_),
            transcription(transcription_)
    {
        solver = transcription.transcribe(ocp);
    }

    bool solve()
    {
        transcription.update_parameters(ocp, solver);
        return solver->solve();
    }

    Ocp ocp;
    Transcription transcription;
    std::shared_ptr<Solver> solver;

    DM get_u0() const { return {}; }
    DM get_T_opt() const { return {}; }
    DM get_U_opt() const { return {}; }
    DM get_X_opt() const { return {}; }
    DM get_p_opt() const { return {}; }
    double get_cost() const { return {}; }


};

#endif //SRC_UOPTIMIZER_HPP
