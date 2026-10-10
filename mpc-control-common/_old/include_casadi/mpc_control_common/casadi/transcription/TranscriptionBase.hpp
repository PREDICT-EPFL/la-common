#ifndef SRC_TRANSCRIPTIONBASE_HPP
#define SRC_TRANSCRIPTIONBASE_HPP

#include "mpc_control_common/casadi/OptiOcpBase.hpp"
#include "casadi/casadi.hpp"

namespace transcription {

using namespace ocp_casadi;

class TranscriptionBase : public virtual OptiOcpBase
{
public:
    void transcribe()
    {
        setup_cost();
        setup_dyn_constraints();
    }

protected:
    void init_base()
    {
        create_problem_vars();
    }

    void create_problem_vars() { create_problem_vars_impl(); }
    void setup_cost() { setup_cost_impl(); }
    void setup_dyn_constraints() { setup_dyn_constraints_impl(); }

    static DM sample_and_hold(const DM &X, const DM &Tx, double dt)
    {
        /* Create new time grid */
        DM tend = Tx(Tx.size2() - 1);
        unsigned nTnew = std::ceil(tend.nonzeros()[0] / dt) + 1;
        DM Tnew = DM::zeros(1, nTnew);
        for (unsigned i = 0; i < (nTnew - 1); ++i) { Tnew(i + 1) = Tnew(i) + dt; }

        /* Sample and hold trajectory on new time grid */
        DM Xnew = DM::zeros(X.size1(), Tnew.size2());
        unsigned jOld = 0;
        for (unsigned iNew = 0; iNew < Xnew.size2(); ++iNew)
        {
            if (Tnew(iNew).nonzeros()[0] > Tx(jOld).nonzeros()[0] && jOld < (Tx.size2() - 1)) { ++jOld; }
            Xnew(Slice(), iNew) = X(Slice(), jOld);
        }
        return DM::vertcat({Tnew, Xnew});
    }

private:
    virtual void create_problem_vars_impl() { std::cout << "BaseClass\n"; }
    virtual void setup_cost_impl() { std::cout << "BaseClass\n"; }
    virtual void setup_dyn_constraints_impl() { std::cout << "BaseClass\n"; }
};

}

#endif //SRC_TRANSCRIPTIONBASE_HPP
