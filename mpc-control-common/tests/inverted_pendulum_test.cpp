#include "mpc_control_common/casadi_solver/UOptimizer.hpp"

#include "InvertedPendulumSimpleUOcp.hpp"
#include "mpc_control_common/casadi_transcription/MultipleShootingUOpti.hpp"
#include "mpc_control_common/casadi_solver/USolverOpti.hpp"

int main()
{
    InvertedPendulumSimpleUOcp uOcp;
    std::cout << uOcp.model << "\n";

    mpc_control_common::casadi_transcription::MultipleShootingUOpti uTranscription(10);

    UOptimizer<InvertedPendulumSimpleUOcp,
            mpc_control_common::casadi_transcription::MultipleShootingUOpti,
            mpc_control_common::casadi_solver::USolverOpti> optimizer(uOcp, uTranscription);

    optimizer.ocp.set_x0(casadi::DM::vertcat({1.5, 0.0}));

    optimizer.solve();

    casadi::DM X_opt = optimizer.solver->get_X_opt();
    std::cout << "X_opt: " << X_opt << "\n";

    return EXIT_SUCCESS;
}
