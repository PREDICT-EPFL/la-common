#ifndef SRC_TRANSCRIBEDOCP_HPP
#define SRC_TRANSCRIBEDOCP_HPP

#include "OptiOcpBase.hpp"
#include "TranscriptionBase.hpp"

namespace mpc {

using namespace casadi;

template<typename Ocp, typename Transcription>
class TranscribedOcp : public Ocp, public Transcription
{
public:
    TranscribedOcp(const Function &f_cont, double tf, const Dict &control_objectives, const Dict &transcription_options)
            : Ocp(f_cont, tf)
    {
        Transcription::init(transcription_options);
        Ocp::init(control_objectives);
        Transcription::transcribe();
    }
};
}

#endif //SRC_TRANSCRIBEDOCP_HPP
