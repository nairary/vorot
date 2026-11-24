#pragma once

#include "WifiOfdm.h"

#include <cstdint>
#include <string>

struct HtSigResult
{
    bool valid = false;
    bool supported_siso_ht20 = false;
    unsigned mcs = 0;
    unsigned length_bytes = 0;
    unsigned stbc = 0;
    unsigned extension_streams = 0;
    bool cbw40 = false;
    bool smoothing = false;
    bool not_sounding = false;
    bool aggregation = false;
    bool ldpc = false;
    bool short_gi = false;
    std::array<uint8_t, 48> bits{};
    std::string error;
};

struct HtSigEvent
{
    uint64_t segment = 0;
    uint64_t ltf_first_index = 0;
    HtSigResult signal;
};

enum class OfdmSignalAxis { InPhase, Quadrature, Unclear };

class HtSigDecoder
{
public:
    // Use only after an L-SIG compatible with HT-mixed format. An in-phase
    // first symbol selects the legacy DATA path without decoding HT-SIG.
    OfdmSignalAxis firstSymbolAxis(std::span<const wifi::IqSample> iq, double cfo);
    HtSigResult decode(std::span<const wifi::IqSample> iq, double cfo);

private:
    wifi::OfdmFft fft_;
};
