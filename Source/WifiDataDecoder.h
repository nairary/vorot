#pragma once

#include "WifiOfdm.h"

#include <cstdint>
#include <string_view>
#include <vector>

enum class WifiPhyFormat { LegacyOfdm, HtMixed };

struct WifiDataParameters
{
    WifiPhyFormat format;
    unsigned rate_or_mcs;
    size_t psdu_bytes;
    bool short_gi = false;
};

struct WifiDataResult
{
    std::vector<uint8_t> psdu;
    std::string_view error;
};

class WifiDataDecoder
{
public:
    // Includes the L-LTF and signaling preceding DATA. Null for an
    // unsupported rate/MCS, length, or guard interval.
    static std::optional<size_t> sampleCount(const WifiDataParameters& params);
    WifiDataResult decode(std::span<const wifi::IqSample> iq, double cfo,
                          const WifiDataParameters& params);
private:
    wifi::OfdmFft fft_;
};
