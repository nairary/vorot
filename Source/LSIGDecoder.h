#pragma once

#include "WifiOfdm.h"

#include <cstdint>
#include <string>

struct LSigResult
{
    bool valid = false;
    unsigned rate_mbps = 0;
    unsigned length_bytes = 0;
    std::array<uint8_t, 24> bits{};
    double pilot_phase_rad = 0;
    std::string channel_parameters;
    std::string error;
};

class LSIGDecoder
{
public:
    // iq[0] is the first useful sample of the first L-LTF symbol.
    LSigResult decode(std::span<const wifi::IqSample> iq, double total_cfo_hz,
                      double sample_rate_hz = wifi::sample_rate_hz);
private:
    wifi::OfdmFft fft_;
};
