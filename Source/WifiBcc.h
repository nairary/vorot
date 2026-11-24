#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace wifi {

enum class BccRate { Half, TwoThirds, ThreeQuarters, FiveSixths };

// Positive soft bits favor 0. Traceback ends at the zero state reached by TAIL.
// The input may also contain the coded PAD bits that follow TAIL.
std::vector<uint8_t> decodeBcc(std::span<const double> soft, size_t bits, BccRate rate);
void deinterleave(std::span<const double> input, std::span<double> output,
                  size_t bits_per_carrier, size_t columns);

// SERVICE has seven known zero bits, which reveal the scrambler state.
// Returns an empty vector if SERVICE cannot be recovered.
std::vector<uint8_t> descramblePsdu(std::span<const uint8_t> bits, size_t length);

constexpr uint8_t scramblerBit(unsigned& state)
{
    const unsigned bit = ((state >> 6) ^ (state >> 3)) & 1u;
    state = ((state << 1) | bit) & 0x7fu;
    return static_cast<uint8_t>(bit);
}

}
