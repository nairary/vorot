#include "WifiBcc.h"

#include <algorithm>
#include <array>
#include <bit>
#include <limits>

namespace wifi {

std::vector<uint8_t> decodeBcc(std::span<const double> soft, size_t bits, BccRate rate)
{
    // X and Y are emitted in that order for each input bit.
    constexpr bool half[] = {true, true};
    constexpr bool two_thirds[] = {true, true, true, false};
    constexpr bool three_quarters[] = {true, true, true, false, false, true};
    constexpr bool five_sixths[] = {true, true, true, false, false, true, true, false, false, true};

    std::span<const bool> puncture = half;
    switch (rate)
    {
    case BccRate::Half: break;
    case BccRate::TwoThirds: puncture = two_thirds; break;
    case BccRate::ThreeQuarters: puncture = three_quarters; break;
    case BccRate::FiveSixths: puncture = five_sixths; break;
    }

    const size_t coded_bits = 2 * bits;
    const size_t period = puncture.size();
    const size_t ones = std::ranges::count(puncture, true);
    if (soft.size() < coded_bits / period * ones +
        static_cast<size_t>(std::ranges::count(puncture.first(coded_bits % period), true)))
        return {};

    // With the newest bit in the LSB, the (133,171) octal generators
    // are reflected. The other predecessor flips both output bits.
    static constexpr auto lower_labels = [] {
        std::array<unsigned, 64> labels{};
        for (unsigned state = 0; state < labels.size(); ++state)
            labels[state] = (std::popcount(state & 0155u) & 1) |
                            ((std::popcount(state & 0117u) & 1) << 1);
        return labels;
    }();

    std::array<double, 64> previous;
    previous.fill(-std::numeric_limits<double>::infinity());
    previous[0] = 0;
    std::vector<uint64_t> survivors(bits);
    size_t position = 0, puncture_index = 0;
    auto observation = [&] {
        const double value = puncture[puncture_index] ? soft[position++] : 0.0;
        puncture_index = (puncture_index + 1) % period;
        return value;
    };

    for (size_t t = 0; t < bits; ++t)
    {
        const double x = observation();
        const double y = observation();
        const double branches[]{x + y, -x + y, x - y, -x - y};
        std::array<double, 64> next;
        uint64_t survivor_mask = 0;
        for (unsigned state = 0; state < next.size(); ++state)
        {
            const double branch = branches[lower_labels[state]];
            const unsigned lower = state >> 1;
            const double a = previous[lower] + branch;
            const double b = previous[lower + 32] - branch;
            next[state] = a;
            if (b > a)
            {
                next[state] = b;
                survivor_mask |= uint64_t{1} << state;
            }
        }
        previous = next;
        survivors[t] = survivor_mask;
    }

    std::vector<uint8_t> decoded(bits);
    unsigned state = 0;
    for (size_t t = bits; t > 0; --t)
    {
        decoded[t - 1] = static_cast<uint8_t>(state & 1u);
        const auto upper = static_cast<unsigned>((survivors[t - 1] >> state) & 1u);
        state = (state >> 1) | (upper << 5);
    }
    return decoded;
}

void deinterleave(std::span<const double> input, std::span<double> output,
                  size_t bits_per_carrier, size_t columns)
{
    const size_t rows = input.size() / columns;
    const size_t s = std::max<size_t>(1, bits_per_carrier / 2);
    for (size_t k = 0; k < input.size(); ++k)
    {
        const size_t i = rows * (k % columns) + k / columns;
        const size_t j = s * (i / s) + (i + input.size() - i / rows) % s;
        output[k] = input[j];
    }
}

std::vector<uint8_t> descramblePsdu(std::span<const uint8_t> bits, size_t length)
{
    constexpr size_t service_bits = 16;
    if (bits.size() < service_bits || length > (bits.size() - service_bits) / 8)
        return {};

    unsigned state = 0;
    for (size_t i = 0; i < 7; ++i)
        state = (state << 1) | bits[i];
    if (state == 0)
        return {};

    for (size_t i = 7; i < service_bits; ++i)
        if (bits[i] != scramblerBit(state))
            return {};

    std::vector<uint8_t> psdu(length);
    for (size_t byte = 0; byte < length; ++byte)
    {
        for (unsigned bit = 0; bit < 8; ++bit)
        {
            const unsigned value = bits[service_bits + byte * 8 + bit] ^ scramblerBit(state);
            psdu[byte] |= static_cast<uint8_t>(value << bit);
        }
    }
    return psdu;
}

}
