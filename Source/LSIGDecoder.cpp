#include "LSIGDecoder.h"
#include "WifiBcc.h"

#include <algorithm>

LSigResult LSIGDecoder::decode(std::span<const wifi::IqSample> iq, double total_cfo_hz, double sample_rate_hz)
{
    LSigResult result;
    if (iq.size() < wifi::legacy_data_start)
    {
        result.error = "Need 208 continuous samples from LTF1";
        return result;
    }
    if (!std::isfinite(total_cfo_hz) || !std::isfinite(sample_rate_hz) ||
        std::abs(sample_rate_hz - wifi::sample_rate_hz) > 20)
    {
        result.error = "Invalid CFO or unsupported sampling rate";
        return result;
    }

    iq = iq.first(wifi::legacy_data_start);
    if (!std::ranges::all_of(iq, [](auto x) { return std::isfinite(x.real()) && std::isfinite(x.imag()); }))
    {
        result.error = "Non-finite IQ sample";
        return result;
    }

    const auto channel = fft_.legacyChannel(iq, total_cfo_hz, sample_rate_hz);
    if (!wifi::hasChannelEnergy(channel))
    {
        result.error = "L-LTF channel estimate has zero energy";
        return result;
    }
    const auto signal = fft_.symbol(iq, 2 * wifi::fft_size, wifi::long_guard, total_cfo_hz, sample_rate_hz);
    const auto correction = wifi::pilotCorrection(signal, channel, wifi::pilotValues(0));
    if (!correction)
    {
        result.error = "Cannot estimate SIGNAL pilot phase";
        return result;
    }

    result.pilot_phase_rad = -std::arg(*correction);
    std::array<double, 48> interleaved{};
    size_t position = 0;
    for (int k = -26; k <= 26; ++k)
    {
        if (k == 0 || wifi::isPilot(k))
            continue;
        const auto b = wifi::bin(k);
        // The IEEE constellation maps bit 0 to -1 and bit 1 to +1.
        interleaved[position++] = -(signal[b] * std::conj(channel[b]) * *correction).real();
    }

    std::array<double, 48> soft{};
    wifi::deinterleave(interleaved, soft, 1, 16);
    std::ranges::copy(wifi::decodeBcc(soft, result.bits.size(), wifi::BccRate::Half), result.bits.begin());

    const unsigned rate_field = (unsigned(result.bits[0]) << 3) | (unsigned(result.bits[1]) << 2) |
                               (unsigned(result.bits[2]) << 1) | unsigned(result.bits[3]);
    struct Rate { unsigned field, mbps; const char* parameters; };
    constexpr Rate rates[] = {
        {0xd, 6, "BPSK, coding 1/2"}, {0xf, 9, "BPSK, coding 3/4"},
        {0x5, 12, "QPSK, coding 1/2"}, {0x7, 18, "QPSK, coding 3/4"},
        {0x9, 24, "16-QAM, coding 1/2"}, {0xb, 36, "16-QAM, coding 3/4"},
        {0x1, 48, "64-QAM, coding 2/3"}, {0x3, 54, "64-QAM, coding 3/4"}
    };
    const auto rate = std::ranges::find(rates, rate_field, &Rate::field);
    if (rate == std::end(rates))
    {
        result.error = "Invalid L-SIG RATE";
        return result;
    }
    result.rate_mbps = rate->mbps;

    if (result.bits[4] != 0)
    {
        result.error = "L-SIG reserved bit is not zero";
        return result;
    }
    for (unsigned i = 0; i < 12; ++i)
        result.length_bytes |= unsigned(result.bits[5 + i]) << i;

    unsigned parity = 0;
    for (size_t i = 0; i <= 17; ++i)
        parity ^= result.bits[i];
    if (parity != 0)
    {
        result.error = "L-SIG parity failed";
        return result;
    }

    result.valid = true;
    result.channel_parameters = rate->parameters;
    return result;
}
