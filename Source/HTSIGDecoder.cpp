#include "HTSIGDecoder.h"
#include "WifiBcc.h"

#include <algorithm>

namespace {

struct SignalSymbol
{
    OfdmSignalAxis axis = OfdmSignalAxis::Unclear;
    std::array<double, 48> soft{};
};

SignalSymbol readSignalSymbol(wifi::OfdmFft& fft, std::span<const wifi::IqSample> iq,
                              double cfo, const wifi::Spectrum& channel, size_t symbol)
{
    const auto signal = fft.symbol(iq, wifi::legacy_data_start + symbol * wifi::symbol_samples,
                                   wifi::long_guard, cfo);
    // HT-SIG data tones are rotated by 90 degrees. Its pilots are real.
    const auto correction = wifi::pilotCorrection(signal, channel, wifi::pilotValues(symbol + 1));
    SignalSymbol result;
    if (!correction)
        return result;

    double in_phase = 0;
    double quadrature = 0;
    size_t position = 0;
    for (int k = -26; k <= 26; ++k)
    {
        if (k == 0 || wifi::isPilot(k))
            continue;
        const auto b = wifi::bin(k);
        const auto value = signal[b] * std::conj(channel[b]) * *correction;
        in_phase += value.real() * value.real();
        quadrature += value.imag() * value.imag();
        // IEEE BPSK, rotated by +90 degrees: bit 0 is -j.
        result.soft[position++] = -value.imag();
    }
    if (quadrature > 2 * in_phase)
        result.axis = OfdmSignalAxis::Quadrature;
    else if (in_phase > 2 * quadrature)
        result.axis = OfdmSignalAxis::InPhase;
    return result;
}

bool crcOk(const std::array<uint8_t, 48>& bits)
{
    // HT-SIG CRC-8 covers the first 34 bits. The complemented register
    // is transmitted most-significant bit first.
    uint8_t crc = 0xff;
    for (size_t i = 0; i < 34; ++i)
    {
        const auto feedback = ((crc >> 7) ^ bits[i]) & 1u;
        crc = static_cast<uint8_t>(crc << 1);
        if (feedback)
            crc ^= 0x07;
    }
    const auto expected = static_cast<uint8_t>(~crc);
    for (unsigned i = 0; i < 8; ++i)
        if (bits[34 + i] != ((expected >> (7 - i)) & 1u))
            return false;
    return true;
}

}

OfdmSignalAxis HtSigDecoder::firstSymbolAxis(std::span<const wifi::IqSample> iq, double cfo)
{
    if (iq.size() < wifi::legacy_data_start + wifi::symbol_samples || !std::isfinite(cfo))
        return OfdmSignalAxis::Unclear;
    const auto channel = fft_.legacyChannel(iq, cfo);
    return readSignalSymbol(fft_, iq, cfo, channel, 0).axis;
}

HtSigResult HtSigDecoder::decode(std::span<const wifi::IqSample> iq, double cfo)
{
    HtSigResult result;
    if (iq.size() < wifi::ht_stf_start || !std::isfinite(cfo))
    {
        result.error = "Need 368 continuous samples and finite CFO";
        return result;
    }
    iq = iq.first(wifi::ht_stf_start);
    if (!std::ranges::all_of(iq, [](auto x) { return std::isfinite(x.real()) && std::isfinite(x.imag()); }))
    {
        result.error = "Non-finite IQ";
        return result;
    }

    const auto channel = fft_.legacyChannel(iq, cfo);
    if (!wifi::hasChannelEnergy(channel))
    {
        result.error = "No channel estimate";
        return result;
    }

    std::array<double, 96> soft{};
    for (size_t symbol = 0; symbol < 2; ++symbol)
    {
        const auto signal = readSignalSymbol(fft_, iq, cfo, channel, symbol);
        if (signal.axis != OfdmSignalAxis::Quadrature)
        {
            result.error = "Expected Q-BPSK on both HT-SIG symbols";
            return result;
        }
        wifi::deinterleave(signal.soft, std::span(soft).subspan(symbol * 48, 48), 1, 16);
    }
    std::ranges::copy(wifi::decodeBcc(soft, result.bits.size(), wifi::BccRate::Half), result.bits.begin());

    if (!crcOk(result.bits))
    {
        result.error = "HT-SIG CRC-8 failed";
        return result;
    }
    if (result.bits[26] != 1)
    {
        result.error = "HT-SIG reserved bit must be 1";
        return result;
    }

    const auto field = [&](size_t start, size_t count) {
        unsigned value = 0;
        for (size_t i = 0; i < count; ++i)
            value |= unsigned(result.bits[start + i]) << i;
        return value;
    };
    result.mcs = field(0, 7);
    result.cbw40 = field(7, 1);
    result.length_bytes = field(8, 16);
    result.smoothing = field(24, 1);
    result.not_sounding = field(25, 1);
    result.aggregation = field(27, 1);
    result.stbc = field(28, 2);
    result.ldpc = field(30, 1);
    result.short_gi = field(31, 1);
    result.extension_streams = field(32, 2);

    if (result.mcs > 76 || (result.mcs == 32 && !result.cbw40))
    {
        result.error = "Invalid HT-SIG MCS for signaled bandwidth";
        return result;
    }
    result.valid = true;
    if (result.cbw40)
        result.error = "HT40 bandwidth is unsupported";
    else if (result.mcs > 7)
        result.error = "HT MIMO / multi-stream MCS is unsupported";
    else if (result.stbc)
        result.error = "HT STBC is unsupported";
    else if (result.ldpc)
        result.error = "HT LDPC is unsupported";
    else if (result.extension_streams)
        result.error = "HT extension spatial streams are unsupported";
    else
        result.supported_siso_ht20 = true;
    return result;
}
