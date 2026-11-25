#include "WifiDataDecoder.h"
#include "WifiBcc.h"

#include <algorithm>
#include <limits>

namespace {

constexpr size_t service_bits = 16;
constexpr size_t tail_bits = 6;

struct Rate
{
    size_t bits_per_carrier;
    wifi::BccRate coding;
    size_t data_bits_per_symbol;
};

using wifi::BccRate;

constexpr std::array<unsigned, 8> legacy_mbps{6, 9, 12, 18, 24, 36, 48, 54};
constexpr std::array<Rate, 8> legacy_rates{{
    {1, BccRate::Half, 24},           {1, BccRate::ThreeQuarters, 36},
    {2, BccRate::Half, 48},           {2, BccRate::ThreeQuarters, 72},
    {4, BccRate::Half, 96},           {4, BccRate::ThreeQuarters, 144},
    {6, BccRate::TwoThirds, 192},     {6, BccRate::ThreeQuarters, 216}
}};
constexpr std::array<Rate, 8> ht_rates{{
    {1, BccRate::Half, 26},           {2, BccRate::Half, 52},
    {2, BccRate::ThreeQuarters, 78},  {4, BccRate::Half, 104},
    {4, BccRate::ThreeQuarters, 156}, {6, BccRate::TwoThirds, 208},
    {6, BccRate::ThreeQuarters, 234}, {6, BccRate::FiveSixths, 260}
}};

struct Layout
{
    Rate rate;
    size_t data_start;
    size_t guard;
    size_t symbols;
    size_t bits_through_tail;
    bool ht;

    size_t symbolSamples() const { return wifi::fft_size + guard; }
    size_t sampleCount() const { return data_start + symbols * symbolSamples(); }
};

std::optional<Layout> dataLayout(const WifiDataParameters& params)
{
    const bool ht = params.format == WifiPhyFormat::HtMixed;
    if (params.psdu_bytes == 0 || params.psdu_bytes > (ht ? 65535u : 4095u))
        return {};

    Rate rate{};
    if (ht)
    {
        if (params.rate_or_mcs >= ht_rates.size())
            return {};
        rate = ht_rates[params.rate_or_mcs];
    }
    else
    {
        const auto found = std::ranges::find(legacy_mbps, params.rate_or_mcs);
        if (found == legacy_mbps.end() || params.short_gi)
            return {};
        rate = legacy_rates[found - legacy_mbps.begin()];
    }
    const size_t bits = service_bits + 8 * params.psdu_bytes + tail_bits;
    const size_t symbols = (bits + rate.data_bits_per_symbol - 1) / rate.data_bits_per_symbol;
    return Layout{rate, ht ? wifi::ht_data_start : wifi::legacy_data_start,
                  params.short_gi ? wifi::short_guard : wifi::long_guard, symbols, bits, ht};
}

std::span<const double> axisLevels(size_t bits_per_carrier)
{
    // Axis labels put the sign bit in the LSB. The full label lists the I
    // bits first, then the Q bits. The scale makes mean symbol power one.
    static constexpr double bpsk[]{-1, 1};
    static constexpr double qpsk[]{-0.7071067811865475, 0.7071067811865475};
    static const double qam16[]{-3 / std::sqrt(10.0), 3 / std::sqrt(10.0),
                              -1 / std::sqrt(10.0), 1 / std::sqrt(10.0)};
    static const double qam64[]{-7 / std::sqrt(42.0), 7 / std::sqrt(42.0),
                              -1 / std::sqrt(42.0), 1 / std::sqrt(42.0),
                              -5 / std::sqrt(42.0), 5 / std::sqrt(42.0),
                              -3 / std::sqrt(42.0), 3 / std::sqrt(42.0)};

    switch (bits_per_carrier)
    {
    case 1: return bpsk;
    case 2: return qpsk;
    case 4: return qam16;
    default: return qam64;
    }
}

void demapAxis(double matched, double gain, std::span<const double> levels,
               std::span<double> output)
{
    constexpr double infinity = std::numeric_limits<double>::infinity();
    std::array<double, 8> distances{};
    for (size_t label = 0; label < levels.size(); ++label)
    {
        const double level = levels[label];
        // Weighted squared distance, with the common term canceled.
        // This avoids division by channel gain in deep fades.
        distances[label] = level * (level * gain - 2 * matched);
    }
    for (size_t bit = 0, mask = 1; mask < levels.size(); ++bit, mask <<= 1)
    {
        double minimum[2]{infinity, infinity};
        for (size_t label = 0; label < levels.size(); ++label)
        {
            auto& best = minimum[(label & mask) != 0];
            best = std::min(best, distances[label]);
        }
        output[bit] = minimum[1] - minimum[0];
    }
}

bool hasHtStf(std::span<const wifi::IqSample> iq)
{
    // HT-STF contains repetitions 16 samples apart. The USRP uses fixed gain,
    // so use the field for validation and train the equalizer on HT-LTF.
    wifi::IqSample correlation{};
    double first_energy = 0, second_energy = 0;
    for (size_t i = 16; i < 64; ++i)
    {
        const auto a = iq[wifi::ht_stf_start + i];
        const auto b = iq[wifi::ht_stf_start + i + 16];
        correlation += std::conj(a) * b;
        first_energy += std::norm(a);
        second_energy += std::norm(b);
    }
    const double energy = first_energy * second_energy;
    return energy > 1e-24 && std::norm(correlation) > 0.5 * energy;
}

wifi::Spectrum htChannel(wifi::OfdmFft& fft, std::span<const wifi::IqSample> iq, double cfo)
{
    const auto ltf = fft.symbol(iq, wifi::ht_ltf_start, wifi::long_guard, cfo);
    wifi::Spectrum channel{};
    for (int k = -28; k <= 28; ++k)
    {
        const int reference = k < -26 ? 1 : (k > 26 ? -1 : wifi::legacy_ltf[k + 26]);
        const auto b = wifi::bin(k);
        channel[b] = ltf[b] * double(reference);
    }
    return channel;
}

}

std::optional<size_t> WifiDataDecoder::sampleCount(const WifiDataParameters& params)
{
    const auto layout = dataLayout(params);
    if (!layout)
        return {};
    return layout->sampleCount();
}

WifiDataResult WifiDataDecoder::decode(std::span<const wifi::IqSample> iq, double cfo,
                                       const WifiDataParameters& params)
{
    const auto layout = dataLayout(params);
    if (!layout)
        return {{}, "Invalid OFDM DATA parameters"};
    if (iq.size() < layout->sampleCount() || !std::isfinite(cfo))
        return {{}, "Missing continuous OFDM DATA samples or CFO"};
    if (layout->ht && !hasHtStf(iq))
        return {{}, "HT-STF periodicity check failed"};

    const auto channel = layout->ht ? htChannel(fft_, iq, cfo) : fft_.legacyChannel(iq, cfo);
    if (!wifi::hasChannelEnergy(channel))
        return {{}, "No DATA channel estimate"};

    const size_t bpsc = layout->rate.bits_per_carrier;
    const size_t data_carriers = layout->ht ? 52 : 48;
    const size_t coded_per_symbol = data_carriers * bpsc;
    const int band_edge = layout->ht ? 28 : 26;
    const auto levels = axisLevels(bpsc);
    const size_t axis_bits = std::max<size_t>(1, bpsc / 2);
    std::vector<double> coded(layout->symbols * coded_per_symbol);
    std::array<double, 52 * 6> soft{};

    for (size_t symbol = 0; symbol < layout->symbols; ++symbol)
    {
        const size_t start = layout->data_start + symbol * layout->symbolSamples();
        const auto received = fft_.symbol(iq, start, layout->guard, cfo);
        const auto pilots = layout->ht ? wifi::pilotValues(symbol + 3, symbol) : wifi::pilotValues(symbol + 1);
        const auto correction = wifi::pilotCorrection(received, channel, pilots);
        if (!correction)
            return {{}, "Cannot track DATA pilot phase"};

        size_t position = 0;
        for (int k = -band_edge; k <= band_edge; ++k)
        {
            if (k == 0 || wifi::isPilot(k))
                continue;
            const auto b = wifi::bin(k);
            const auto matched = received[b] * std::conj(channel[b]) * *correction;
            const double gain = std::norm(channel[b]);
            const auto out = std::span(soft).subspan(position, bpsc);
            demapAxis(matched.real(), gain, levels, out.first(axis_bits));
            if (bpsc > 1)
                demapAxis(matched.imag(), gain, levels, out.last(axis_bits));
            position += bpsc;
        }
        wifi::deinterleave(std::span(soft).first(coded_per_symbol),
                           std::span(coded).subspan(symbol * coded_per_symbol, coded_per_symbol),
                           bpsc, layout->ht ? 13 : 16);
    }

    // The zero convolutional state occurs at the end of TAIL. PAD follows
    // TAIL and can leave the encoder in a nonzero state.
    const auto bits = wifi::decodeBcc(coded, layout->bits_through_tail, layout->rate.coding);
    auto psdu = wifi::descramblePsdu(bits, params.psdu_bytes);
    if (psdu.empty())
        return {{}, "Cannot recover the scrambler from DATA SERVICE bits"};
    return {std::move(psdu), {}};
}
