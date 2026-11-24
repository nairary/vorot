#include "WifiOfdm.h"
#include "WifiBcc.h"

#include <numbers>
#include <stdexcept>

namespace wifi {
namespace {
constexpr double tau = 2 * std::numbers::pi;

constexpr auto pilot_polarity = [] {
    std::array<int, 127> polarity{};
    unsigned state = 0x7f;
    for (auto& value : polarity)
        value = 1 - 2 * scramblerBit(state);
    return polarity;
}();

const auto timing_correction = [] {
    std::array<Spectrum, 2> correction{};
    for (size_t i = 0; i < correction.size(); ++i)
    {
        const size_t early = i == 0 ? long_guard / 4 : short_guard / 4;
        for (size_t k = 0; k < fft_size; ++k)
            correction[i][k] = std::polar(1.0, tau * double(k * early) / fft_size);
    }
    return correction;
}();
}

OfdmFft::OfdmFft()
    : plan_(fftwf_plan_dft_1d(static_cast<int>(fft_size), input_, output_, FFTW_FORWARD, FFTW_ESTIMATE))
{
    if (!plan_)
        throw std::runtime_error("Cannot create Wi-Fi FFTW plan");
}

OfdmFft::~OfdmFft()
{
    fftwf_destroy_plan(plan_);
}

Spectrum OfdmFft::transform(std::span<const IqSample> iq, size_t start, double cfo, double rate)
{
    const double radians = -tau * cfo / rate;
    const auto step = std::polar(1.0, radians);
    auto phase = std::polar(1.0, radians * double(start));
    for (size_t i = 0; i < fft_size; ++i)
    {
        const auto value = iq[start + i] * phase;
        input_[i][0] = static_cast<float>(value.real());
        input_[i][1] = static_cast<float>(value.imag());
        phase *= step;
    }
    fftwf_execute(plan_);

    Spectrum spectrum{};
    for (size_t k = 0; k < fft_size; ++k)
        spectrum[k] = {output_[k][0], output_[k][1]};
    return spectrum;
}

Spectrum OfdmFft::symbol(std::span<const IqSample> iq, size_t cp_start, size_t guard, double cfo, double rate)
{
    // Start just inside the CP to tolerate a slightly late timing lock.
    // Undo the resulting phase slope so all symbols share the LTF reference.
    auto spectrum = transform(iq, cp_start + guard - guard / 4, cfo, rate);
    const auto& correction = timing_correction[guard == short_guard ? 1 : 0];
    for (size_t k = 0; k < fft_size; ++k)
        spectrum[k] *= correction[k];
    return spectrum;
}

Spectrum OfdmFft::legacyChannel(std::span<const IqSample> iq, double cfo, double rate)
{
    const auto first = transform(iq, 0, cfo, rate);
    const auto second = transform(iq, fft_size, cfo, rate);
    Spectrum channel{};
    for (int k = -26; k <= 26; ++k)
        channel[bin(k)] = (first[bin(k)] + second[bin(k)]) * (0.5 * legacy_ltf[k + 26]);
    return channel;
}

Pilots pilotValues(size_t polarity_index, size_t rotation)
{
    constexpr Pilots signs{1, 1, 1, -1};
    const auto polarity = pilot_polarity[polarity_index % pilot_polarity.size()];
    Pilots result{};
    for (size_t i = 0; i < result.size(); ++i)
        result[i] = polarity * signs[(i + rotation) % signs.size()];
    return result;
}

std::optional<IqSample> pilotCorrection(const Spectrum& received, const Spectrum& channel, const Pilots& pilots)
{
    IqSample sum{};
    for (size_t i = 0; i < pilots.size(); ++i)
    {
        const auto k = bin(pilot_carriers[i]);
        sum += received[k] * std::conj(channel[k]) * double(pilots[i]);
    }
    if (!(std::norm(sum) > 1e-24))
        return {};
    return std::conj(sum) / std::abs(sum);
}

bool hasChannelEnergy(const Spectrum& channel)
{
    double energy = 0;
    for (const auto& tone : channel)
        energy += std::norm(tone);
    return energy > 1e-20 && std::isfinite(energy);
}

}
