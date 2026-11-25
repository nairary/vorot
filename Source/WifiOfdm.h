#pragma once

#include <fftw3.h>
#include <array>
#include <complex>
#include <cstddef>
#include <optional>
#include <span>

namespace wifi {

inline constexpr size_t fft_size = 64;
inline constexpr size_t long_guard = 16;
inline constexpr size_t short_guard = 8;
inline constexpr size_t symbol_samples = fft_size + long_guard;

// Offsets relative to the first useful L-LTF sample.
inline constexpr size_t legacy_data_start = 2 * fft_size + symbol_samples;
inline constexpr size_t ht_stf_start = legacy_data_start + 2 * symbol_samples;
inline constexpr size_t ht_ltf_start = ht_stf_start + symbol_samples;
inline constexpr size_t ht_data_start = ht_ltf_start + symbol_samples;
inline constexpr double sample_rate_hz = 20e6;

using IqSample = std::complex<double>;
using Spectrum = std::array<IqSample, fft_size>;
using Pilots = std::array<int, 4>;

inline constexpr Pilots pilot_carriers{-21, -7, 7, 21};

constexpr size_t bin(int carrier)
{
    return static_cast<size_t>(carrier < 0 ? carrier + 64 : carrier);
}

constexpr bool isPilot(int carrier)
{
    return carrier == -21 || carrier == -7 || carrier == 7 || carrier == 21;
}

// IEEE L-LTF on [-26,26], including DC. The HT-LTF adds two tones at each edge.
inline constexpr std::array<int, 53> legacy_ltf{
     1, 1,-1,-1, 1, 1,-1, 1,-1, 1, 1, 1, 1,
     1, 1,-1,-1, 1, 1,-1, 1,-1, 1, 1, 1, 1,
     0,
     1,-1,-1, 1, 1,-1, 1,-1, 1,-1,-1,-1,-1,
    -1, 1, 1,-1,-1, 1,-1, 1,-1, 1, 1, 1, 1
};

class OfdmFft
{
public:
    OfdmFft();
    ~OfdmFft();
    OfdmFft(const OfdmFft&) = delete;
    OfdmFft& operator=(const OfdmFft&) = delete;

    Spectrum transform(std::span<const IqSample> iq, size_t start, double cfo,
                       double rate = sample_rate_hz);
    Spectrum symbol(std::span<const IqSample> iq, size_t cp_start, size_t guard, double cfo,
                    double rate = sample_rate_hz);
    Spectrum legacyChannel(std::span<const IqSample> iq, double cfo, double rate = sample_rate_hz);

private:
    fftwf_complex input_[fft_size]{};
    fftwf_complex output_[fft_size]{};
    fftwf_plan plan_;
};

Pilots pilotValues(size_t polarity_index, size_t rotation = 0);
std::optional<IqSample> pilotCorrection(const Spectrum& received, const Spectrum& channel, const Pilots& pilots);
bool hasChannelEnergy(const Spectrum& channel);

}
