#include "LSIGDecoder.h"

LSIGDecoder::LSIGDecoder()
{
    plan_ = fftwf_plan_dft_1d(
        64,
        fft_input_,
        fft_output_,
        FFTW_FORWARD,
        FFTW_ESTIMATE
    );

    if (!plan_)
        throw std::runtime_error("Cannot create L-SIG FFTW plan");
}

LSIGDecoder::~LSIGDecoder()
{
    if (plan_)
        fftwf_destroy_plan(plan_);
}

LSigResult LSIGDecoder::decode(std::span<const std::complex<double>> iq, double total_cfo_hz, double sample_rate_hz)
{
    LSigResult result;

    if (iq.size() < 208)
    {
        result.error = "Need 208 continuous samples from LTF1";
        return result;
    }

    if (!std::isfinite(total_cfo_hz) ||
        !std::isfinite(sample_rate_hz) ||
        std::abs(sample_rate_hz - 20e6) > 20.0)
    {
        result.error = "Invalid CFO or unsupported sampling rate";
        return result;
    }

    for (size_t i = 0; i < 208; ++i)
    {
        if (!std::isfinite(iq[i].real() || !std::isfinite(iq[i].imag())))
        {
            result.error = "Non-finite IQ sample";
            return result;
        }
    }
    // CFO компенсируется с единым отсчётом фазы для всех трёх FFT.
    const auto ltf1 = fftCorrected(iq, 0, total_cfo_hz, sample_rate_hz);
    const auto ltf2 = fftCorrected(iq, 64, total_cfo_hz, sample_rate_hz);
    const auto signal = fftCorrected(iq, 144, total_cfo_hz, sample_rate_hz);

    // Частотная последовательность IEEE legacy L-LTF:
    // поднесущие -26 ... 26, центральная поднесущая равна нулю.
    static constexpr std::array<int, 53> ltf = {
         1, 1,-1,-1, 1, 1,-1, 1,-1, 1, 1, 1, 1,
         1, 1,-1,-1, 1, 1,-1, 1,-1, 1, 1, 1, 1,
         0,
         1,-1,-1, 1, 1,-1, 1,-1, 1,-1,-1,-1,-1,
        -1, 1, 1,-1,-1, 1,-1, 1,-1, 1, 1, 1, 1
    };

    std::array<std::complex<double>, 64> channel{};
    double channel_energy = 0;
    for (int k = -26; k <= 26; ++k)
    {
        if (k == 0) continue;

        const size_t b = bin(k);
        // X_LTF[k] = ±1, поэтому деление эквивалентно умножению.
        channel[b] = 0.5 * (ltf1[b] + ltf2[b]) * double(ltf[k + 26]);
        channel_energy += std::norm(channel[b]);
    }

    if (!(channel_energy > 1e-20))
    {
        result.error = "L-LTF channel estimate has zero energy";
        return result;
    }

    // Пилоты первого SIGNAL-символа:
    // -21, -7, +7, +21 -> +1, +1, +1, -1.
    static constexpr std::array<int, 4> pilot_carriers = {
        -21, -7, 7, 21
    };
    static constexpr std::array<int, 4> pilot_values = {
        1,1,1,-1
    };

    std::complex<double> pilot_sum{};
    for (size_t i = 0; i < pilot_carriers.size(); ++i)
    {
        const size_t b = bin(pilot_carriers[i]);
        // Согласованная эквализация с весом по мощности канала
        pilot_sum += signal[b] * std::conj(channel[b]) * double(pilot_values[i]);
    }

    if (!(std::norm(pilot_sum) > 1e-24))
    {
        result.error = "Cannot estimate SIGNAL pilot phase";
        return result;
    }

    result.pilot_phase_rad = std::arg(pilot_sum);

    const auto phase_correction = std::polar(1.0, -result.pilot_phase_rad);
    // 48 информационных поднесущих в порядке -26 ... +26,
    // исключая DC и четыре пилота.
    std::array<double, 48> interleaved_soft{};
    size_t position = 0;

    for (int k = -26; k <= 26; ++k)
    {
        if (k == 0 || isPilot(k)) continue;
        const size_t b = bin(k);

        // BPSK: bit 0 -> +1, bit 1 -> -1.
        //
        // real(Y * conj(H)) эквивалентно real(Y/H), умноженному
        // на |H|². Так слабые поднесущие получают меньший вес,
        // а деление на почти нулевой H не усиливает шум.

        const auto value = signal[b] * std::conj(channel[b]) * phase_correction;
        interleaved_soft[position++] = value.real();
    }

    // Для L - SIG: N_CBPS = 48, N_BPSC = 1, s = 1.
    // Передающий interleaver помещает coded[k] в позицию:
    // j = 3*(k mod 16) + floor(k/16).
        std::array<double, 48> coded_soft{};

    for (size_t k = 0; k < 48; ++k) {
        const size_t j = 3 * (k % 16) + k / 16;
        coded_soft[k] = interleaved_soft[j];
    }

    result.bits = viterbi(coded_soft);

    // RATE передаётся как R1,R2,R3,R4.
    // Например, 1101 означает 6 Мбит/с и упаковывается в 0xD.
    const unsigned rate_field =
        (unsigned(result.bits[0]) << 3) |
        (unsigned(result.bits[1]) << 2) |
        (unsigned(result.bits[2]) << 1) |
        unsigned(result.bits[3]);

    result.rate_mbps = rateMbps(rate_field);

    if (result.rate_mbps == 0) {
        result.error = "Invalid L-SIG RATE";
        return result;
    }

    if (result.bits[4] != 0) {
        result.error = "L-SIG reserved bit is not zero";
        return result;
    }

    // LENGTH: 12 бит, младший передаётся первым.
    for (unsigned i = 0; i < 12; ++i)
        result.length_bytes |= unsigned(result.bits[5 + i]) << i;

    // Even parity по битам 0...17 включительно.
    unsigned parity = 0;

    for (size_t i = 0; i <= 17; ++i)
        parity ^= result.bits[i];

    if (parity != 0) {
        result.error = "L-SIG parity failed";
        return result;
    }

    // Декодер Viterbi использует известное нулевое конечное
    // состояние: последние шесть tail-битов ограничены нулями.
    // Это условие декодирования, а не независимая CRC-проверка.
    result.valid = true;
    result.channel_parameters = rateToChannel(result.rate_mbps);
    return result;
}

std::array<std::complex<double>, 64> LSIGDecoder::fftCorrected(std::span<const std::complex<double>> iq, size_t start, double cfo_hz, double sample_rate_hz)
{
    for (size_t k = 0; k < 64; ++k)
    {
        const size_t n = start + k;
        const auto corrected = iq[n] * std::polar(1.0, -two_pi * cfo_hz * double(n) / sample_rate_hz);
        fft_input_[k][0] = static_cast<float>(corrected.real());
        fft_input_[k][1] = static_cast<float>(corrected.imag());
    }

    fftwf_execute(plan_);

    std::array<std::complex<double>, 64> output{};

    for (size_t k = 0; k < 64; ++k)
    {
        output[k] = {
            double(fft_output_[k][0]),
            double(fft_output_[k][1])
        };
    }
    return output;
}
