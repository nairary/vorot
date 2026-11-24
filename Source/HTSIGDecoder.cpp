#include "HTSIGDecoder.h"

HtSigDecoder::HtSigDecoder()
{
    plan_ = fftwf_plan_dft_1d(
        64,
        in_,
        out_,
        FFTW_FORWARD,
        FFTW_ESTIMATE
    );

    if (!plan_)
        throw std::runtime_error("HT-SIG FFTW plan failed");
}

HtSigDecoder::~HtSigDecoder()
{
    if (plan_)
        fftwf_destroy_plan(plan_);
}

HtSigResult HtSigDecoder::decode(std::span<const std::complex<double>> iq, double total_cfo_hz)
{
    HtSigResult result;

    if (iq.size() < 368 || !std::isfinite(total_cfo_hz)) {
        result.error =
            "Need 368 continuous samples and finite CFO";
        return result;
    }

    for (size_t i = 0; i < 368; ++i) {
        if (!std::isfinite(iq[i].real()) ||
            !std::isfinite(iq[i].imag())) {
            result.error = "Non-finite IQ";
            return result;
        }
    }

    // IEEE legacy L-LTF, поднесущие -26...26.
    static constexpr int ltf[53] = {
         1, 1,-1,-1, 1, 1,-1, 1,-1, 1, 1, 1, 1,
         1, 1,-1,-1, 1, 1,-1, 1,-1, 1, 1, 1, 1,
         0,
         1,-1,-1, 1, 1,-1, 1,-1, 1,-1,-1,-1,-1,
        -1, 1, 1,-1,-1, 1,-1, 1,-1, 1, 1, 1, 1
    };

    const auto ltf1 = fft(iq, 0, total_cfo_hz);
    const auto ltf2 = fft(iq, 64, total_cfo_hz);

    Spectrum channel{};
    double channel_energy = 0;

    for (int k = -26; k <= 26; ++k) {
        if (k == 0)
            continue;

        const size_t n = bin(k);

        channel[n] =
            0.5 * (ltf1[n] + ltf2[n]) * double(ltf[k + 26]);

        channel_energy += std::norm(channel[n]);
    }

    if (!(channel_energy > 1e-20)) {
        result.error = "No channel estimate";
        return result;
    }

    std::array<double, 96> coded_soft{};

    static constexpr int pilot_carriers[4] = {
        -21, -7, 7, 21
    };

    static constexpr int pilot_signs[4] = {
        1, 1, 1, -1
    };

    for (size_t symbol = 0; symbol < 2; ++symbol) {
        const auto y = fft(
            iq,
            224 + 80 * symbol,
            total_cfo_hz
        );

        std::complex<double> pilot_sum{};

        // Пилоты HT-SIG остаются legacy BPSK.
        // Для этих двух символов polarity p1=p2=+1.
        for (size_t i = 0; i < 4; ++i) {
            const size_t n = bin(pilot_carriers[i]);

            pilot_sum +=
                y[n] * std::conj(channel[n])
                * double(pilot_signs[i]);
        }

        if (!(std::norm(pilot_sum) > 1e-24)) {
            result.error = "No pilot phase estimate";
            return result;
        }

        const auto phase_correction =
            std::polar(1.0, -std::arg(pilot_sum));

        std::array<double, 48> interleaved_soft{};
        size_t position = 0;

        for (int k = -26; k <= 26; ++k) {
            if (k == 0 || isPilot(k))
                continue;

            const size_t n = bin(k);

            const auto value =
                y[n] * std::conj(channel[n])
                * phase_correction;

            // QBPSK: +j -> bit 0, -j -> bit 1.
            // imag(value) == real(value * (-j)).
            //
            // Умножение на conj(H) также даёт вес |H|²:
            // слабые поднесущие имеют меньшую надёжность.
            interleaved_soft[position++] = value.imag();
        }

        // Деинтерливинг выполняется отдельно для каждого символа.
        // N_CBPS=48, N_BPSC=1.
        for (size_t k = 0; k < 48; ++k) {
            const size_t j = 3 * (k % 16) + k / 16;

            coded_soft[48 * symbol + k] =
                interleaved_soft[j];
        }
    }

    // Один непрерывный Viterbi для обоих HT-SIG символов.
    result.bits = viterbi(coded_soft);

    if (!crcOk(result.bits)) {
        result.error = "HT-SIG CRC-8 failed";
        return result;
    }

    // В HT-SIG этот reserved bit должен быть единицей.
    if (result.bits[26] != 1) {
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

    result.valid = true;

    result.supported_siso_ht20 =
        result.mcs <= 7 &&
        !result.cbw40 &&
        result.stbc == 0 &&
        !result.ldpc &&
        result.extension_streams == 0;

    return result;
}

HtSigDecoder::Spectrum HtSigDecoder::fft(std::span<const std::complex<double>> iq, size_t start, double cfo_hz)
{
    for (size_t k = 0; k < 64; ++k) {
        const size_t n = start + k;

        // Общая фазовая шкала для LTF и обоих HT-SIG.
        const auto x = iq[n] * std::polar(
            1.0,
            -tau * cfo_hz * double(n) / 20e6
        );

        in_[k][0] = float(x.real());
        in_[k][1] = float(x.imag());
    }

    fftwf_execute(plan_);

    Spectrum output{};

    for (size_t k = 0; k < 64; ++k) {
        output[k] = {
            double(out_[k][0]),
            double(out_[k][1])
        };
    }

    return output;
}

std::array<uint8_t, 48> HtSigDecoder::viterbi(const std::array<double, 96>& soft)
{
    const double negative_infinity =
        -std::numeric_limits<double>::infinity();

    std::array<double, 64> metric;
    metric.fill(negative_infinity);
    metric[0] = 0;

    std::array<std::array<uint8_t, 64>, 48> predecessor{};
    std::array<std::array<uint8_t, 64>, 48> input_bit{};

    for (size_t t = 0; t < 48; ++t) {
        std::array<double, 64> next;
        next.fill(negative_infinity);

        for (unsigned state = 0; state < 64; ++state) {
            if (!std::isfinite(metric[state]))
                continue;

            for (unsigned bit = 0; bit < 2; ++bit) {
                const unsigned reg = (state << 1) | bit;
                const unsigned destination = reg & 63u;

                // IEEE generators (133,171) octal.
                // При хранении нового бита в LSB используются
                // отражённые маски 0155 и 0117.
                const unsigned a =
                    std::popcount(reg & 0155u) & 1u;

                const unsigned b =
                    std::popcount(reg & 0117u) & 1u;

                const double score =
                    metric[state] +
                    (a ? -soft[2 * t] : soft[2 * t]) +
                    (b ? -soft[2 * t + 1] : soft[2 * t + 1]);

                if (score > next[destination]) {
                    next[destination] = score;

                    predecessor[t][destination] =
                        uint8_t(state);

                    input_bit[t][destination] =
                        uint8_t(bit);
                }
            }
        }

        metric = next;
    }

    // Последние шесть tail-битов завершают кодер
    // в нулевом состоянии.
    unsigned state = 0;
    std::array<uint8_t, 48> bits{};

    for (int t = 47; t >= 0; --t) {
        bits[t] = input_bit[t][state];
        state = predecessor[t][state];
    }

    return bits;
}

bool HtSigDecoder::crcOk(const std::array<uint8_t, 48>& bits)
{
    // CRC-8: x^8 + x^2 + x + 1.
    // Initial register = all ones.
    uint8_t crc = 0xff;

    // CRC покрывает первые 34 бита HT-SIG.
    for (size_t i = 0; i < 34; ++i) {
        const unsigned feedback =
            ((crc >> 7) ^ bits[i]) & 1u;

        crc = uint8_t(crc << 1);

        if (feedback)
            crc ^= 0x07;
    }

    // Передаются инвертированные биты регистра,
    // начиная со старшего.
    const uint8_t expected = uint8_t(~crc);

    for (unsigned i = 0; i < 8; ++i) {
        if (bits[34 + i] !=
            ((expected >> (7 - i)) & 1u)) {
            return false;
        }
    }

    return true;
}
