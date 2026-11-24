#pragma once
#pragma once

#include <fftw3.h>

#include <array>
#include <bit>
#include <cmath>
#include <complex>
#include <cstdint>
#include <limits>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>

struct HtSigResult {
    // CRC и reserved bit прошли проверку.
    bool valid = false;

    // Поддерживаемая здесь конфигурация последующего DATA:
    // HT20, MCS 0...7, один поток, без STBC/LDPC/extension streams.
    bool supported_siso_ht20 = false;

    unsigned mcs = 0;
    unsigned length_bytes = 0;
    unsigned stbc = 0;
    unsigned extension_streams = 0;

    bool cbw40 = false;
    bool smoothing = false;
    bool not_sounding = false;
    bool aggregation = false;
    bool ldpc = false;
    bool short_gi = false;

    std::array<uint8_t, 48> bits{};
    std::string error;
};

struct HtSigEvent {
    uint64_t segment = 0;
    uint64_t ltf_first_index = 0;
    HtSigResult signal;
};

class HtSigDecoder {
    using Spectrum = std::array<std::complex<double>, 64>;

public:
    HtSigDecoder();
    ~HtSigDecoder();
    HtSigDecoder(const HtSigDecoder&) = delete;
    HtSigDecoder& operator=(const HtSigDecoder&) = delete;

    // iq[0] соответствует ltf_first_index.
    // Формат: комплексные IQ, нормализованные делением sc16 на 32768.
    // Частота дискретизации: 20 MS/s.
    HtSigResult decode(
        std::span<const std::complex<double>> iq,
        double total_cfo_hz);

private:
    static constexpr double tau = 2 * std::numbers::pi;

    fftwf_complex in_[64]{};
    fftwf_complex out_[64]{};
    fftwf_plan plan_ = nullptr;

    static size_t bin(int carrier)
    {
        return size_t(carrier < 0 ? carrier + 64 : carrier);
    }

    static bool isPilot(int carrier)
    {
        return carrier == -21 || carrier == -7 ||
            carrier == 7 || carrier == 21;
    }

    Spectrum fft(
        std::span<const std::complex<double>> iq,
        size_t start,
        double cfo_hz);

    static std::array<uint8_t, 48> viterbi(
        const std::array<double, 96>& soft);

    static bool crcOk(const std::array<uint8_t, 48>& bits);
};