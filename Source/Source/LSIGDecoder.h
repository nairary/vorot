#pragma once
#include <fftw3.h>

#include <array>
#include <algorithm>
#include <bit>
#include <cmath>
#include <complex>
#include <cstdint>
#include <limits>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>

struct LSigResult
{
	bool valid = false;
	unsigned rate_mbps = 0;
	unsigned length_bytes = 0;

	// 24 декодированных бита в порядке передачи
	std::array<uint8_t, 24> bits{};
	double pilot_phase_rad = 0;
	std::string channel_parameters;
	std::string error;
};

class LSIGDecoder
{
public:
	LSIGDecoder();
	~LSIGDecoder();
	LSIGDecoder(const LSIGDecoder&) = delete;
	LSIGDecoder& operator=(const LSIGDecoder&) = delete;

	// iq[0] соответствует первому символу LTF
	LSigResult decode(
		std::span<const std::complex<double>> iq,
		double total_cfo_hz,
		double sample_rate_hz = 20e6
	);
private:
	static constexpr double two_pi = 2 * std::numbers::pi;
	fftwf_complex fft_input_[64]{};
	fftwf_complex fft_output_[64]{};
	fftwf_plan plan_ = nullptr;

	std::array<std::complex<double>, 64> fftCorrected(
		std::span<const std::complex<double>> iq,
		size_t start,
		double cfo_hz,
		double sample_rate_hz
	);

	static size_t bin(int carrier)
	{
		// FFTW: DC = 0, положительные частоты 1...31,
		// отрицательные -32...-1 представлены индексами 32...63.
		return static_cast<size_t>(carrier >= 0 ? carrier : carrier + 64);
	}

	static bool isPilot(int carrier)
	{
		return carrier == -21 || carrier == -7 || carrier == 7 || carrier == 21;
	}

	static unsigned parity(unsigned value)
	{
		return std::popcount(value) & 1u;
	}

	static unsigned rateMbps(unsigned field)
	{
		switch (field) {
		case 0xD: return 6;
		case 0xF: return 9;
		case 0x5: return 12;
		case 0x7: return 18;
		case 0x9: return 24;
		case 0xB: return 36;
		case 0x1: return 48;
		case 0x3: return 54;
		default:  return 0;
		}
	}

	static std::string rateToChannel(unsigned rate_mbps)
	{
		switch (rate_mbps)
		{
		case 6: return std::string{ "Modulation BPSK, Coding 1/2" };
		case 9: return std::string{ "Modulation BPSK, Coding 3/4" };
		case 12: return std::string{ "Modulation QPSK, Coding 1/2" };
		case 18: return std::string{ "Modulation QPSK, Coding 3/4" };
		case 24: return std::string{ "Modulation 16-QAM, Coding 1/2" };
		case 36: return std::string{ "Modulation 16-QAM, Coding 3/4" };
		case 48: return std::string{ "Modulation 64-QAM, Coding 1/2" };
		case 54: return std::string{ "Modulation 64-QAM, Coding 3/4" };
		default: return std::string{ "Unknowned modulation and coding" };
		}
	}

	// надеюсь, что алгоритм Витерби написна верно
	static std::array<uint8_t, 24> viterbi(
		const std::array<double, 48>& soft)
	{
		constexpr double minus_infinity =
			-std::numeric_limits<double>::infinity();

		std::array<double, 64> metric;
		metric.fill(minus_infinity);
		metric[0] = 0; // SIGNAL начинается с нулевого состояния.

		std::array<std::array<uint8_t, 64>, 24> predecessor{};
		std::array<std::array<uint8_t, 64>, 24> input_bit{};

		for (size_t t = 0; t < 24; ++t) {
			std::array<double, 64> next_metric;
			next_metric.fill(minus_infinity);

			for (unsigned state = 0; state < 64; ++state) {
				if (!std::isfinite(metric[state]))
					continue;

				for (unsigned bit = 0; bit <= 1; ++bit) {
					// Текущий вход находится в младшем бите.
					const unsigned reg = (state << 1) | bit;
					const unsigned next_state = reg & 63u;

					// IEEE generators (133,171) octal.
					// Для выбранного порядка хранения регистра
					// маски отражены: 0155 и 0117.
					const unsigned a = parity(reg & 0155u);
					const unsigned b = parity(reg & 0117u);

					// Положительное soft-значение поддерживает bit=0.
					const double score =
						metric[state] +
						(a == 0 ? soft[2 * t] : -soft[2 * t]) +
						(b == 0 ? soft[2 * t + 1] : -soft[2 * t + 1]);

					if (score > next_metric[next_state]) {
						next_metric[next_state] = score;
						predecessor[t][next_state] =
							static_cast<uint8_t>(state);
						input_bit[t][next_state] =
							static_cast<uint8_t>(bit);
					}
				}
			}

			metric = next_metric;
		}

		// SIGNAL заканчивается шестью нулевыми tail-битами.
		unsigned state = 0;
		std::array<uint8_t, 24> bits{};

		for (int t = 23; t >= 0; --t) {
			bits[t] = input_bit[t][state];
			state = predecessor[t][state];
		}

		return bits;
	}
};