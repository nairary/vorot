#pragma once

#include <deque>
#include <complex>
#include <array>
#include <vector>
#include <numbers>
#include <algorithm>
#include "LStfDetector.h"
#include "LSIGDecoder.h"
#include "HTSIGDecoder.h"

constexpr double twoPi = 2 * std::numbers::pi;
constexpr double sampleRate = 20000000.0;

static std::array<std::complex<double>, 64> makeLtfReference()
{
	constexpr int tones[53] = {
		1,1,-1,-1,1,1,-1,1,-1,1,1,1,1,1,1,-1,-1,1,1,-1,1,-1,1,1,1,1,
		0,1,-1,-1,1,1,-1,1,-1,1,-1,-1,-1,-1,-1,1,1,-1,-1,1,-1,1,-1,1,1,1,1
	};
	std::array<std::complex<double>, 64> ref{};
	for (size_t n = 0; n < 64; ++n)
	{
		for (int k = -26; k <= 26; ++k)
			ref[n] += double(tones[k + 26]) * std::polar(1.0, twoPi * k * double(n) / 64.0) / 64.0;
	}
	return ref;
}

struct LLTFSyncEvent {
	uint64_t segment = 0;
	uint64_t candidate_index = 0;
	uint64_t ltf_first_index = 0;
	uint64_t packet_start_index = 0;
	double correlation = 0;
	double coarse_cfo_hz = 0;
	double fine_cfo_hz = 0;
	double total_cfo_hz = 0;
	LSigResult lsig_result{};
};

struct PendingHint {
	uint64_t segment;
	uint64_t ltf_first_index;
	double total_cfo_hz;
};

class LLTFSynchronizer 
{
public:
	explicit LLTFSynchronizer(HtSigDecoder& decoder) : htsig_decoder_(decoder) {}
	void reset(uint64_t first);
	void append(std::span<const std::complex<int16_t>> samples);
	void candidate(const LStfEvent& event);
	std::vector<LLTFSyncEvent> process(uint64_t segment, uint64_t& rejected, std::vector<HtSigEvent>& ht_events);
private:
	std::complex<double> at(uint64_t index) const;
	// Ищет корреляцию между данными и референсом
	double match(uint64_t index, double cfo) const;
	// Ищет корреляцию двух одинаковых символов между собой
	double repeatScore(uint64_t a, uint64_t b, size_t count) const;
	const std::array<std::complex<double>, 64> reference_ = makeLtfReference();

	std::deque<std::complex<double>> history_;
	std::deque<LStfEvent> pending_;
	uint64_t first_ = 0;

	LSIGDecoder lsig_decoder_ = LSIGDecoder();

	HtSigDecoder& htsig_decoder_;
	std::deque<PendingHint> pending_ht_;
};