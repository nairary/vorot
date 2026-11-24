#pragma once
// Пока здесь код для WiFi 2.4
#include <fftw3.h>
#include <vector>
#include <complex>
#include <iostream>
#include "Core.h"
#include "IqQueue.h"
#include "LStfDetector.h"
#include "LLTFSynchronizer.h"
#include "HTSIGDecoder.h"

const int FFT_SIZE = 64;
const int CP_SIZE = 16; // циклический префикс
const int OFDM_SYMBOL_SIZE = 80; // размер символа ofdm
const int STS_LENGTH = 16; // Длина преамбулы

std::vector<uint8_t> demodulate_bpsk(const std::vector<std::complex<float>>& subcarriers);
void process_phy_demodulation(const std::vector<std::complex<int16_t>>& iq_buffer);

class WiFiDemodulator
{
public:
	struct Snapshot
	{
		bool enabled = false;
		uint64_t candidates = 0; 
		uint64_t resets = 0; 
		uint64_t processed_samples = 0; 

		uint64_t synchronized_packets = 0;
		uint64_t rejected_candidates = 0;

		IqQueue::Stats queue;
		std::vector<LStfEvent> recent;
		std::vector<LLTFSyncEvent> recent_sync;

		uint64_t decoded_ht_sig = 0;
		std::vector<HtSigEvent> recent_ht;

		std::string error;
	};

	explicit WiFiDemodulator(size_t queue_capacity = 32);
	~WiFiDemodulator();
	void enable(bool enabled, double actual_rate_hz);
	void submit(std::span<const std::complex<int16_t>> samples, uint64_t first, uint64_t segment, bool gap);
	Snapshot snapshot() const;

private:
	void run();
	IqQueue queue_;
	mutable std::mutex mutex_;
	Snapshot state_;
	uint64_t generation_ = 0;

	HtSigDecoder htsig_decoder_;

	std::jthread worker_;
};