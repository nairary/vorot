#include "WiFiDemodulator.h"

std::vector<uint8_t> demodulate_bpsk(const std::vector<std::complex<float>>& subcarriers)
{
	std::vector<uint8_t> bits;
	for (const auto& sample : subcarriers)
	{
		if (sample.real() > 0.0f)
		{
			bits.push_back(1);
		}
		else
		{
			bits.push_back(0);
		}
	}
	return bits;
}

void process_phy_demodulation(const std::vector<std::complex<int16_t>>& iq_buffer)
{
	if (iq_buffer.size() < 1000) return;

	size_t packet_start_index = 0;
	bool packet_found = false;

	// Обнаружение кадра
	for (size_t i = 0; i < iq_buffer.size() - OFDM_SYMBOL_SIZE; ++i)
	{
		std::complex<double> R = 0;
		double P = 0;
		for (int m = 0; m < STS_LENGTH; ++m)
		{
			std::complex<double> x1(iq_buffer[i + m].real(), iq_buffer[i + m].imag());
			std::complex<double> x2(iq_buffer[i + m + STS_LENGTH].real(), iq_buffer[i + m + STS_LENGTH].imag());

			R += x1 * std::conj(x2);
			P += std::norm(x1);
		}

		double metric = std::norm(R) / (P * P + 1e-6);
		if (metric > 0.75)
		{
			packet_start_index = i + 160;
			packet_found = true;
			break;
		}
	}

	if (!packet_found) return;
	std::cout << "[PHY] Found packet (sc16) on sample: " << packet_start_index << std::endl;

	// Удаление СР и нормализация для БПФ
	fftwf_complex* fft_in = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex) * FFT_SIZE);
	fftwf_complex* fft_out = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex) * FFT_SIZE);
	fftwf_plan plan = fftwf_plan_dft_1d(FFT_SIZE, fft_in, fft_out, FFTW_FORWARD, FFTW_ESTIMATE);

	for (int i = 0; i < FFT_SIZE; ++i)
	{
		// пропускаем 16 сэмплов СР
		std::complex<int16_t> raw_sample = iq_buffer[packet_start_index + CP_SIZE + i];
		fft_in[i][0] = static_cast<float>(raw_sample.real()) / 32768.0f;
		fft_in[i][1] = static_cast<float>(raw_sample.imag()) / 32768.0f;
	}

	std::vector<std::complex<float>> subcarriers(FFT_SIZE);
	for (int i = 0; i < FFT_SIZE; ++i)
	{
		subcarriers[i] = std::complex<float>(fft_out[i][0], fft_out[i][1]);
	}

	fftwf_destroy_plan(plan);
	fftwf_free(fft_in);
	fftwf_free(fft_out);

	// Работа с созвездием
	std::vector<uint8_t> decoded_bits = demodulate_bpsk(subcarriers);

	std::cout << "[PHY] First 10 bits of packet: ";
	for (int i = 0; i < 10; ++i) {
		std::cout << (int)decoded_bits[i] << " ";
	}
	std::cout << "..." << std::endl;
}

WiFiDemodulator::WiFiDemodulator(const size_t capacity) : queue_(capacity), worker_([this] {run();}) {}

WiFiDemodulator::~WiFiDemodulator() { queue_.close(); worker_.join(); }

// Пока эта функция жестко лочит нас на 20 МГц
void WiFiDemodulator::enable(bool enabled, double rate)
{
	std::lock_guard lock(mutex_);
	state_.enabled = enabled && std::isfinite(rate) && std::abs(rate - 20e6) <= 20;
	++generation_;
	state_.recent.clear();
	state_.recent_ht.clear();
	state_.error.clear();
}

void WiFiDemodulator::submit(std::span<const std::complex<int16_t>> samples, uint64_t first, uint64_t segment, bool gap)
{
	std::lock_guard lock(mutex_);
	if (state_.enabled)
		queue_.push(samples, first, segment, gap, generation_);
}

WiFiDemodulator::Snapshot WiFiDemodulator::snapshot() const
{
	std::lock_guard lock(mutex_);
	auto result = state_;
	result.queue = queue_.stats();
	return result;
}

// Пока здесь только поиск LSTF
void WiFiDemodulator::run()
{
	LStfDetector detector;
	LLTFSynchronizer synchronizer(htsig_decoder_);

	uint64_t generation = ~uint64_t{ 0 };
	uint64_t segment = ~uint64_t{ 0 };
	uint64_t next = 0;

	IqBlock block;
	
	while (queue_.pop(block))
	{
		uint64_t current;
		{
			{
				std::lock_guard lock(mutex_);
				if (!state_.enabled || block.token != generation_) continue;

				current = generation_;
				// Если был разрыв, то запускаем детектор заново
				if (current != generation || segment != block.segment || next != block.first || block.discontinuity)
				{
					detector.reset(block.first);
					synchronizer.reset(block.first);
					++state_.resets;
				}
			}
			try
			{
				std::vector<LStfEvent> events;
				std::vector<LLTFSyncEvent> synced;
				std::vector<HtSigEvent> htsig_events;
				uint64_t rejected = 0;
				for (size_t offset = 0; offset < block.samples.size(); offset += 256)
				{
					const auto part = std::span(block.samples).subspan(offset, std::min<size_t>(256, block.samples.size() - offset));
					synchronizer.append(part);
					auto found = detector.process(part);
					for (const auto& event : found)
					{
						synchronizer.candidate(event);
					}
					events.insert(events.end(), found.begin(), found.end());
					auto results = synchronizer.process(block.segment, rejected, htsig_events);
					synced.insert(synced.end(), results.begin(), results.end());
				}

				std::lock_guard lock(mutex_);
				if (current == generation_ && state_.enabled)
				{
					state_.processed_samples += block.samples.size();
					state_.rejected_candidates += rejected;

					// обработка lstf событий
					for (auto event : events)
					{
						event.segment = block.segment;
						++state_.candidates;
						state_.recent.push_back(event);
						if (state_.recent.size() > 8) state_.recent.erase(state_.recent.begin());
					}

					// обработка lltf событий
					for (auto event : synced)
					{
						++state_.synchronized_packets;
						state_.recent_sync.push_back(event);
						if (state_.recent_sync.size() > 8)
							state_.recent_sync.erase(state_.recent_sync.begin());
					}

					// обработка ht-sig событий
					for (const auto& event : htsig_events)
					{
						++state_.decoded_ht_sig;
						state_.recent_ht.push_back(event);

						if (state_.recent_ht.size() > 8)
							state_.recent_ht.erase(state_.recent_ht.begin());
					}
				}
			}
			catch (const std::exception& e) 
			{
				std::lock_guard lock(mutex_);
				state_.error = e.what();
				state_.enabled = false;
			}
			generation = current;
			segment = block.segment;
			next = block.first + block.samples.size();
		}
	}
}
