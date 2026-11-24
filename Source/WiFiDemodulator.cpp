#include "WiFiDemodulator.h"

#include <algorithm>
#include <iostream>
#include <iterator>

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

namespace {

constexpr size_t diagnostic_history_limit = 8;

template<class T>
void appendHistory(std::vector<T>& history, std::vector<T>& events, size_t limit = diagnostic_history_limit)
{
    const size_t incoming = std::min(events.size(), limit);
    const size_t available = limit - incoming;
    if (history.size() > available)
        history.erase(history.begin(), history.end() - static_cast<std::ptrdiff_t>(available));
    history.insert(history.end(), std::make_move_iterator(events.end() - static_cast<std::ptrdiff_t>(incoming)),
                   std::make_move_iterator(events.end()));
}

}

WiFiDemodulator::WiFiDemodulator(size_t capacity) : queue_(capacity), worker_([this] { run(); }) {}

WiFiDemodulator::~WiFiDemodulator()
{
    queue_.close();
    worker_.join();
}

void WiFiDemodulator::enable(bool enabled, double rate)
{
    std::lock_guard lock(mutex_);
    state_.enabled = enabled && std::isfinite(rate) && std::abs(rate - wifi::sample_rate_hz) <= 20;
    ++generation_;
    state_.recent.clear();
    state_.recent_sync.clear();
    state_.recent_ht.clear();
    state_.recent_frames.clear();
    state_.recent_diagnostics.clear();
    state_.error = enabled && !state_.enabled ? "Wi-Fi OFDM requires a 20 MS/s RX rate" : "";
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

void WiFiDemodulator::run()
{
    uint64_t generation = ~uint64_t{0};
    uint64_t segment = ~uint64_t{0};
    uint64_t next = 0;
    IqBlock block;
    while (queue_.pop(block))
    {
        {
            std::lock_guard lock(mutex_);
            if (!state_.enabled || block.token != generation_)
                continue;
        }

        try
        {
            Snapshot batch;
            if (block.token != generation || block.segment != segment || block.first != next || block.discontinuity)
            {
                if (block.token == generation)
                    for (const auto& packet : packets_)
                        diagnose(packet, "IQ discontinuity interrupted the packet", batch);
                detector_.reset(block.first);
                synchronizer_.reset(block.first);
                packets_.clear();
                ++batch.resets;
            }

            constexpr size_t chunk_size = 256;
            const auto samples = std::span<const std::complex<int16_t>>(block.samples);
            for (size_t offset = 0; offset < samples.size(); offset += chunk_size)
                processPart(samples.subspan(offset, std::min(chunk_size, samples.size() - offset)), block.segment, batch);
            batch.processed_samples = samples.size();

            std::lock_guard lock(mutex_);
            if (generation_ == block.token && state_.enabled)
                publish(batch);
        }
        catch (const std::exception& e)
        {
            std::lock_guard lock(mutex_);
            if (generation_ == block.token)
            {
                state_.error = e.what();
                state_.enabled = false;
            }
        }

        generation = block.token;
        segment = block.segment;
        next = block.first + block.samples.size();
    }
}

void WiFiDemodulator::processPart(std::span<const std::complex<int16_t>> samples, uint64_t segment, Snapshot& batch)
{
    synchronizer_.append(samples);
    for (auto event : detector_.process(samples))
    {
        synchronizer_.candidate(event);
        event.segment = segment;
        batch.recent.push_back(event);
    }

    auto synced = synchronizer_.process(segment, batch.rejected_candidates);
    for (const auto& event : synced)
    {
        if (!event.lsig_result.valid)
            continue;

        const auto rate = event.lsig_result.rate_mbps;
        const auto length = event.lsig_result.length_bytes;
        // The duration-matched HT-mixed L-SIG uses 6 Mb/s and a length
        // divisible by 3. Legacy packets can also have these fields.
        const bool could_be_ht = rate == 6 && length >= 9 && length % 3 == 0;
        packets_.push_back({event.segment, event.packet_start_index, event.ltf_first_index,
                            event.total_cfo_hz, length, {WifiPhyFormat::LegacyOfdm, rate, length},
                            could_be_ht ? PacketStage::IdentifyFormat : PacketStage::Data});
    }
    batch.recent_sync.insert(batch.recent_sync.end(), std::make_move_iterator(synced.begin()),
                             std::make_move_iterator(synced.end()));

    auto earliest = synchronizer_.endIndex();
    for (auto packet = packets_.begin(); packet != packets_.end();)
    {
        if (advancePacket(*packet, batch))
            packet = packets_.erase(packet);
        else
        {
            earliest = std::min(earliest, packet->ltf_first_index);
            ++packet;
        }
    }
    synchronizer_.discardBefore(earliest);
}

bool WiFiDemodulator::advancePacket(PendingPacket& packet, Snapshot& batch)
{
    const auto iq = synchronizer_.samplesFrom(packet.ltf_first_index);
    if (packet.stage == PacketStage::IdentifyFormat)
    {
        if (iq.size() < wifi::legacy_data_start + wifi::symbol_samples)
            return false;
        const auto axis = htsig_decoder_.firstSymbolAxis(iq, packet.cfo);
        if (axis == OfdmSignalAxis::Unclear)
        {
            diagnose(packet, "Cannot classify OFDM format from the first symbol after L-SIG", batch);
            return true;
        }
        packet.stage = axis == OfdmSignalAxis::Quadrature ? PacketStage::HtSignal : PacketStage::Data;
    }

    if (packet.stage == PacketStage::HtSignal)
    {
        if (iq.size() < wifi::ht_stf_start)
            return false;
        auto sig = htsig_decoder_.decode(iq, packet.cfo);
        if (sig.valid)
            ++batch.decoded_ht_sig;
        batch.recent_ht.push_back({packet.segment, packet.ltf_first_index, sig});
        if (!sig.supported_siso_ht20)
        {
            diagnose(packet, sig.error, batch);
            return true;
        }
        packet.data = {WifiPhyFormat::HtMixed, sig.mcs, sig.length_bytes, sig.short_gi};
        packet.aggregated = sig.aggregation;
        packet.stage = PacketStage::Data;
    }

    const auto samples_needed = WifiDataDecoder::sampleCount(packet.data);
    if (!samples_needed)
    {
        diagnose(packet, "Invalid OFDM PSDU length or DATA parameters", batch);
        return true;
    }
    if (packet.data.format == WifiPhyFormat::HtMixed)
    {
        // HT duration from the end of L-SIG is rounded up to 4 us. Checking
        // it against L-SIG also bounds how long the packet can retain IQ.
        const auto duration = *samples_needed - wifi::legacy_data_start;
        const auto symbols = (duration + wifi::symbol_samples - 1) / wifi::symbol_samples;
        if (packet.lsig_length != 3 * (symbols - 1))
        {
            diagnose(packet, "HT-SIG DATA duration disagrees with L-SIG length", batch);
            return true;
        }
    }
    if (iq.size() < *samples_needed)
        return false;

    const auto decoded = data_decoder_.decode(iq.first(*samples_needed), packet.cfo, packet.data);
    if (!decoded.error.empty())
        diagnose(packet, decoded.error, batch);
    else if (!packet.aggregated)
        parseMpdu(packet, decoded.psdu, batch);
    else
    {
        const auto ampdu = splitHtAmpdu(decoded.psdu);
        for (const auto mpdu : ampdu.mpdus)
            parseMpdu(packet, mpdu, batch);
        if (!ampdu.error.empty())
            diagnose(packet, ampdu.error, batch);
        else if (ampdu.mpdus.empty())
            diagnose(packet, "HT A-MPDU contains no MPDUs", batch);
    }

    return true;
}

void WiFiDemodulator::parseMpdu(const PendingPacket& packet, std::span<const uint8_t> mpdu, Snapshot& batch)
{
    auto parsed = MacFrameParser::parse(mpdu);
    if (!parsed.frame)
    {
        diagnose(packet, parsed.error, batch);
        return;
    }

    ++batch.verified_frames;
    batch.recent_frames.push_back({packet.segment, packet.sample_index, packet.data.format,
                                   packet.data.rate_or_mcs, std::move(*parsed.frame)});
}

void WiFiDemodulator::publish(Snapshot& batch)
{
    state_.processed_samples += batch.processed_samples;
    state_.resets += batch.resets;
    state_.candidates += batch.recent.size();
    state_.synchronized_packets += batch.recent_sync.size();
    state_.rejected_candidates += batch.rejected_candidates;
    state_.decoded_ht_sig += batch.decoded_ht_sig;
    state_.verified_frames += batch.verified_frames;
    appendHistory(state_.recent, batch.recent);
    appendHistory(state_.recent_sync, batch.recent_sync);
    appendHistory(state_.recent_ht, batch.recent_ht);
    appendHistory(state_.recent_frames, batch.recent_frames, frame_history_limit);
    appendHistory(state_.recent_diagnostics, batch.recent_diagnostics);
}

void WiFiDemodulator::diagnose(const PendingPacket& packet, std::string_view message, Snapshot& batch)
{
    batch.recent_diagnostics.push_back({packet.segment, packet.sample_index, std::string(message)});
}
