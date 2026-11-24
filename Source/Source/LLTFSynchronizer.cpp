#include "LLTFSynchronizer.h"

void LLTFSynchronizer::reset(uint64_t first)
{
	history_.clear();
	pending_.clear();
	pending_ht_.clear();
	first_ = first;
}

void LLTFSynchronizer::append(std::span<const std::complex<int16_t>> samples)
{
	for (auto x : samples)
	{
		history_.emplace_back(x.real() / 32768.0, x.imag() / 32768.0);
	}
}

void LLTFSynchronizer::candidate(const LStfEvent& event)
{
	pending_.push_back(event);
}

std::vector<LLTFSyncEvent> LLTFSynchronizer::process(uint64_t segment, uint64_t& rejected, std::vector<HtSigEvent>& ht_events)
{
	std::vector<LLTFSyncEvent> result;

	while (!pending_.empty())
	{
		const auto event = pending_.front();
		// ищем полезный LTF между кандидат+128 и кандидат+288
		const uint64_t begin = std::max<uint64_t>(192, event.sample_index + 128); // 192 - это 160 сэмплов на L-STF и 32 сэмпла на защитный интервал
		const uint64_t end = event.sample_index + 288;
		// Ждём не только 2 ltf символа общей длиной 128, но и одного lsig длиной 80
		if (first_ + history_.size() < end + 128 + 80) break;

		pending_.pop_front();
		double best = 0;
		uint64_t bestIndex = 0;

		for (uint64_t m = begin; m < end; ++m)
		{
			if (m < first_ + 32) continue; // первые 32 отсчёта - защитный интервал
			const auto a = match(m, event.cfo_hz);
			const auto b = match(m + 64, event.cfo_hz);
			const double repeat = repeatScore(m, m + 64, 64);
			const double cp = repeatScore(m - 32, m + 32, 32);
			const double score = std::min(a, b);

			if (repeat >= 0.75 && cp > 0.65 && score > best)
			{
				best = score;
				bestIndex = m;
			}
		}
		if (best < 0.55) { ++rejected; continue; }

		std::complex<double> r{};
		for (size_t k = 0; k < 64; ++k)
		{
			r += std::conj(at(bestIndex + k)) * at(bestIndex + 64 + k);
		}

		// удаляем уже известную грубую расфазировку
		r *= std::polar(1.0, -twoPi * event.cfo_hz * 64 / sampleRate);
		const double fine = std::arg(r) * sampleRate / (twoPi * 64);

		std::array<std::complex<double>, 208> window;

		for (size_t i = 0; i < window.size(); ++i)
		{
			window[i] = at(bestIndex + i);
		}

		const LSigResult lsig = lsig_decoder_.decode(window, event.cfo_hz + fine, sampleRate);

		result.push_back(
			{
				segment,
				event.sample_index,
				bestIndex,
				bestIndex - 192,
				best,
				event.cfo_hz,
				fine,
				event.cfo_hz + fine,
				lsig
			}
		);
		if (lsig.valid && lsig.rate_mbps == 6)
		{
			pending_ht_.push_back(
				{
					segment,
					bestIndex,
					event.cfo_hz + fine
				}
			);
		}
	}

	while (!pending_ht_.empty())
	{
		const auto pending = pending_ht_.front();
		const uint64_t m = pending.ltf_first_index;

		if (m < first_)
		{
			pending_ht_.pop_front();
			continue;
		}

		// Ждём конца второго HT-SIG
		if (first_ + history_.size() < m + 368)
			break;

		pending_ht_.pop_front();
		std::array<std::complex<double>, 368> window{};
		for (size_t i = 0; i < window.size(); ++i)
			window[i] = at(m + i);
		
		auto decoded = htsig_decoder_.decode(window, pending.total_cfo_hz);

		if (decoded.valid)
		{
			ht_events.push_back({
				pending.segment,
				m,
				std::move(decoded)
				});
		}
	}

	while (history_.size() > 1024)
	{
		history_.pop_front();
		++first_;
	}
	return result;
}

std::complex<double> LLTFSynchronizer::at(uint64_t index) const
{
	return history_.at(static_cast<size_t>(index - first_));
}

double LLTFSynchronizer::match(uint64_t index, double cfo) const
{
	std::complex<double> r{}; // корреляция с эталоном 
	double power = 0;
	const auto step = std::polar(1.0, -twoPi * cfo / sampleRate );
	std::complex<double> phase = 1;
	for (size_t k = 0; k < 64; ++k) // 64 - это длина одного символа l-ltf
	{
		const auto x = at(index + k);
		r += x * phase * std::conj(reference_[k]);
		power += std::norm(x);
		phase *= step;
	}

	return power > 1e-20 ? std::clamp(std::norm(r) / power / (52.0 / 64), 0.0, 1.0) : 0;
}

double LLTFSynchronizer::repeatScore(uint64_t a, uint64_t b, size_t count) const
{
	std::complex<double> r{};
	double p = 0;
	double q = 0;
	for (size_t k = 0; k < count; ++k)
	{
		auto x = at(a + k);
		auto y = at(b + k);
		r += std::conj(x) * y;
		p += std::norm(x);
		q += std::norm(y);
	}
	return p * q > 1e-24 ? std::clamp(std::norm(r) / (p * q), 0.0, 1.0) : 0;
}
