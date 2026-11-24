#include "LLTFSynchronizer.h"

#include <algorithm>
#include <numbers>

namespace {

constexpr double tau = 2 * std::numbers::pi;
constexpr size_t ltf_offset = 192; // 160 L-STF samples, then 32 L-LTF guard samples.

const auto reference = [] {
    wifi::Spectrum samples{};
    for (size_t n = 0; n < samples.size(); ++n)
        for (int k = -26; k <= 26; ++k)
            samples[n] += double(wifi::legacy_ltf[k + 26]) *
                          std::polar(1.0, tau * k * double(n) / wifi::fft_size) / double(wifi::fft_size);
    return samples;
}();

}

void LLTFSynchronizer::reset(uint64_t first)
{
    history_.clear();
    pending_.clear();
    last_ltf_.reset();
    first_ = first;
}

void LLTFSynchronizer::append(std::span<const std::complex<int16_t>> samples)
{
    for (auto x : samples)
        history_.emplace_back(x.real() / 32768.0, x.imag() / 32768.0);
}

void LLTFSynchronizer::candidate(const LStfEvent& event)
{
    pending_.push_back(event);
}

std::vector<LLTFSyncEvent> LLTFSynchronizer::process(uint64_t segment, uint64_t& rejected)
{
    std::vector<LLTFSyncEvent> result;
    while (!pending_.empty())
    {
        const auto event = pending_.front();
        const auto begin = std::max<uint64_t>({ltf_offset, first_ + 32, event.sample_index + 128});
        const auto end = event.sample_index + 288;
        if (endIndex() < end + wifi::legacy_data_start)
            break;

        pending_.pop_front();
        double best = 0;
        uint64_t best_index = 0;
        for (uint64_t m = begin; m < end; ++m)
        {
            const double score = std::min(match(m, event.cfo_hz), match(m + 64, event.cfo_hz));
            if (score > best && repeatScore(m, m + 64, 64) >= 0.75 &&
                repeatScore(m - 32, m + 32, 32) > 0.65)
            {
                best = score;
                best_index = m;
            }
        }
        if (best < 0.55)
        {
            ++rejected;
            continue;
        }
        if (last_ltf_ == best_index)
            continue;
        last_ltf_ = best_index;

        const auto window = samplesFrom(best_index);
        wifi::IqSample repeat{};
        for (size_t k = 0; k < 64; ++k)
            repeat += std::conj(window[k]) * window[k + 64];

        repeat *= std::polar(1.0, -tau * event.cfo_hz * 64 / wifi::sample_rate_hz);
        const double fine = std::arg(repeat) * wifi::sample_rate_hz / (tau * 64);
        const double cfo = event.cfo_hz + fine;
        result.push_back({segment, event.sample_index, best_index, best_index - ltf_offset, best,
                          event.cfo_hz, fine, cfo, lsig_decoder_.decode(window, cfo)});
    }
    return result;
}

std::span<const wifi::IqSample> LLTFSynchronizer::samplesFrom(uint64_t index) const
{
    if (index < first_ || index > endIndex())
        return {};
    return std::span(history_).subspan(static_cast<size_t>(index - first_));
}

void LLTFSynchronizer::discardBefore(uint64_t earliest_packet)
{
    // Retain the preamble search window, even when no packet is pending.
    constexpr size_t search_history = 512;
    if (history_.size() <= search_history)
        return;
    const auto keep = std::min(earliest_packet, endIndex() - search_history);
    // Compact in batches so a quiet channel does not move the search window
    // for every small chunk.
    if (keep >= first_ + 4096)
    {
        history_.erase(history_.begin(), history_.begin() + static_cast<std::ptrdiff_t>(keep - first_));
        first_ = keep;
    }
}

double LLTFSynchronizer::match(uint64_t index, double cfo) const
{
    wifi::IqSample correlation{};
    double power = 0;
    const auto step = std::polar(1.0, -tau * cfo / wifi::sample_rate_hz);
    wifi::IqSample phase = 1;
    const auto samples = samplesFrom(index);
    for (size_t k = 0; k < reference.size(); ++k)
    {
        correlation += samples[k] * phase * std::conj(reference[k]);
        power += std::norm(samples[k]);
        phase *= step;
    }
    return power > 1e-20 ? std::clamp(std::norm(correlation) / power / (52.0 / 64), 0.0, 1.0) : 0;
}

double LLTFSynchronizer::repeatScore(uint64_t a, uint64_t b, size_t count) const
{
    const auto first = samplesFrom(a);
    const auto second = samplesFrom(b);
    wifi::IqSample correlation{};
    double first_power = 0, second_power = 0;
    for (size_t k = 0; k < count; ++k)
    {
        correlation += std::conj(first[k]) * second[k];
        first_power += std::norm(first[k]);
        second_power += std::norm(second[k]);
    }
    const double energy = first_power * second_power;
    return energy > 1e-24 ? std::clamp(std::norm(correlation) / energy, 0.0, 1.0) : 0;
}
