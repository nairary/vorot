#pragma once

#include "LStfDetector.h"
#include "LSIGDecoder.h"

#include <deque>
#include <optional>
#include <vector>

struct LLTFSyncEvent
{
    uint64_t segment = 0;
    uint64_t candidate_index = 0;
    uint64_t ltf_first_index = 0;
    uint64_t packet_start_index = 0;
    double correlation = 0;
    double coarse_cfo_hz = 0;
    double fine_cfo_hz = 0;
    double total_cfo_hz = 0;
    LSigResult lsig_result;
};

class LLTFSynchronizer
{
public:
    void reset(uint64_t first);
    void append(std::span<const std::complex<int16_t>> samples);
    void candidate(const LStfEvent& event);
    std::vector<LLTFSyncEvent> process(uint64_t segment, uint64_t& rejected);

    // These samples are contiguous, and the view is valid until the next append,
    // discard, or reset. Keep in-flight PPDUs pinned until their DATA is complete.
    std::span<const wifi::IqSample> samplesFrom(uint64_t index) const;
    void discardBefore(uint64_t earliest_packet);
    uint64_t endIndex() const { return first_ + history_.size(); }

private:
    double match(uint64_t index, double cfo) const;
    double repeatScore(uint64_t a, uint64_t b, size_t count) const;

    std::vector<wifi::IqSample> history_;
    std::deque<LStfEvent> pending_;
    std::optional<uint64_t> last_ltf_;
    uint64_t first_ = 0;
    LSIGDecoder lsig_decoder_;
};
