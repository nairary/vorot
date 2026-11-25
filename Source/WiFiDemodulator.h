#pragma once

#include "HTSIGDecoder.h"
#include "IqQueue.h"
#include "LLTFSynchronizer.h"
#include "MacFrameParser.h"
#include "WifiDataDecoder.h"

#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

const int FFT_SIZE = 64;
const int CP_SIZE = 16;
const int OFDM_SYMBOL_SIZE = 80;
const int STS_LENGTH = 16;

std::vector<uint8_t> demodulate_bpsk(const std::vector<std::complex<float>>& subcarriers);
void process_phy_demodulation(const std::vector<std::complex<int16_t>>& iq_buffer);

struct DecodedWifiFrame
{
    uint64_t segment;
    uint64_t sample_index;
    WifiPhyFormat format;
    unsigned rate_or_mcs;
    MacFrame mac;
};

struct WifiPacketDiagnostic
{
    uint64_t segment;
    uint64_t sample_index;
    std::string message;
};

class WiFiDemodulator
{
public:
    static constexpr size_t frame_history_limit = 64;

    struct Snapshot
    {
        bool enabled = false;
        uint64_t candidates = 0;
        uint64_t resets = 0;
        uint64_t processed_samples = 0;
        uint64_t synchronized_packets = 0;
        uint64_t rejected_candidates = 0;
        uint64_t decoded_ht_sig = 0;
        uint64_t verified_frames = 0;

        IqQueue::Stats queue;
        std::vector<LStfEvent> recent;
        std::vector<LLTFSyncEvent> recent_sync;
        std::vector<HtSigEvent> recent_ht;
        std::vector<DecodedWifiFrame> recent_frames;
        std::vector<WifiPacketDiagnostic> recent_diagnostics;
        std::string error;
    };

    explicit WiFiDemodulator(size_t queue_capacity = 32);
    ~WiFiDemodulator();
    void enable(bool enabled, double actual_rate_hz);
    void submit(std::span<const std::complex<int16_t>> samples, uint64_t first, uint64_t segment, bool gap);
    Snapshot snapshot() const;

private:
    enum class PacketStage { IdentifyFormat, HtSignal, Data };

    struct PendingPacket
    {
        uint64_t segment;
        uint64_t sample_index;
        uint64_t ltf_first_index;
        double cfo;
        unsigned lsig_length;
        WifiDataParameters data;
        PacketStage stage;
        bool aggregated = false;
    };

    void run();
    void processPart(std::span<const std::complex<int16_t>> samples, uint64_t segment, Snapshot& batch);
    bool advancePacket(PendingPacket& packet, Snapshot& batch);
    void parseMpdu(const PendingPacket& packet, std::span<const uint8_t> mpdu, Snapshot& batch);
    void publish(Snapshot& batch);
    static void diagnose(const PendingPacket& packet, std::string_view message, Snapshot& batch);

    IqQueue queue_;
    mutable std::mutex mutex_;
    Snapshot state_;
    uint64_t generation_ = 0;

    LStfDetector detector_;
    LLTFSynchronizer synchronizer_;
    HtSigDecoder htsig_decoder_;
    WifiDataDecoder data_decoder_;
    std::deque<PendingPacket> packets_;
    std::jthread worker_;
};
