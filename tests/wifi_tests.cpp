#include "WiFiDemodulator.h"
#include "WifiBcc.h"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <numbers>
#include <random>
#include <sstream>
#include <stdexcept>

namespace {

using Sc16 = std::complex<int16_t>;
using Bytes = std::vector<uint8_t>;
std::string test_case;

void check(bool value, const char* expression, int line)
{
    if (!value)
        throw std::runtime_error(test_case + ", line " + std::to_string(line) + ": " + expression);
}
#define CHECK(...) check(static_cast<bool>((__VA_ARGS__)), #__VA_ARGS__, __LINE__)

Bytes unhex(const std::string& hex)
{
    CHECK(hex.size() % 2 == 0);
    Bytes bytes;
    for (size_t i = 0; i < hex.size(); i += 2)
        bytes.push_back(static_cast<uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));
    return bytes;
}

struct Packet
{
    std::string name;
    std::string fault;
    bool ht;
    unsigned rate;
    bool short_gi;
    std::vector<Sc16> iq;
    Bytes psdu;
    std::vector<Bytes> aggregate;
};

std::vector<Packet> readPackets(const std::filesystem::path& dir)
{
    std::ifstream iq_file(dir / "wifi20.sc16", std::ios::binary);
    std::ifstream rows(dir / "packets.txt");
    CHECK(iq_file && rows);
    const Bytes raw{std::istreambuf_iterator<char>(iq_file), std::istreambuf_iterator<char>()};
    std::vector<Packet> packets;
    size_t next = 0;
    std::string line;
    while (std::getline(rows, line))
    {
        Packet packet{};
        std::string phy, psdu, mpdus;
        size_t offset = 0, count = 0;
        std::istringstream row(line);
        CHECK(row >> packet.name >> phy >> packet.rate >> packet.short_gi >> offset >> count >>
              packet.fault >> psdu >> mpdus);
        CHECK(phy == "HT" || phy == "L");
        CHECK(offset == next && count > 192 && offset + count <= raw.size() / 4);
        packet.ht = phy == "HT";
        packet.psdu = unhex(psdu);
        packet.iq.reserve(count);
        const auto word = [&](size_t i) {
            return static_cast<int16_t>(raw[i] | (unsigned(raw[i + 1]) << 8));
        };
        for (size_t i = offset * 4; i < (offset + count) * 4; i += 4)
            packet.iq.emplace_back(word(i), word(i + 2));
        if (mpdus != "-")
        {
            std::istringstream aggregate(mpdus);
            for (std::string mpdu; std::getline(aggregate, mpdu, ',');)
                packet.aggregate.push_back(unhex(mpdu));
        }
        packets.push_back(std::move(packet));
        next = offset + count;
    }
    CHECK(!packets.empty() && next * 4 == raw.size());
    return packets;
}

const Packet& findPacket(const std::vector<Packet>& packets, std::string_view name)
{
    const auto it = std::ranges::find(packets, name, &Packet::name);
    CHECK(it != packets.end());
    return *it;
}

Bytes bitString(const std::string& bits)
{
    Bytes values;
    for (const char bit : bits)
    {
        CHECK(bit == '0' || bit == '1');
        values.push_back(static_cast<uint8_t>(bit - '0'));
    }
    return values;
}

void testCoding(const std::filesystem::path& dir)
{
    std::ifstream coding(dir / "bcc.txt");
    unsigned rate;
    std::string message, coded;
    size_t cases = 0;
    const wifi::BccRate rates[] = {wifi::BccRate::Half, wifi::BccRate::TwoThirds,
                                  wifi::BccRate::ThreeQuarters, wifi::BccRate::FiveSixths};
    while (coding >> rate >> message >> coded)
    {
        CHECK(rate < std::size(rates));
        test_case = "BCC rate " + std::to_string(rate);
        const auto expected = bitString(message);
        std::vector<double> soft;
        for (const auto bit : bitString(coded))
            soft.push_back(bit ? -1.0 : 1.0);
        soft[19] = -0.25 * soft[19]; // An erroneous observation with low confidence.
        CHECK(wifi::decodeBcc(soft, expected.size(), rates[rate]) == expected);
        CHECK(wifi::decodeBcc(std::span(soft).first(8), expected.size(), rates[rate]).empty());
        ++cases;
    }
    CHECK(cases == std::size(rates));

    std::ifstream scrambled(dir / "scrambler.txt");
    cases = 0;
    unsigned seed;
    std::string psdu, bits;
    while (scrambled >> seed >> psdu >> bits)
    {
        test_case = "scrambler seed " + std::to_string(seed);
        const auto expected = unhex(psdu);
        auto input = bitString(bits);
        CHECK(wifi::descramblePsdu(input, expected.size()) == expected);
        input[11] ^= 1;
        CHECK(wifi::descramblePsdu(input, expected.size()).empty());
        std::fill_n(input.begin(), 7, 0);
        CHECK(wifi::descramblePsdu(input, expected.size()).empty());
        ++cases;
    }
    CHECK(cases == 6);
    std::cout << "Four convolutional code rates and six scrambler seeds checked\n";
}

MacAddress address(uint8_t i) { return {0x02, 0x11, 0x22, 0x33, 0x44, i}; }

void testMac(const std::filesystem::path& dir, const std::vector<Packet>& packets)
{
    for (unsigned ds = 0; ds < 4; ++ds)
    {
        test_case = "Data DS flags " + std::to_string(ds);
        const auto& packet = findPacket(packets, "ht" + std::to_string(ds) + "_long");
        const auto parsed = MacFrameParser::parse(packet.psdu);
        CHECK(parsed.frame);
        const auto& mac = *parsed.frame;
        const auto base = static_cast<uint8_t>(40 + ds * 4);
        const auto a1 = address(base), a2 = address(base + 1), a3 = address(base + 2), a4 = address(base + 3);
        CHECK(mac.type == MacFrameType::Data && mac.subtype == 8);
        CHECK(mac.to_ds == ((ds & 1) != 0) && mac.from_ds == ((ds & 2) != 0));
        CHECK(mac.address1 == a1 && mac.address2 == a2 && mac.address3 == a3);
        CHECK(mac.receiver == a1 && mac.transmitter == a2);
        CHECK(mac.qos_control == 5 && mac.sequence_number == base);
        const MacAddress sources[] = {a2, a2, a3, a4};
        const MacAddress destinations[] = {a1, a3, a1, a3};
        const std::optional<MacAddress> bssids[] = {a3, a1, a2, std::nullopt};
        CHECK(mac.source == sources[ds] && mac.destination == destinations[ds] && mac.bssid == bssids[ds]);
        CHECK(mac.address4 == (ds == 3 ? std::optional{a4} : std::nullopt));
        CHECK(mac.length_bytes == packet.psdu.size());
    }

    test_case = "legacy management and data";
    const auto beacon = MacFrameParser::parse(findPacket(packets, "legacy12").psdu);
    CHECK(beacon.frame && beacon.frame->type == MacFrameType::Management && beacon.frame->subtype == 8);
    CHECK(beacon.frame->source == address(23) && beacon.frame->bssid == address(23));
    CHECK(beacon.frame->destination == MacAddress{255, 255, 255, 255, 255, 255});
    CHECK(beacon.frame->sequence_number == 0x123);
    const auto data = MacFrameParser::parse(findPacket(packets, "legacy6").psdu);
    CHECK(data.frame && data.frame->subtype == 0 && !data.frame->qos_control);
    CHECK(formatMacAddress(*data.frame->receiver) == "02:11:22:33:44:01");

    std::ifstream vectors(dir / "mac.txt");
    std::map<std::string, MacParseResult> results;
    for (std::string name, hex; vectors >> name >> hex;)
        results.emplace(name, MacFrameParser::parse(unhex(hex)));
    CHECK(results.size() == 21);

    test_case = "optional MAC fields";
    CHECK(results.at("ordered_qos").frame && results.at("ordered_qos").frame->ht_control == 0x78563412);
    CHECK(results.at("ordered_qos").frame->address4 == address(13));
    CHECK(results.at("management_htc").frame &&
          results.at("management_htc").frame->ht_control == 0x79453612);
    const auto& probe = results.at("probe_request").frame;
    CHECK(probe && !probe->bssid && probe->source == address(5));
    const auto& amsdu = results.at("amsdu").frame;
    CHECK(amsdu && !amsdu->source && !amsdu->destination && amsdu->bssid == address(20));

    for (const auto* name : {"rts", "cts", "ack", "ps_poll", "cf_end", "cf_end_ack", "bar",
                             "ba", "ba_basic", "multi_tid_bar", "multi_tid_ba", "wrapper"})
    {
        test_case = name;
        const auto& frame = results.at(name).frame;
        CHECK(frame && frame->type == MacFrameType::Control && frame->receiver == address(1));
        CHECK(!frame->sequence_number && !frame->source && !frame->destination);
    }
    test_case = "control address semantics";
    CHECK(results.at("rts").frame->transmitter == address(2));
    CHECK(!results.at("cts").frame->address2 && !results.at("ack").frame->transmitter);
    CHECK(results.at("ps_poll").frame->bssid == address(1) && results.at("ps_poll").frame->transmitter == address(2));
    CHECK(results.at("cf_end").frame->bssid == address(2) && !results.at("cf_end").frame->transmitter);
    CHECK(results.at("wrapper").frame->ht_control == 0x12345678 &&
          results.at("wrapper").frame->transmitter == address(2));
    for (const auto* name : {"short_qos", "short_rts", "short_ba", "short_wrapper", "unsupported"})
    {
        test_case = name;
        CHECK(!results.at(name).frame && !results.at(name).error.empty());
    }

    test_case = "bad FCS";
    const auto bad_fcs = MacFrameParser::parse(findPacket(packets, "bad_fcs").psdu);
    CHECK(!bad_fcs.frame && bad_fcs.error == "MAC FCS mismatch");

    test_case = "malformed HT delimiters";
    auto aggregate = findPacket(packets, "ampdu").psdu;
    CHECK(splitHtAmpdu(aggregate).mpdus.size() == 3);
    const Bytes zero_length_delimiter{0x00, 0x00, 0x14, 0x4e};
    const auto padding = splitHtAmpdu(zero_length_delimiter);
    CHECK(padding.mpdus.empty() && padding.error.empty());
    const auto truncated = splitHtAmpdu(std::span(aggregate).first(aggregate.size() - 1));
    CHECK(truncated.mpdus.size() == 2 && !truncated.error.empty());
    aggregate[2] ^= 1;
    const auto bad = splitHtAmpdu(aggregate);
    CHECK(bad.mpdus.empty() && bad.error == "Invalid HT A-MPDU delimiter");
    std::cout << "MAC addresses, optional headers, control layouts, delimiters and FCS checked\n";
}

std::string_view expectedDiagnostic(std::string_view fault)
{
    if (fault == "lsig") return "L-SIG parity failed";
    if (fault == "crc") return "HT-SIG CRC-8 failed";
    if (fault == "reserved") return "HT-SIG reserved bit must be 1";
    if (fault == "illegal_mcs") return "Invalid HT-SIG MCS for signaled bandwidth";
    if (fault == "bandwidth") return "HT40 bandwidth is unsupported";
    if (fault == "stbc") return "HT STBC is unsupported";
    if (fault == "ldpc") return "HT LDPC is unsupported";
    if (fault == "ess") return "HT extension spatial streams are unsupported";
    if (fault == "mimo") return "HT MIMO / multi-stream MCS is unsupported";
    if (fault == "qbpsk") return "Expected Q-BPSK on both HT-SIG symbols";
    if (fault == "duration") return "HT-SIG DATA duration disagrees with L-SIG length";
    if (fault == "htstf") return "HT-STF periodicity check failed";
    if (fault == "fcs") return "MAC FCS mismatch";
    return {};
}

void testPhy(const std::vector<Packet>& packets)
{
    LSIGDecoder legacy;
    HtSigDecoder ht;
    WifiDataDecoder data;
    size_t decoded = 0;
    for (const auto& packet : packets)
    {
        test_case = packet.name;
        std::vector<wifi::IqSample> iq;
        for (auto x : std::span(packet.iq).subspan(192))
            iq.emplace_back(x.real() / 32768.0, x.imag() / 32768.0);
        const auto lsig = legacy.decode(iq, 0);
        if (packet.fault == "lsig")
        {
            CHECK(!lsig.valid && lsig.error == expectedDiagnostic(packet.fault));
            continue;
        }
        CHECK(lsig.valid && lsig.rate_mbps == (packet.ht ? 6 : packet.rate));
        if (!packet.ht)
            CHECK(lsig.length_bytes == packet.psdu.size());
        if (!packet.ht && lsig.rate_mbps == 6)
            CHECK(ht.firstSymbolAxis(iq, 0) == OfdmSignalAxis::InPhase);
        if (packet.ht)
        {
            CHECK(lsig.length_bytes % 3 == 0);
            CHECK(ht.firstSymbolAxis(iq, 0) == OfdmSignalAxis::Quadrature);
            const auto sig = ht.decode(iq, 0);
            if (!sig.supported_siso_ht20 && packet.fault != "-")
            {
                CHECK(sig.error == expectedDiagnostic(packet.fault));
                continue;
            }
            CHECK(sig.valid && sig.supported_siso_ht20);
            CHECK(sig.mcs == packet.rate && sig.short_gi == packet.short_gi);
            CHECK(sig.aggregation == !packet.aggregate.empty() && sig.length_bytes == packet.psdu.size());
        }

        const WifiDataParameters params{packet.ht ? WifiPhyFormat::HtMixed : WifiPhyFormat::LegacyOfdm,
                                       packet.rate, packet.psdu.size(), packet.short_gi};
        CHECK(WifiDataDecoder::sampleCount(params) == iq.size());
        auto psdu = data.decode(iq, 0, params);
        if (packet.fault == "htstf")
        {
            CHECK(psdu.error == expectedDiagnostic(packet.fault) && psdu.psdu.empty());
            continue;
        }
        if (!psdu.error.empty())
            std::cerr << psdu.error << '\n';
        CHECK(psdu.error.empty());
        CHECK(psdu.psdu == packet.psdu);
        const auto too_short = data.decode(std::span(iq).first(iq.size() - 1), 0, params);
        CHECK(too_short.psdu.empty() && !too_short.error.empty());
        ++decoded;

        if (!packet.aggregate.empty())
        {
            const auto split = splitHtAmpdu(psdu.psdu);
            CHECK(split.error.empty() && split.mpdus.size() == packet.aggregate.size());
            for (size_t i = 0; i < split.mpdus.size(); ++i)
            {
                CHECK(std::ranges::equal(split.mpdus[i], packet.aggregate[i]));
                const auto mpdu = MacFrameParser::parse(split.mpdus[i]);
                CHECK(mpdu.frame.has_value() == !(packet.fault == "fcs" && i == 1));
            }
        }
    }
    CHECK(decoded == 28);
    std::cout << decoded << " PSDUs compared byte for byte across " << packets.size() << " reference PPDUs\n";
}

WiFiDemodulator::Snapshot waitForSamples(const WiFiDemodulator& receiver, uint64_t samples)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    for (;;)
    {
        auto status = receiver.snapshot();
        CHECK(status.enabled && status.error.empty() && status.queue.overflows == 0);
        if (status.processed_samples == samples)
            return status;
        CHECK(status.processed_samples < samples && std::chrono::steady_clock::now() < deadline);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void submitBlocks(WiFiDemodulator& receiver, std::span<const Sc16> stream, uint64_t first, uint64_t segment)
{
    constexpr size_t chunks[] = {1, 3, 127, 2, 239, 512, 71, 1135, 17, 4096};
    size_t block = 0;
    while (!stream.empty())
    {
        const size_t n = std::min(stream.size(), chunks[block++ % std::size(chunks)]);
        receiver.submit(stream.first(n), first, segment, false);
        stream = stream.subspan(n);
        first += n;
    }
}

std::vector<Sc16> impaired(std::span<const Sc16> iq, size_t delay, double cfo, double noise_stddev, unsigned seed)
{
    const size_t trailing = 560;
    std::vector<Sc16> stream(delay + iq.size() + trailing);
    std::mt19937 rng(seed);
    std::normal_distribution<double> noise(0, noise_stddev);
    const auto taps = std::complex<double>(0.18, 0.12);
    for (size_t n = 0; n < stream.size(); ++n)
    {
        const auto signal = [&](size_t sample) -> std::complex<double> {
            if (sample < delay || sample >= delay + iq.size())
                return {};
            const auto x = iq[sample - delay];
            return {double(x.real()), double(x.imag())};
        };
        const auto echo = n >= 3 ? taps * signal(n - 3) : std::complex<double>{};
        const auto phase = std::polar(1.0, 0.42 + 2 * std::numbers::pi * cfo * double(n) / 20e6);
        const auto x = phase * (signal(n) + echo) + std::complex<double>(noise(rng), noise(rng));
        CHECK(std::abs(x.real()) < 32767 && std::abs(x.imag()) < 32767);
        stream[n] = Sc16(static_cast<int16_t>(std::lround(x.real())), static_cast<int16_t>(std::lround(x.imag())));
    }
    return stream;
}

void testStream(const std::vector<Packet>& packets)
{
    WiFiDemodulator receiver(256);
    receiver.enable(true, 19e6);
    CHECK(!receiver.snapshot().enabled && !receiver.snapshot().error.empty());
    receiver.enable(true, 20e6);

    uint64_t total = 0, first = 90000;
    uint64_t verified = 0, segment = 7;
    for (size_t i = 0; i < packets.size(); ++i)
    {
        const auto& packet = packets[i];
        test_case = packet.name + " over split IQ";
        const size_t delay = 13 + 19 * (i % 11);
        const double cfo = i % 2 == 0 ? -124000.0 : 82000.0;
        const double noise = packet.name == "legacy6" || packet.rate == 0 ? 48.0 : 13.0;
        const auto stream = impaired(packet.iq, delay, cfo, noise, static_cast<unsigned>(i + 7));
        submitBlocks(receiver, stream, first, segment);
        total += stream.size();
        const auto result = waitForSamples(receiver, total);

        size_t count = 0;
        if (packet.fault == "-")
            count = packet.aggregate.empty() ? 1 : packet.aggregate.size();
        else if (packet.name == "ampdu_bad_fcs")
            count = 2;
        if (result.verified_frames != verified + count)
        {
            std::cerr << "Candidates: " << result.candidates << ", rejected: " << result.rejected_candidates << '\n';
            for (const auto& event : result.recent)
                std::cerr << event.sample_index << ": L-STF CFO " << event.cfo_hz << '\n';
            for (const auto& diagnostic : result.recent_diagnostics)
                std::cerr << diagnostic.sample_index << ": " << diagnostic.message << '\n';
            for (const auto& sync : result.recent_sync)
                std::cerr << sync.packet_start_index << ": L-SIG " << sync.lsig_result.error << '\n';
        }
        CHECK(result.verified_frames == verified + count);
        const auto reported = std::span(result.recent_frames).last(count);
        for (const auto& frame : reported)
        {
            CHECK(frame.sample_index == first + delay && frame.segment == segment);
            CHECK(frame.rate_or_mcs == packet.rate);
            CHECK(frame.format == (packet.ht ? WifiPhyFormat::HtMixed : WifiPhyFormat::LegacyOfdm));
        }
        if (count && packet.aggregate.empty())
        {
            const auto known = MacFrameParser::parse(packet.psdu);
            CHECK(known.frame && reported.front().mac.receiver == known.frame->receiver);
        }
        if (count && !packet.aggregate.empty())
        {
            CHECK(reported.front().mac.receiver == address(100));
            CHECK(reported.back().mac.receiver == address(108));
        }
        if (!packet.ht)
            CHECK(std::ranges::none_of(result.recent_ht, [&](const HtSigEvent& ht) {
                return ht.ltf_first_index == first + delay + 192 && ht.segment == segment;
            }));
        if (packet.fault == "lsig")
            CHECK(!result.recent_sync.empty() && result.recent_sync.back().lsig_result.error == expectedDiagnostic("lsig"));
        else if (packet.fault != "-")
            CHECK(std::ranges::any_of(result.recent_diagnostics, [&](const auto& d) {
                return d.segment == segment && d.sample_index == first + delay &&
                       d.message == expectedDiagnostic(packet.fault);
            }));

        verified += count;
        first += stream.size();
    }
    CHECK(receiver.snapshot().resets == 1);

    test_case = "discontinuous IQ";
    const auto& long_frame = findPacket(packets, "legacy6").iq;
    receiver.submit(std::span(long_frame).first(760), first, segment, false);
    total += 760;
    CHECK(waitForSamples(receiver, total).verified_frames == verified);
    const std::vector<Sc16> silence(600);
    receiver.submit(silence, 0, ++segment, true);
    total += silence.size();
    const auto result = waitForSamples(receiver, total);
    CHECK(result.verified_frames == verified && result.resets == 2);
    CHECK(result.recent_diagnostics.back().message == "IQ discontinuity interrupted the packet");

    test_case = "bounded verified frame history";
    const auto& ack = findPacket(packets, "legacy9").iq;
    std::vector<Sc16> burst;
    constexpr size_t repetitions = WiFiDemodulator::frame_history_limit + 4;
    for (size_t i = 0; i < repetitions; ++i)
    {
        burst.insert(burst.end(), ack.begin(), ack.end());
        burst.resize(burst.size() + 96);
    }
    receiver.submit(burst, silence.size(), segment, false);
    total += burst.size();
    const auto latest = waitForSamples(receiver, total);
    CHECK(latest.verified_frames == verified + repetitions);
    CHECK(latest.recent_frames.size() == WiFiDemodulator::frame_history_limit);
    CHECK(latest.recent_frames.back().sample_index == silence.size() + (repetitions - 1) * (ack.size() + 96));
    CHECK(latest.recent_frames.front().sample_index == silence.size() + 4 * (ack.size() + 96));
    receiver.enable(false, 20e6);
    CHECK(receiver.snapshot().recent_frames.empty());
    std::cout << packets.size() << " impaired PPDUs checked through the receive worker, plus gap and history tests\n";
}

}

int main(int argc, char** argv)
{
    try
    {
        CHECK(argc == 3);
        const std::string_view suite = argv[1];
        const std::filesystem::path dir = argv[2];
        test_case = std::string(suite);
        if (suite == "coding")
            testCoding(dir);
        else
        {
            const auto packets = readPackets(dir);
            if (suite == "mac") testMac(dir, packets);
            else if (suite == "phy") testPhy(packets);
            else if (suite == "stream") testStream(packets);
            else CHECK(false);
        }
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
