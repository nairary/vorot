#include "MacFrameParser.h"

#include <algorithm>

namespace {

uint16_t read16(std::span<const uint8_t> bytes, size_t offset)
{
    return static_cast<uint16_t>(bytes[offset] | (unsigned(bytes[offset + 1]) << 8));
}

uint32_t read32(std::span<const uint8_t> bytes, size_t offset)
{
    return uint32_t{read16(bytes, offset)} | (uint32_t{read16(bytes, offset + 2)} << 16);
}

MacAddress readAddress(std::span<const uint8_t> bytes, size_t offset)
{
    MacAddress address{};
    std::ranges::copy(bytes.subspan(offset, address.size()), address.begin());
    return address;
}

uint32_t crc32(std::span<const uint8_t> bytes)
{
    uint32_t crc = 0xffffffffu;
    for (auto byte : bytes)
    {
        crc ^= byte;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ (crc & 1u ? 0xedb88320u : 0);
    }
    return ~crc;
}

std::string_view parseControl(std::span<const uint8_t> bytes, MacFrame& frame)
{
    constexpr size_t common_length = 10;  // FC, Duration/ID, RA
    auto subtype = frame.subtype;
    size_t address2_offset = common_length;

    if (subtype == 7) // Control Wrapper
    {
        if (bytes.size() < 16)
            return "Truncated Control Wrapper";
        const auto carried_fc = read16(bytes, common_length);
        if ((carried_fc & 0x0f) != 0x04 || ((carried_fc >> 4) & 0x0f) == 7)
            return "Invalid carried control frame";
        subtype = static_cast<uint8_t>((carried_fc >> 4) & 0x0f);
        frame.ht_control = read32(bytes, 12);
        address2_offset += 6;
    }

    size_t length = address2_offset;
    switch (subtype)
    {
    case 12: // CTS
    case 13: // ACK
        break;
    case 10: // PS-Poll
    case 11: // RTS
    case 14: // CF-End
    case 15: // CF-End + CF-Ack
        length += 6;
        break;
    case 8:  // Block Ack Request
    case 9:  // Block Ack
    {
        if (bytes.size() < length + 8)
            return "Truncated Block Ack control field";
        const auto control = read16(bytes, address2_offset + 6);
        const auto variant = (control >> 1) & 7u;
        length += 8; // TA and BAR/BA Control
        if (variant == 0 || variant == 2)
        {
            length += 2; // Starting Sequence Control
            if (subtype == 9)
                length += variant == 2 ? 8 : 128;
        }
        else if (variant == 3)
        {
            const unsigned tids = (control >> 12) + 1;
            length += tids * (subtype == 9 ? 12 : 4);
        }
        else
            return "Unsupported Block Ack variant";
        break;
    }
    default:
        return "Unsupported control frame subtype";
    }

    if (bytes.size() != length)
        return "Control frame length does not match its subtype";

    if (length > address2_offset)
        frame.address2 = readAddress(bytes, address2_offset);
    if (subtype == 14 || subtype == 15)
        frame.bssid = frame.address2;
    else
        frame.transmitter = frame.address2;
    if (subtype == 10)
        frame.bssid = frame.address1;
    return {};
}

std::string_view parseManagementOrData(std::span<const uint8_t> bytes, MacFrame& frame, bool ordered)
{
    const bool data = frame.type == MacFrameType::Data;
    const bool qos = data && (frame.subtype & 8u);
    const bool four_addresses = data && frame.to_ds && frame.from_ds;
    const bool ht_control = ordered && (!data || qos);
    const size_t length = 24 + (four_addresses ? 6 : 0) + (qos ? 2 : 0) + (ht_control ? 4 : 0);
    if (bytes.size() < length)
        return "Truncated management or data header";
    if (!data && (frame.to_ds || frame.from_ds))
        return "Unexpected DS flags on a management frame";

    frame.address2 = readAddress(bytes, 10);
    frame.address3 = readAddress(bytes, 16);
    frame.transmitter = frame.address2;
    frame.sequence_number = read16(bytes, 22) >> 4;
    size_t offset = 24;
    if (four_addresses)
    {
        frame.address4 = readAddress(bytes, offset);
        offset += 6;
    }
    if (qos)
    {
        frame.qos_control = read16(bytes, offset);
        offset += 2;
    }
    if (ht_control)
        frame.ht_control = read32(bytes, offset);

    if (four_addresses)
    {
        frame.source = frame.address4;
        frame.destination = frame.address3;
    }
    else if (frame.to_ds)
    {
        frame.source = frame.address2;
        frame.destination = frame.address3;
        frame.bssid = frame.address1;
    }
    else if (frame.from_ds)
    {
        frame.source = frame.address3;
        frame.destination = frame.address1;
        frame.bssid = frame.address2;
    }
    else
    {
        frame.source = frame.address2;
        frame.destination = frame.address1;
        frame.bssid = frame.address3;
    }

    // A-MSDU subframes carry individual MSDU source and destination addresses.
    if (qos && (*frame.qos_control & 0x0080))
    {
        frame.source.reset();
        frame.destination.reset();
    }
    return {};
}

uint8_t delimiterCrc(uint16_t field)
{
    // Bits are sent LSB first within each octet, including the CRC octet.
    // Use the reflected x^8 + x^2 + x + 1 polynomial.
    uint8_t crc = 0xff;
    for (unsigned bit = 0; bit < 16; ++bit)
    {
        const bool feedback = ((crc ^ (field >> bit)) & 1u) != 0;
        crc >>= 1;
        if (feedback)
            crc ^= 0xe0;
    }
    return static_cast<uint8_t>(~crc);
}

}

MacParseResult MacFrameParser::parse(std::span<const uint8_t> mpdu)
{
    constexpr size_t fcs_size = 4;
    if (mpdu.size() < 10 + fcs_size)
        return {{}, "Truncated MPDU"};
    const auto body = mpdu.first(mpdu.size() - fcs_size);
    if (crc32(body) != read32(mpdu, body.size()))
        return {{}, "MAC FCS mismatch"};

    const auto fc = read16(body, 0);
    if ((fc & 3u) != 0)
        return {{}, "Unsupported MAC protocol version"};
    const auto type = (fc >> 2) & 3u;
    if (type > static_cast<unsigned>(MacFrameType::Data))
        return {{}, "Unsupported MAC frame type"};

    MacFrame frame;
    frame.type = static_cast<MacFrameType>(type);
    frame.subtype = static_cast<uint8_t>((fc >> 4) & 0x0f);
    frame.to_ds = (fc & 0x0100) != 0;
    frame.from_ds = (fc & 0x0200) != 0;
    frame.address1 = readAddress(body, 4);
    frame.receiver = frame.address1;
    frame.length_bytes = mpdu.size();

    const auto error = frame.type == MacFrameType::Control ? parseControl(body, frame)
                                                          : parseManagementOrData(body, frame, (fc & 0x8000) != 0);
    if (!error.empty())
        return {{}, error};
    if (frame.bssid && ((frame.bssid->front() & 1u) || *frame.bssid == MacAddress{}))
        frame.bssid.reset(); // Keep only an individual BSSID.
    return {std::move(frame), {}};
}

AmpduResult splitHtAmpdu(std::span<const uint8_t> psdu)
{
    constexpr size_t delimiter_size = 4;
    AmpduResult result;
    while (!psdu.empty())
    {
        if (psdu.size() < delimiter_size)
        {
            result.error = "Truncated HT A-MPDU delimiter";
            break;
        }
        const auto field = read16(psdu, 0);
        if (psdu[3] != 0x4e || (field & 0x0f) != 0 || psdu[2] != delimiterCrc(field))
        {
            result.error = "Invalid HT A-MPDU delimiter";
            break;
        }

        const size_t length = field >> 4;
        psdu = psdu.subspan(delimiter_size);
        if (length > psdu.size())
        {
            result.error = "Truncated HT A-MPDU subframe";
            break;
        }
        if (length)
            result.mpdus.push_back(psdu.first(length));
        if (length == psdu.size())
            break; // The last MPDU can omit its alignment padding.

        const size_t aligned_length = (length + 3) & ~size_t{3};
        if (aligned_length > psdu.size())
        {
            result.error = "Truncated HT A-MPDU alignment padding";
            break;
        }
        psdu = psdu.subspan(aligned_length);
    }
    return result;
}

std::string formatMacAddress(const MacAddress& address)
{
    constexpr char digits[] = "0123456789ABCDEF";
    std::string text(17, ':');
    for (size_t i = 0; i < address.size(); ++i)
    {
        text[3 * i] = digits[address[i] >> 4];
        text[3 * i + 1] = digits[address[i] & 0x0f];
    }
    return text;
}

const char* macFrameTypeName(MacFrameType type)
{
    switch (type)
    {
    case MacFrameType::Management: return "Management";
    case MacFrameType::Control: return "Control";
    case MacFrameType::Data: return "Data";
    }
    return "Unknown";
}

const char* macFrameSubtypeName(MacFrameType type, uint8_t subtype)
{
    constexpr const char* management[] = {
        "Association Request", "Association Response", "Reassociation Request", "Reassociation Response",
        "Probe Request", "Probe Response", "Timing Advertisement", "Reserved",
        "Beacon", "ATIM", "Disassociation", "Authentication",
        "Deauthentication", "Action", "Action No Ack", "Reserved"
    };
    constexpr const char* control[] = {
        "Reserved", "Reserved", "Trigger", "TACK", "Beamforming Report Poll", "VHT/HE NDP Announcement",
        "Control Extension", "Control Wrapper", "Block Ack Request", "Block Ack", "PS-Poll",
        "RTS", "CTS", "ACK", "CF-End", "CF-End + CF-Ack"
    };
    constexpr const char* data[] = {
        "Data", "Data + CF-Ack", "Data + CF-Poll", "Data + CF-Ack + CF-Poll",
        "Null", "CF-Ack", "CF-Poll", "CF-Ack + CF-Poll",
        "QoS Data", "QoS Data + CF-Ack", "QoS Data + CF-Poll", "QoS Data + CF-Ack + CF-Poll",
        "QoS Null", "Reserved", "QoS CF-Poll", "QoS CF-Ack + CF-Poll"
    };
    if (subtype >= 16)
        return "Unknown";
    switch (type)
    {
    case MacFrameType::Management: return management[subtype];
    case MacFrameType::Control: return control[subtype];
    case MacFrameType::Data: return data[subtype];
    }
    return "Unknown";
}
