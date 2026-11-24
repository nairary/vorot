#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

using MacAddress = std::array<uint8_t, 6>;

enum class MacFrameType : uint8_t { Management, Control, Data };

struct MacFrame
{
    MacFrameType type{};
    uint8_t subtype = 0;
    bool to_ds = false;
    bool from_ds = false;
    std::optional<MacAddress> address1, address2, address3, address4;
    std::optional<MacAddress> receiver, transmitter, source, destination, bssid;
    std::optional<uint16_t> sequence_number, qos_control;
    std::optional<uint32_t> ht_control;
    size_t length_bytes = 0;  // includes the four-byte FCS
};

struct MacParseResult
{
    // Present only for a supported, structurally valid MPDU with a good FCS.
    std::optional<MacFrame> frame;
    std::string_view error;
};

class MacFrameParser
{
public:
    static MacParseResult parse(std::span<const uint8_t> mpdu);
};

struct AmpduResult
{
    // Each span borrows from the input PSDU. MAC parsing validates its FCS.
    std::vector<std::span<const uint8_t>> mpdus;
    std::string_view error;
};

AmpduResult splitHtAmpdu(std::span<const uint8_t> psdu);
std::string formatMacAddress(const MacAddress& address);
const char* macFrameTypeName(MacFrameType type);
const char* macFrameSubtypeName(MacFrameType type, uint8_t subtype);
