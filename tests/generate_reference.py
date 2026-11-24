"""Regenerate offline test vectors using cloud9477/gr-ieee80211 at add75653e8de63361b9d3d34606eb8dc5b6029c5."""
import argparse
from pathlib import Path
import struct
import sys
from unittest.mock import patch
import zlib

import numpy as np


def with_fcs(body):
    return body + struct.pack("<I", zlib.crc32(body))


def address(number):
    return bytes((0x02, 0x11, 0x22, 0x33, 0x44, number))


def data_mpdu(number, payload_length, ds=0, qos=False):
    fc = 0x0088 if qos else 0x0008
    header = struct.pack("<HH", fc | ds << 8, 0)
    header += address(number) + address(number + 1) + address(number + 2)
    header += struct.pack("<H", (number << 4) | 1)
    if ds == 3:
        header += address(number + 3)
    if qos:
        header += struct.pack("<H", 5)
    payload = bytes((number + 73 * i) % 256 for i in range(payload_length))
    return with_fcs(header + payload)


def confirmation_mpdu():
    return with_fcs(struct.pack("<HH", 0x00D4, 0) + address(16))


def beacon_mpdu():
    header = struct.pack("<HH", 0x0080, 0) + b"\xff" * 6 + address(23) * 2 + struct.pack("<H", 0x1230)
    body = struct.pack("<QHH", 12345, 100, 1) + b"\x00\x05Vorot"
    return with_fcs(header + body)


def bits_to_string(bits):
    return "".join(map(str, bits))


def write_coding_vectors(directory, reference):
    message = np.random.default_rng(517).integers(0, 2, 43).tolist() + [0] * 6
    # Scrambled PAD can leave the encoder in a nonzero state after TAIL.
    pad = [1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0]
    lines = []
    for label, rate in enumerate((reference.CR.CR12, reference.CR.CR23, reference.CR.CR34, reference.CR.CR56)):
        coded = reference.procBcc(message + pad, rate)
        lines.append(f"{label} {bits_to_string(message)} {bits_to_string(coded)}\n")
    (directory / "bcc.txt").write_bytes("".join(lines).encode())
    psdu = bytes.fromhex("80d32100015ef603fd90")
    data_bits = [byte >> b & 1 for byte in psdu for b in range(8)]
    lines = []
    for seed in (1, 2, 15, 63, 93, 127):
        bits = reference.procScramble([0] * 16 + data_bits, seed)
        lines.append(f"{seed} {psdu.hex()} {bits_to_string(bits)}\n")
    (directory / "scrambler.txt").write_bytes("".join(lines).encode())


def write_mac_vectors(directory):
    def control(subtype, after_ra=b""):
        return with_fcs(struct.pack("<HH", (subtype << 4) | 4, 0) + address(1) + after_ra)

    ordered = bytearray(data_mpdu(10, 3, ds=3, qos=True)[:-4])
    ordered[1] |= 0x80
    ordered[32:32] = struct.pack("<I", 0x78563412)
    management = bytearray(beacon_mpdu()[:-4])
    management[1] |= 0x80
    management[24:24] = struct.pack("<I", 0x79453612)
    amsdu = bytearray(data_mpdu(20, 0, ds=1, qos=True)[:-4])
    amsdu[24] |= 0x80

    frames = {
        "management_htc": with_fcs(management),
        "probe_request": with_fcs(struct.pack("<HH", 0x0040, 0) +
                                  b"\xff" * 6 + address(5) + b"\xff" * 6 + b"\x00" * 2),
        "ordered_qos": with_fcs(ordered),
        "short_qos": with_fcs(ordered[:31]),
        "amsdu": with_fcs(amsdu),
        "rts": control(11, address(2)),
        "cts": control(12),
        "ack": control(13),
        "ps_poll": control(10, address(2)),
        "cf_end": control(14, address(2)),
        "cf_end_ack": control(15, address(2)),
        "bar": control(8, address(2) + struct.pack("<HH", 0x4004, 0x3710)),
        "ba": control(9, address(2) + struct.pack("<HH", 0x4004, 0x3710) + b"\xff" * 8),
        "ba_basic": control(9, address(2) + struct.pack("<HH", 0x4000, 0x3710) + b"\xff" * 128),
        "multi_tid_bar": control(8, address(2) + struct.pack("<H", 0x1006) + b"\x00" * 8),
        "multi_tid_ba": control(9, address(2) + struct.pack("<H", 0x1006) + b"\x00" * 24),
        "short_ba": control(9, address(2) + struct.pack("<HH", 0x4004, 0x3710)),
        "short_rts": control(11),
        "unsupported": control(6),
        "wrapper": control(7, struct.pack("<HI", 0x00B4, 0x12345678) + address(2)),
        "short_wrapper": control(7, struct.pack("<H", 0x00B4)),
    }
    (directory / "mac.txt").write_bytes(
        "".join(f"{name} {mpdu.hex()}\n" for name, mpdu in frames.items()).encode()
    )


def generate(directory, tools):
    sys.path.insert(0, str(tools.resolve()))
    import mac80211
    import phy80211
    import phy80211header as reference

    directory.mkdir(parents=True, exist_ok=True)
    write_coding_vectors(directory, reference)
    write_mac_vectors(directory)
    iq_path = directory / "wifi20.sc16"
    manifest = directory / "packets.txt"
    iq_offset = 0
    real_bcc = reference.procBcc
    with iq_path.open("wb") as iq, manifest.open("w", encoding="ascii", newline="\n") as rows:
        def packet(name, mcs, mpdus, *, ht=False, short=False, aggregate=False, fault="-"):
            nonlocal iq_offset
            psdu = mac80211.genAmpduHT(mpdus) if aggregate else mpdus[0]
            mode = reference.modulation(
                phyFormat=reference.F.HT if ht else reference.F.L,
                mcs=mcs,
                bw=reference.BW.BW20,
                nSTS=2 if mcs == 8 else 1,
                shortGi=short,
            )
            transmitter = phy80211.phy80211(ifDebug=False)
            transmitter.dataScrambler = 1 + (97 * (mcs + 1) + int(short)) % 127

            def bcc_with_signal_fault(bits, rate):
                modified = bits[:]
                if ht and len(bits) == 48:
                    if fault == "crc":
                        modified[34] ^= 1
                    signal_bits = {
                        "reserved": (26, 0), "bandwidth": (7, 1), "ldpc": (30, 1),
                        "stbc": (28, 1), "ess": (32, 1), "illegal_mcs": (6, 1),
                    }
                    if fault in signal_bits:
                        position, bit = signal_bits[fault]
                        modified[position] = bit
                        if fault == "illegal_mcs":
                            modified[5] = 1  # MCS 96 is reserved.
                        modified[34:42] = reference.genBitBitCrc8(modified[:34])
                if len(bits) == 24 and fault in ("duration", "lsig"):
                    length = sum(bits[5 + i] << i for i in range(12)) + 3
                    modified[5:17] = [length >> i & 1 for i in range(12)]
                    modified[17] = sum(modified[:17]) % 2
                    if fault == "lsig":
                        modified[17] ^= 1
                return real_bcc(modified, rate)

            with patch.object(reference, "procBcc", bcc_with_signal_fault):
                if aggregate:
                    transmitter.genFromAmpdu(psdu, mode)
                else:
                    transmitter.genFromMpdu(psdu, mode)
            waveform = np.asarray(transmitter.ssPhySig[0])
            if fault == "qbpsk":
                waveform[480:560] = waveform[320:400]  # A real-axis symbol in place of HT-SIG2.
            if fault == "htstf":
                waveform[560:640] = 0
            if short:
                # Upstream sets the short-GI signaling and duration but always
                # emits a 16-sample CP. Keep the useful FFT and the last 8 CP
                # samples of each HT DATA symbol. Training still has long GI.
                data = waveform[720:].reshape((-1, 80))
                waveform = np.concatenate((waveform[:720], data[:, 8:].ravel()))

            quantized = np.column_stack((waveform.real, waveform.imag)) * 20000
            assert np.max(np.abs(quantized)) < 32767
            iq.write(np.rint(quantized).astype("<i2").tobytes())
            rate = mcs if ht else (6, 9, 12, 18, 24, 36, 48, 54)[mcs]
            mpdu_hex = ",".join(mpdu.hex() for mpdu in mpdus) if aggregate else "-"
            fields = (name, "HT" if ht else "L", rate, int(short), iq_offset,
                      waveform.size, fault, psdu.hex(), mpdu_hex)
            rows.write(" ".join(map(str, fields)) + "\n")
            iq_offset += waveform.size

        # 399 bytes at 6 Mb/s exercises both legacy/HT discrimination and
        # pilot polarity wraparound after 127 DATA symbols.
        packet("legacy6", 0, [data_mpdu(1, 371)])
        packet("legacy9", 1, [confirmation_mpdu()])
        packet("legacy12", 2, [beacon_mpdu()])
        for mcs in range(3, 8):
            packet(f"legacy{(6, 9, 12, 18, 24, 36, 48, 54)[mcs]}",
                   mcs, [data_mpdu(4 * mcs, 39 + mcs, ds=mcs % 4, qos=True)])

        for short in (False, True):
            for mcs in range(8):
                packet(f"ht{mcs}_{'short' if short else 'long'}", mcs,
                       [data_mpdu(40 + 4 * mcs, 77 + 5 * mcs, ds=mcs % 4, qos=True)], ht=True, short=short)

        first = data_mpdu(100, 19, qos=True)
        second = data_mpdu(104, 26, ds=1, qos=True)
        third = data_mpdu(108, 17, ds=2, qos=True)
        packet("ampdu", 7, [first, second, third], ht=True, short=True, aggregate=True)
        corrupt = bytearray(second)
        corrupt[21] ^= 2
        packet("ampdu_bad_fcs", 5, [first, bytes(corrupt), third], ht=True, aggregate=True, fault="fcs")
        packet("bad_fcs", 7, [bytes(corrupt)], fault="fcs")
        packet("mimo", 8, [first], ht=True, fault="mimo")
        for fault in ("lsig", "crc", "reserved", "illegal_mcs", "bandwidth", "stbc",
                      "ldpc", "ess", "qbpsk", "duration", "htstf"):
            packet(f"ht_{fault}", 2, [first], ht=True, fault=fault)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference_tools", type=Path, help="tools/ from the pinned gr-ieee80211 checkout")
    parser.add_argument("--output", type=Path, default=Path(__file__).parent / "vectors")
    args = parser.parse_args()
    generate(args.output, args.reference_tools)
