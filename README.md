# Vorot

## Receive Wi-Fi frames

Choose **Turn on Wi-Fi detection** before starting the USRP stream, tune to the center of the Wi-Fi channel, and set the actual RX rate to **20 MS/s**. Select **Enable Wi-Fi detection**, then start the stream. The Wi-Fi worker consumes the sc16 samples from `GetWiFiData()`. Select the spectrum mode before starting a spectrum stream with `GetUSRPData()`.

The Wi-Fi table keeps the 64 most recent verified MPDUs. Each row shows the RX segment and inferred PPDU start sample, PHY, legacy rate or HT MCS, MAC addresses, frame type and subtype, MPDU length including FCS, and FCS status. The receiver and transmitter columns show RA and TA when present. SA and DA describe end-to-end addresses where the MAC header identifies them. The UI also reports the latest decode diagnostic. FCS validates the received frame bytes. The addresses come from the transmitted header.

| PHY | Supported configurations |
|---|---|
| Legacy OFDM | 20 MHz at 6, 9, 12, 18, 24, 36, 48 and 54 Mb/s |
| HT-mixed | HT20 MCS 0–7, one spatial stream, BCC, normal and short guard intervals, single MPDUs and HT A-MPDUs |

After a valid L-SIG, the decoder looks for the rotated HT-SIG constellation only when L-SIG can describe HT-mixed duration. It checks both HT-SIG symbols, CRC-8, reserved fields, and the advertised duration before decoding HT DATA. DSSS/CCK, HT greenfield, HT40, multiple spatial streams, STBC, LDPC, and newer PHY formats are outside the supported set. Bandwidth, coding, and stream limitations are reported when identified in HT-SIG. A gap resets in-flight reception.

MAC parsing covers management and data headers, four-address data, QoS and HT Control, ACK, CTS, RTS, PS-Poll, CF-End variants, basic/compressed/Multi-TID Block Ack and Block Ack Request, and Control Wrapper for supported carried frames. In an HT A-MPDU, each subframe has its own FCS check. An invalid delimiter ends extraction at that boundary. A bad FCS rejects that MPDU. The parser exposes the outer MAC header for A-MSDU and protected frames, with A-MSDU payload addresses and decryption outside its scope.

## Run offline tests

Build the receiver tests with CMake 3.20 or newer, a C++20 compiler, and the FFTW single-precision development files. The test target runs without UHD, SDR hardware, and the GUI.

```sh
cmake -S tests -B build/wifi-tests -DCMAKE_BUILD_TYPE=Release
cmake --build build/wifi-tests --config Release
ctest --test-dir build/wifi-tests -C Release --output-on-failure
```

If CMake needs explicit FFTW paths, pass `-DFFTW3_INCLUDE_DIR=<directory containing fftw3.h>` and `-DFFTW3F_LIBRARY=<FFTW single-precision library>` at configuration. On Windows with a DLL build of FFTW, add the directory containing `libfftw3f-3.dll` to `PATH` before running CTest.

The four suites exercise BCC and descrambling, MAC address roles and FCS, byte-for-byte DATA recovery, and the threaded receive worker. The worker tests add CFO, AWGN, multipath and timing offsets to the reference IQ, deliver it across uneven blocks, and verify gap handling and the bounded frame history. Invalid L-SIG and HT-SIG, unsupported HT configurations, and corrupt FCS have dedicated vectors.

### Reference vectors

The checked-in vectors are deterministic, offline sc16 samples from an independent IEEE 802.11 waveform generator. `tests/vectors/packets.txt` records each packet's sample offset and length in `wifi20.sc16`, its declared format and rate, any deliberate fault, and the expected PSDU and individual MPDU bytes. FCS values for the synthetic MPDUs come from Python's `zlib.crc32`. The generator adjusts the upstream DATA cyclic prefixes for short-GI fixtures while preserving the generated signaling and useful symbols. [Waveform generator at the pinned revision](https://github.com/cloud9477/gr-ieee80211/tree/add75653e8de63361b9d3d34606eb8dc5b6029c5/tools)

Regeneration requires Python, NumPy and Matplotlib. Run from the repository root with `tests/generate_reference.py <path-to-pinned-gr-ieee80211/tools>` using Python. Normal test runs use the checked-in vectors.
