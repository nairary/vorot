#pragma once
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

#include <complex>
#include <cstdint>
#include <string>
#include <vector>
#include <iostream>

#include "IqQueue.h"
#include "IqAccumulator.h"
#include "ReaTimeFFT.h"
#include "RealTimeViewer.h"

class EspSdrReceiver
{
public:
	enum class Format
	{
		IQ8,
		IQ10
	};

	struct Capture {
		std::vector<std::complex<float>> iq;
		std::vector<std::complex<int16_t>> iq_int;
		uint32_t captureTimesUs = 0;
		double sampleRateHz = 0.0;
		double centerFrequencyHz = 0.0;
	};

		explicit EspSdrReceiver(MonitorParameters& parameters, size_t queue_capacity = 32);
		~EspSdrReceiver();
		EspSdrReceiver(const EspSdrReceiver&) = delete;
		EspSdrReceiver& operator=(const EspSdrReceiver&) = delete;

		bool connect(const std::string& port);
		void disconnect();

		bool isConnected() const;

		std::string getInfo();
		std::string getLimits();

		bool setFrequency(uint32_t mhz);
		bool setBandwidth(uint32_t mhz);

		bool setAutomaticGain();
		bool setManualGain(uint32_t index);

		bool capture(
			uint32_t sampleCount,
			Format format,
			Capture& output
		);

		void startStream() { if (!is_streaming_.load()) is_streaming_.store(true); }
		void stopStream() { if (is_streaming_.load()) is_streaming_.store(false); }
		bool isStreaming() const { return is_streaming_.load(); }

		void setSampleRate(uint32_t code) { current_rate_code_ = code; }
		uint32_t getSampleRate() const { return current_rate_code_; }

		void StreamIQData(
			std::vector<float>& output_data,
			std::atomic_bool& is_data_ready,
			std::mutex& data_mutex
		);

		void StreamImages(
			cv::Mat& output_image,
			std::atomic_bool& is_image_ready,
			std::mutex& data_mutex
		);

		const std::string& lastError() const;

		// Доступные коды
		// 1 - 80 MS/s, 2 - 40 MS/s, 3 - 20 MS/s, 4 - 10 MS/s
		// 5 - 8MS/s, 6 - 4 MS/s, 7 - 16 MS/s, 8 - 32 MS/s, 
		// 9 - 80 / 7.5 MS/s, 10 - 6.4 MS/s
		static double sampleRate(uint32_t code);

		std::unique_ptr<RealTimeViewer> image_processor_;
private:

	HANDLE port_ = INVALID_HANDLE_VALUE;

	std::string lastError_;
	uint32_t frequency_MHz_ = 2442;

	bool writeAll(const void* data, size_t size);
	bool readExact(void* data, size_t size);
	bool readLine(std::string& line);

	bool sendLine(const std::string& command);
	bool commandOK(const std::string& command);

	// Позволяет восстановиться после предыдущего незавершённого ответа или ответа о перезагрузке
	bool synchronoize();

	static uint32_t crc32(const uint8_t* data, size_t size);
	static int16_t signExtend10(uint16_t value);

	bool fail(const std::string& message);

	std::atomic_bool is_streaming_ = false;

	IqQueue queue_;
	uint32_t current_rate_code_ = 2;
	std::unique_ptr<RealTimeFFT> fft_processor_;
	
};