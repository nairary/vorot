#include "EspSdrReceiver.h"
#include <sstream>

EspSdrReceiver::EspSdrReceiver(MonitorParameters& parameters, size_t queue_capacity) 
	: queue_{ IqQueue(queue_capacity) }
{
	fft_processor_ = std::make_unique<RealTimeFFT>(32768);
	image_processor_ = std::make_unique<RealTimeViewer>(parameters);
}

EspSdrReceiver::~EspSdrReceiver()
{
	disconnect();
	fft_processor_ = nullptr;
	image_processor_ = nullptr;
	queue_.close();
}

bool EspSdrReceiver::connect(const std::string& port)
{
	disconnect();
	lastError_.clear();

	std::string path = port;

	if (path.rfind("\\\\.\\", 0) != 0) {
		path = "\\\\.\\" + port;
	}

	port_ = CreateFileA(
		path.c_str(),
		GENERIC_READ | GENERIC_WRITE,
		0,
		nullptr,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		nullptr
	);

	if (!isConnected())
	{
		return fail(
			"Cannot open " + port +
			" (Win32 error " +
			std::to_string(GetLastError()) + ")"
		);
	}

	auto closeOnError = [this]()
		{
			disconnect();
			return false;
		};

	if (!SetupComm(port_, 65536, 65536))
	{
		fail("SetupComm failed");
		return closeOnError();
	}

	// специальная структура данных, которая определяет параметры управления для последовательного порта
	DCB dcb{};
	dcb.DCBlength = sizeof(dcb);

	if (!GetCommState(port_, &dcb)) {
		fail("GetCommState failed");
		return closeOnError();
	}

	dcb.BaudRate = 2000000;
	dcb.ByteSize = 8;
	dcb.Parity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;

	dcb.fBinary = TRUE;
	dcb.fParity = FALSE;
	dcb.fOutxCtsFlow = FALSE;
	dcb.fDsrSensitivity = FALSE;
	dcb.fOutX = FALSE;
	dcb.fInX = FALSE;

	dcb.fDtrControl = DTR_CONTROL_ENABLE;
	dcb.fRtsControl = RTS_CONTROL_DISABLE;

	if (!SetCommState(port_, &dcb))
	{
		fail("SetCommState failed");
		return closeOnError();
	}

	COMMTIMEOUTS timeouts{};

	timeouts.ReadIntervalTimeout = 50;
	timeouts.ReadTotalTimeoutConstant = 1000;
	timeouts.ReadTotalTimeoutMultiplier = 0;

	timeouts.WriteTotalTimeoutConstant = 1000;
	timeouts.WriteTotalTimeoutMultiplier = 0;

	if (!SetCommTimeouts(port_, &timeouts))
	{
		fail("SetCommTimeouts failed");
		return closeOnError();
	}

	if (!PurgeComm(port_, PURGE_RXCLEAR | PURGE_TXCLEAR))
	{
		fail("PurgeComm failed");
		return closeOnError();
	}

	if (!synchronoize())
		return closeOnError();

	return true;
}

void EspSdrReceiver::disconnect()
{
	if (isConnected())
	{
		CloseHandle(port_);
		port_ = INVALID_HANDLE_VALUE;
	}
}

bool EspSdrReceiver::isConnected() const
{
	return port_ != INVALID_HANDLE_VALUE;
}

std::string EspSdrReceiver::getInfo()
{
	if (!sendLine("INFO"))
		return {};

	std::string response;
	if (!readLine(response))
		return {};

	if (response.rfind("ERR", 0) == 0) 
	{
		fail(response);
		return {};
	}

	return response;
}

std::string EspSdrReceiver::getLimits()
{
	lastError_.clear();

	if (!sendLine("LIMITS?"))
		return{};

	std::string response;
	if (!readLine(response))
		return {};

	if (response.rfind("ERR", 0) == 0)
	{
		fail(response);
		return {};
	}

	return response;
}

bool EspSdrReceiver::setFrequency(uint32_t mhz)
{
	if (mhz < 100 || mhz > 6000)
		return fail("Invalid frequency");

	if (!commandOK("FREQ " + std::to_string(mhz)))
		return false;

	frequency_MHz_ = mhz;
	return true;
}

bool EspSdrReceiver::setBandwidth(uint32_t mhz)
{
	return commandOK("BANDWIDTH " + std::to_string(mhz));
}

bool EspSdrReceiver::setAutomaticGain()
{
	return commandOK("GAIN HARDWARE");
}

bool EspSdrReceiver::setManualGain(uint32_t index)
{
	return commandOK("GAIN MANUAL " + std::to_string(index));
}

bool EspSdrReceiver::capture(uint32_t sampleCount, Format format, Capture& output)
{
	lastError_.clear();

	if (!isConnected())
		return fail("Port is not connected");

	if (sampleCount < 256 || sampleCount > 16380)
		return fail("Invalid sample count");

	if (current_rate_code_ > 5)
		return fail("Unsupported C5 rate code");

	const bool iq8 = (format == Format::IQ8);

	const std::string command = std::string(iq8 ? "CAP16 " : "CAP20 ") +
		std::to_string(sampleCount) + " " +
		std::to_string(current_rate_code_);

	if (!sendLine(command))
		return false;

	std::string header;

	if (!readLine(header))
		return false;

	std::istringstream stream(header);
	std::string tag;
	std::string crcHex;
	uint32_t returnedCount = 0;
	uint32_t captureTimeUs = 0;
	std::string extra;

	if (!(stream >> tag
		>> returnedCount
		>> crcHex
		>> captureTimeUs
		) ||
		(stream >> extra) ||
		tag != "DATA" ||
		returnedCount != sampleCount)
		return fail("Invalid DATA header: " + header);

	uint32_t expectedCRC = 0;
	try
	{
		size_t parsed = 0;
		unsigned long value = std::stoul(crcHex, &parsed, 16);
		if (parsed != crcHex.size() || value > 0xFFFFFFFFul)
			return fail("Invalid CRC field");
		expectedCRC = static_cast<uint32_t>(value);
	}
	catch (...)
	{
		return fail("Invalid CRC field");
	}

	const size_t payloadSize = iq8 ? static_cast<size_t>(sampleCount * 2)
		: (static_cast<size_t>(sampleCount) * 20 + 7) / 8;

	std::vector<uint8_t> payload(payloadSize);
	if (!readExact(payload.data(), payload.size()))
		return false;

	uint32_t actualCRC = crc32(payload.data(), payload.size());

	if (actualCRC != expectedCRC)
		return fail("IQ payload CRC32 mismatch");

	Capture result;
	result.iq.resize(sampleCount);
	result.iq_int.resize(sampleCount);
	result.captureTimesUs = captureTimeUs;
	result.sampleRateHz = sampleRate(current_rate_code_);
	result.centerFrequencyHz = static_cast<double>(frequency_MHz_) * 1e6;

	if (iq8)
	{
		for (uint32_t i = 0; i < sampleCount; ++i)
		{
			const int8_t rawI = static_cast<int8_t>(payload[2 * i]);
			const int8_t rawNegQ = static_cast<int8_t>(payload[2 * i+1]);
			
			const int16_t I = static_cast<int16_t>(rawI);
			const int16_t Q = static_cast<int16_t>(-rawNegQ);

			result.iq_int[i] = { I,Q };
		}
	}
	else
	{
		for (uint32_t i = 0; i < sampleCount; ++i)
		{
			const size_t bitPos = static_cast<size_t>(i) * 20;
			const size_t bytePos = bitPos / 8;
			const unsigned shift = static_cast<unsigned>(bitPos % 8);

			uint32_t packed = 0;

			for (size_t j = 0; j < 4; ++j)
			{
				if (bytePos + j < payload.size())
				{
					packed |= uint32_t(payload[bytePos + j]) << (8 * j);
				}
			}
			packed = (packed >> shift) & 0xFFFFFu;
			int16_t rawI = signExtend10(static_cast<uint16_t>(packed & 0x03FFu));
			int16_t rawNegQ = signExtend10(static_cast<uint16_t>((packed >> 10) & 0x03FFu));
			
			const int16_t I = rawI;
			const int16_t Q = -rawNegQ;

			result.iq_int[i] = { I, Q };
		}
	}
	output = std::move(result);
	return true;
}

void EspSdrReceiver::StreamIQData(
	std::vector<float>& output_data,
	std::atomic_bool& is_data_ready,
	std::mutex& data_mutex)
{
	
	IqQueue processing(64);

	Capture data;
	uint64_t first = 0;
	uint64_t segment = 0;
	bool gap = false;
	uint64_t generation = 0;

	const size_t fft_size = fft_processor_->getFFTSize();
	
	std::jthread processor([&] {
		try
		{
			IqAccumulator fft_pending(fft_size);
			uint64_t segment = 0;
			uint64_t next = 0;
			IqBlock block;
			while (processing.pop(block) && is_streaming_.load())
			{
				if (block.discontinuity || segment != block.segment || next != block.first)
				{
					fft_pending.reset();
				}
				
				segment = block.segment;
				next = block.first + block.samples.size();
				fft_pending.process(block.samples, [&](auto fft_block)
					{
						fft_processor_->processBlock(fft_block.data(), fft_block.size());
						std::lock_guard lock(data_mutex);
						output_data = fft_processor_->getLatestMagnitudes();
						is_data_ready.store(true);
					});
			}
		}
		catch (const std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
		});


	while (is_streaming_.load())
	{
		if (!capture(16380, Format::IQ8, data))
		{
			std::cout << "Error while capturing data\n";
			is_streaming_.store(false);
			return;
		}

		const std::span<const std::complex<int16_t>> samples(data.iq_int.data(), data.iq_int.size());
		processing.push(samples, first, segment, gap, generation);
		first += data.iq_int.size();
	}
	processing.close();
	processor.join();
}

void EspSdrReceiver::StreamImages(cv::Mat& output_image, std::atomic_bool& is_image_ready, std::mutex& data_mutex)
{
	IqQueue processing(64);

	Capture data;
	uint64_t first = 0;
	uint64_t segment = 0;
	bool gap = false;
	uint64_t generation = 0;
	double fd = sampleRate(current_rate_code_);
	const size_t image_size = static_cast<size_t>(fd / image_processor_->current_f_cadr() + 10.0);
	// Это лямбда для восстановления изображения
	std::jthread processor([&] {
		try
		{
			IqAccumulator image_pending(image_size);
			uint64_t segment = 0;
			uint64_t next = 0;
			IqBlock block;
			int i = 0;
			while (processing.pop(block) && is_streaming_.load())
			{
				if (block.discontinuity || segment != block.segment || next != block.first)
				{
					image_pending.reset();
				}

				segment = block.segment;
				next = block.first + block.samples.size();
				image_pending.process(block.samples, [&](auto image_block)
					{
						image_processor_->processBlock(image_block.data(), image_block.size(), fd);
						std::lock_guard lock(data_mutex);
						output_image = image_processor_->get_last_image();
						/*std::string filename = "Image # " + std::to_string(i) + ".png";
						cv::imwrite(filename, output_image);*/
						is_image_ready.store(true);
					});
				i++;
			}
		}
		catch (const std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
		});

	// Это заполнение из потока
	while (is_streaming_.load())
	{
		if (!capture(16380, Format::IQ8, data))
		{
			std::cout << "Error while capturing data\n";
			is_streaming_.store(false);
			return;
		}

		const std::span<const std::complex<int16_t>> samples(data.iq_int.data(), data.iq_int.size());
		processing.push(samples, first, segment, gap, generation);
		first += data.iq_int.size();
	}
	processing.close();
	processor.join();
}

bool EspSdrReceiver::writeAll(const void* data, size_t size)
{
	if (!isConnected())
	{
		return fail("Port is not connected");
	}

	const auto* bytes = static_cast<const uint8_t*>(data);

	size_t written = 0;

	while (written < size)
	{
		DWORD count = 0;
		DWORD chunk = static_cast<DWORD>(std::min<size_t>(size - written, 65536));

		if (!WriteFile(port_, bytes + written, chunk, &count, nullptr))
		{
			return fail("Serial write failed");
		}

		if (count == 0)
			return fail("Serial write timeout");

		written += count;
	}
	return true;
}

bool EspSdrReceiver::readExact(void* data, size_t size)
{
	if (!isConnected())
		return fail("Port is not connected");

	auto* bytes = static_cast<uint8_t*>(data);
	size_t received = 0;

	while (received < size)
	{
		DWORD count = 0;
		DWORD chunk = static_cast<DWORD>(std::min<size_t>(size - received, 65636));

		if (!ReadFile(port_, bytes + received, chunk, &count, nullptr))
			return fail("Serial read failed");

		if (count == 0)
			return fail("Timeout while reading binary data");

		received += count;
	}

	return true;
}

bool EspSdrReceiver::readLine(std::string& line)
{
	line.clear();
	constexpr size_t MAX_LINE = 512;
	while (line.size() < MAX_LINE)
	{
		char c = 0;
		if (!readExact(&c, 1))
			return false;

		if (c == '\n')
		{
			if (!line.empty() && line.back() == '\r')
				line.pop_back();
			return true;
		}

		line.push_back(c);
	}
	return fail("Response line is too long");
}

bool EspSdrReceiver::sendLine(const std::string& command)
{
	std::string line = command + "\n";
	return writeAll(line.data(), line.size());
}

bool EspSdrReceiver::commandOK(const std::string& command)
{
	lastError_.clear();
	if (!sendLine(command))
		return false;

	std::string response;
	if (!readLine(response))
		return false;

	if (response != "OK")
		return fail(command + ": " + response);

	return true;
}

bool EspSdrReceiver::synchronoize()
{
	for (int attempt = 0; attempt < 5; ++attempt)
	{
		PurgeComm(port_, PURGE_RXCLEAR);

		const std::string nonce = std::to_string(10000000ULL + 
			static_cast<unsigned long long>(GetTickCount64()) + attempt);

		const std::string expected = "SYNC " + nonce;

		if (!sendLine(expected))
			return false;

		for (int i = 0; i < 12; ++i)
		{
			std::string line;
			if (!readLine(line))
				break;

			if (line == expected)
			{
				lastError_.clear();
				return true;
			}
		}
	}
	return fail("ESP-SDR synchronization failed");
}

uint32_t EspSdrReceiver::crc32(const uint8_t* data, size_t size)
{
	uint32_t crc = 0xFFFFFFFFu;

	for (size_t i = 0; i < size; ++i) 
	{
		crc ^= data[i];

		for (int bit = 0; bit < 8; ++bit) 
		{
			if (crc & 1u) 
			{
				crc = (crc >> 1) ^ 0xEDB88320u;
			}
			else 
			{ 
				crc >>= 1;
			}
		}
	}

	return crc ^ 0xFFFFFFFFu;
}

int16_t EspSdrReceiver::signExtend10(uint16_t value)
{
	value &= 0x03FFu;

	if (value & 0x0200u)
	{
		return static_cast<int16_t>(static_cast<int>(value) - 1024);
	}
	return static_cast<int16_t>(value);
}

bool EspSdrReceiver::fail(const std::string& message)
{
	lastError_ = message;
	return false;
}

const std::string& EspSdrReceiver::lastError() const
{
	return lastError_;
}

double EspSdrReceiver::sampleRate(uint32_t code)
{
	static constexpr double rates[] = {
	   80e6, 40e6, 20e6, 10e6,
	   8e6, 4e6, 16e6, 32e6,
	   80e6 / 7.5, 6.4e6
	};

	if (code >= std::size(rates))
		return 0.0;

	return rates[code];
}
