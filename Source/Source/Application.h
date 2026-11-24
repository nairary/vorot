#pragma once
#include "Core.h"
#include "imgui/implot.h"
#include <stdexcept>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <deque>

#include "USRPDevice.h"
#include "PlotFunctions.h"
#include "EspSdrReceiver.h"

class Application
{
public:
	Application() = delete;
	Application(const int width = 1920, const int height = 1080);
	void ShowPlot();
	void ShowUSRPInterface();
	void ShowESPReceiverInterface();
	void EspStopStream();
	void EspStartStream();
	void ESPShowRealTimeImage();
	void CreateUSRPDevice();
	void GetAndRenderStreamingData();
	void GetAndRenderESPData();
	void ResetSpectrumAveraging();
	void UpdateSpectrumAveraging(const std::vector<float>& spectrum, double sampling_rate_hz);
	const std::vector<float>& GetSpectrumForPlot() const;
	GLuint CreateTextureFromCVMat(cv::Mat& texture, const int hor, const int ver);
	void Run();
	~Application();
private:
	std::unique_ptr<USRPDevice> usrp_device = nullptr;
	std::atomic_bool usrp_device_exists = false;
	std::atomic_bool is_data_ready = false;
	std::atomic_bool is_image_ready = false;
	std::atomic_bool is_texture_ready = false;
	std::mutex receiver_mutex;
	std::mutex texture_mutex;
	std::vector<std::jthread> app_threads;
	std::vector<float> local_data;
	std::vector<float> temp_data;
	cv::Mat tmp_image;
	cv::Mat local_image;
	std::vector<float> averaged_data;
	std::vector<float> average_sum;
	std::deque<std::vector<float>> average_history;
	std::atomic_uint64_t data_sequence = 0;
	uint64_t last_averaged_sequence = 0;
	bool average_spectrum_enabled = false;
	bool enable_marker = false;
	bool enable_low_frequency_spectrum = false;
	bool wifi_receive_mode = false;
	float average_time_seconds = 1.0f;
	float recording_time_seconds = 1.0f;
	double spectrum_display_width_khz = -1.0;
	int selected_fft_exponent = DEFAULT_FFT_EXPONENT;
	GLFWwindow* window = nullptr;
	ImGuiIO io;

	double current_frequency = 1000.0;
	double current_sampling_rate = 10.0;
	double current_rx_gain = 30.0;
	int selected_sampling_rate_index = 0;
	std::vector<double> sampling_rates_mhz;
	const double frequency_max = 6000;
	const double frequency_min = 10;
	const double rx_gain_max = 31.5;
	const double rx_gain_min = 0.0;

	std::vector<uint32_t> esp_sampling_rates;
	std::vector<std::string> esp_sampling_rates_MHz;
	int esp_selected_sample_rate_index = 0;
	uint32_t esp_current_bandwidth = 48;
	uint32_t esp_current_frequency = 2400;
	uint32_t esp_current_gain = 10;
	int esp_current_regime = 0;

	std::unique_ptr<EspSdrReceiver> esp_receiver = nullptr;

	MonitorParameters monitor_parameters = MonitorParameters{ 67500.0, 60.0, 2200, 1125 };
	GLuint real_time_texture_id = GL_INVALID_VALUE;
	GLuint current_texture = GL_INVALID_VALUE;
	
};


