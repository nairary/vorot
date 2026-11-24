#pragma once
#include <mutex>
#include <vector>
#include "Core.h"



struct MonitorParameters
{
	double string_frequency = 0.0;
	double fcadr = 0.0;
	int horTotal = 0;
	int verTotal = 0;
};

class RealTimeViewer
{
public:
	RealTimeViewer(MonitorParameters& parameters) : m_monitor_parameters(parameters) {}
	void change_monitor_parameters(MonitorParameters& new_parameters);
	void processBlock(const std::complex<int16_t>* inputBlock, size_t count, const double fd);
	// Возвращает текстуру
	cv::Mat fast_image_restoration(std::vector<double>& signal, const double fd);
	cv::Mat get_last_image() const { return last_image; }
	bool is_image_ready() const { return m_image_is_ready.load(); }
	void set_creation_condition() { m_image_is_ready.store(false); }
	void reset_first_frame() { m_first_frame.store(true); }
	double current_f_cadr() const { return m_monitor_parameters.string_frequency / m_monitor_parameters.verTotal; }
	GLuint get_texture_id() const { return m_texture_id; }
private:
	// Создаёт текстуру по данным из сигнала записи
	GLuint create_texture(const char* data, const int width, const int height);
	void synchronyze_images(std::vector<double>& interp_frame, int horTotal, int verTotal);
	MonitorParameters m_monitor_parameters = MonitorParameters{};
	GLuint m_texture_id = GL_INVALID_VALUE;
	std::atomic_bool m_image_is_ready = false;
	std::atomic_bool m_first_frame = true;
	std::vector<float> m_etalon_cadr{};
	std::vector<float> m_etalon_string{};
	cv::Mat last_image;
	
};

