#include "RealTimeViewer.h"

void RealTimeViewer::change_monitor_parameters(MonitorParameters& new_parameters)
{
	m_monitor_parameters = new_parameters;
	std::vector<float>{}.swap(m_etalon_cadr);
	std::vector<float>{}.swap(m_etalon_string);
	m_etalon_cadr.resize(m_monitor_parameters.verTotal);
	m_etalon_string.resize(m_monitor_parameters.horTotal);
}

void RealTimeViewer::processBlock(const std::complex<int16_t>* inputBlock, size_t count, const double fd)
{

	int samples_per_cadr = static_cast<int>(std::ceil(fd / m_monitor_parameters.fcadr));
	if (count < samples_per_cadr || inputBlock == nullptr) return;
	std::vector<double> signal(samples_per_cadr);
	for (int i = 0; i < samples_per_cadr; ++i)
	{
		auto real = inputBlock[i].real();
		auto imag = inputBlock[i].imag();
		signal[i] = sqrt(real * real + imag * imag);
	}
	last_image = fast_image_restoration(signal, fd);
}

cv::Mat RealTimeViewer::fast_image_restoration(std::vector<double>& signal, const double fd)
{
	
	const size_t length = signal.size();
	const int horTotal = m_monitor_parameters.horTotal;
	const int verTotal = m_monitor_parameters.verTotal;
	const size_t L = horTotal * verTotal;
	const double f_pix = L * m_monitor_parameters.fcadr;
	std::vector<char> image(L);
	if (m_monitor_parameters.fcadr < 20.0)
	{
		throw std::runtime_error("Signal size is less then needed!\n");
	}
		
	int samples_per_cadr = static_cast<int>(std::ceil(fd / m_monitor_parameters.fcadr));
	
	// Ненужное копирование. Нужно переделать через std::ranges::view
	std::vector<double> frame(samples_per_cadr);
	for (size_t i = 0; i < samples_per_cadr; i++)
	{
		frame[i] = signal[i];
	}
	auto interp_frame = interp_double(frame, fd, f_pix);
	VectorPadding(interp_frame, L);

	// Синхронизация. Возможно, стоит её сделать опциональной
	if (m_first_frame.load())
	{
		synchronyze_images(interp_frame, horTotal, verTotal);
	}
	else
	{
		RTSynchronize(interp_frame, m_etalon_string, m_etalon_cadr, horTotal, verTotal);
	}
	auto max_element = std::max_element(interp_frame.begin(), interp_frame.end());
	for (size_t j = 0; j < L; j++)
	{
		image[j] = static_cast<char>(interp_frame[j] / (*max_element) * 255);
	}
	
	cv::Mat1b dst_every(verTotal, horTotal, CV_8UC1);
	cv::Mat rgb_image(verTotal, horTotal, 3);
	imadjust(image, dst_every, horTotal, verTotal, 1);
	cv::cvtColor(dst_every, rgb_image, cv::COLOR_GRAY2RGB);
	m_texture_id = 42;
	//cv::imwrite(filename, rgb_image);
	//m_texture_id = create_texture((const char*)rgb_image.ptr(), horTotal, verTotal);
	//std::cout << "Texture ID from func " << m_texture_id << std::endl;
	m_image_is_ready.store(true);
	return rgb_image;
}

GLuint RealTimeViewer::create_texture(const char* data, const int width, const int height)
{
	GLuint textureID = 42;
	
	if (data)
	{
		glGenTextures(1, &textureID);
		if (m_texture_id != GL_INVALID_VALUE)
			textureID = m_texture_id;

		GLenum format{};
		format = GL_RGB;
		glBindTexture(GL_TEXTURE_2D, textureID);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
		// Установка параметров фильтрации текстуры
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		//glGenerateMipmap(GL_TEXTURE_2D);
	}
	else
	{
		std::string message = "Error: Incorrect data during frame restoration";
		return 0;
	}
	return textureID;
}

void RealTimeViewer::synchronyze_images(std::vector<double>& interp_frame, int horTotal, int verTotal)
{
	std::vector<float>{}.swap(m_etalon_cadr);
	std::vector<float>{}.swap(m_etalon_string);
	m_etalon_cadr.resize(verTotal);
	m_etalon_string.resize(horTotal);
	for (size_t j = 0; j < horTotal; j++)
	{
		for (size_t i = 0; i < verTotal; i++)
		{
			m_etalon_cadr[i] += (float)interp_frame[i * horTotal + j];
			m_etalon_string[j] += (float)interp_frame[i * horTotal + j];
		}
	}
	auto diff_hor = diffSig(m_etalon_cadr);
	auto diff_ver = diffSig(m_etalon_string);

	auto ind_hor = std::distance(diff_hor.begin(), std::max_element(diff_hor.begin(), diff_hor.end()));
	auto ind_ver = std::distance(diff_ver.begin(), std::max_element(diff_ver.begin(), diff_ver.end()));

	m_etalon_cadr = circshift(m_etalon_cadr, m_etalon_cadr.size(), 1, -(ind_hor), 0);
	m_etalon_string = circshift(m_etalon_string, m_etalon_string.size(), 1, -(ind_ver), 0);


	m_first_frame.store(false);
}
