#pragma once
#include <string>
#include <array>
#include <span>

#include <filesystem>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "Utils.h"
#include "Console.h"
#include "PhaseModul.h"
#include "FIRDecimator.h"

#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"
#include "imgui/implot.h"
#include "imgui/implot_demo.h"
#include "imgui/ImGuiFileDialog.h"

#include "alglib/fasttransforms.h"

namespace wave
{
	// Вспомогательный класс, который позволит получить промежуточные данну о сигнале
	enum class CustomReceiverDataType
	{
		COMPLEX_INT16,
		COMPLEX_FLOAT
	};

	struct MonitorParameters
	{
		double modulation_frequency = 0.0;
		double string_frequency = 0.0;
		double fcadr = 0.0;
		int horTotal = 0;
		int verTotal = 0;
	};

	struct InterfaceParameters
	{
		bool restore_as_HDMI = false;
		bool restore_as_printer = false;
		bool restore_as_LVDS = false;
		bool restore_as_HDMI20 = false;
		bool find_exact_frequency = false;
		int window_length = 256;
		int hop_size = 30;
		float f_cadr = 0.0f;
		float f_string = 0.0f;
		float f_mod = 0.0f;
		int in_verTotal = 1125;
		int in_horTotal = 2200;
		int pix_shift = 0;
		float freq_dev_ampl = 0.0f;
		std::filesystem::path current_folder = std::filesystem::path();
		bool invert_phase = false;
		double phase_shift = 0.0;
		bool sync_with_phase = false;
		bool hardcoded_fstring = false;
		bool use_phi_0 = false;
		bool delete_bad_frames = false;
		bool use_printer_variance = false;
		bool use_2_channels = false;
		double phi_0 = 0.0;
		int start_frame = 1;
		double printer_bandwidth = 0.0;
		int scr_length = 0;
		double f_d = 0.0;
		bool use_phase_hdmi20 = false;
	};

	struct PrinterParameters
	{
		double f_string = 0.0;
		std::filesystem::path current_folder = std::filesystem::path{};
		double bandwidth = 0.0;
		bool use_variance_data = false;
	};

	class AnalysisResults
	{
	public:
		MonitorParameters m_monitor_parameters = MonitorParameters{};
		std::vector<MonitorParameters> additional_parameters;
		AnalysisResults() = default;
		void CopyValues(
			const std::vector<std::vector<double>>& spe,
			FreqSaw& saw,
			Spectrum saw_spectrum,
			double modulation_frequency,
			double string_frequency
		);
		// Единицы измерения МГц
		int central_frequency = 0;
		float central_frequency_f = 0.0f;
		// Единицы измерения Гц
		float sampling_frequency = 0.0f; 
		std::vector<double> spec_projection;
		std::vector<float> hor_sum;
		std::vector <float> ver_sum;
		std::vector<double> hor_sum_vector;
		std::vector<double> hor_sum_freqs;
		CrossCorrelation<double> corr = CrossCorrelation<double>{};
		CrossCorrelation<double> corr1 = CrossCorrelation<double>{};
		CrossCorrelation<float> corr2 = CrossCorrelation<float>{};
		FreqSaw m_freq_saw = FreqSaw{};
		FreqSaw ridge_saw = FreqSaw{};
		Spectrum m_freq_saw_spectrum = Spectrum{};
		Spectrum sparse_spectrum = Spectrum{};
		Spectrum sparse_spectrum_ch2 = Spectrum{};
		Spectrum phase_spectrum = Spectrum{}; // Спектр, рассчитываемый по фазе и необходимый для восстановления по фазе
		Spectrum phase_spectrum_ch2 = Spectrum{}; // Спектр, рассчитываемый по фазе и необходимый для восстановления по фазе
		Spectrum iq_spectrum = Spectrum{};
		Spectrum iq_spectrum_ch2 = Spectrum{};
		std::vector<double> signal_fft = std::vector<double>{};
		std::vector<double> signal_fft_ch2 = std::vector<double>{};
		void ShowSpectrum();
		 
		void ShowSpectrogram();
		void ShowSaw();
		void ShowRidgeSaw();
		void ShowCorrelation();
		void ShowSawSpectrum();
		ImPlotPoint ShowIQSpectrum(ImPlotRect& axis_limits);
		ImPlotPoint ShowSparseSpectrum();
		ImPlotPoint ShowPhaseSpectrum(ImPlotRect& axis_limits);
		ImPlotPoint ShowHDIM2Correlation();
		void ShowHorSum();
		void ShowVerSum();
		void ShowStringsVector();
		void ShowShiftHors();
		void ShowPrinterVars();
		void ShowPrinterVarsCh2();
		bool IsEmpty() const { return m_is_emty; }
		void Reset();
		bool m_is_emty = true;
		FreqSaw m_generated_saw = FreqSaw{};
		std::vector<std::vector<double>> m_spectrogram = std::vector<std::vector<double>>{};
		// Дополнительные данные для изменения начальной фазы сигнала
		std::vector<std::complex<double>> result_iq_signal = std::vector<std::complex<double>>();
		double fd = 0.0;
		int horTotal = 0;
		int verTotal = 0;
		bool have_iq_image = false;
		bool have_iq_specturm = false;
		bool have_iq_spectrum_ch2 = false;
		std::vector<int> shift_hors = std::vector<int>{};
		std::vector<char> shift_hors_mask = std::vector<char>{};
		std::vector<int> diff_shift_hors = std::vector<int>{};
		std::vector<double> printer_variance = std::vector<double>{};
		std::vector<double> printer_variance_ch2 = std::vector<double>{};
		std::vector<double> hdmi2_correlation = std::vector<double>{};
		std::vector<int> printer_freqs = std::vector<int>{};
		// параметры линейной апроксимации сдвига кадров ax+b
		double shift_slope = 0.0;
		double shift_start = 0.0;
	private:

	};

	struct ScreenParamsHDMI
	{
		float f_cadr = 0.0f;
		float f_string = 0.0f;
		int ver_total = 0;
		int hor_total = 0;
		double f_d = 0.0;
		double f_pix = 0.0;
		double fshift = 0.0;
		std::filesystem::path current_folder = std::filesystem::path();
		bool invert_phase = false;
		double phase_shift = 0.0;
		bool sync_with_phase = false;
		bool hardcoded_fstring = false;
		bool use_phi_0 = false;
		bool delete_bad_frames = false;
		double phi_0 = 0.0;
		int start_frame = 1;
	};

	struct ScreenParamsHDMI20
	{
		float f_cadr = 0.0f;
		float f_string = 135000.0f;
		int ver_total = 2250;
		int hor_total = 4400;
		int scr_length = 0;
		double f_d = 0.0;
		double f_pix = 0.0;
		std::filesystem::path current_folder = std::filesystem::path();
		bool hardcoded_fstring = false;
		bool delete_bad_frames = false;
		int start_frame = 1;
		double f_shift = 0.0;
		double f_shift_ch2 = 0.0;
		double initial_phase_shift = 0.0;
		bool invert_phase = false;
	};

	struct ScreenParamsLVDS
	{
		float f_cadr = 0.0f;
		float f_string = 0.0f;
		float f_mod = 0.0f;
		float freq_dev_ampl = 0.0f;
		int ver_total = 0;
		int hor_total = 0;
		double f_d = 0.0;
		double f_pix = 0.0;
	};

	// Основная фукнция
	// filename - расположение файла записи
	// analysis - вспомогательные промежуточные результаты
	// stop_restoration, restoration_finished - атомики, управляющие процедурой восстановления
	double RestoreImages(std::string& filename, AnalysisResults& analysis, WaveConsole& console,
		std::atomic<bool>& stop_restoration,
		std::atomic<bool>& restoration_finished,
		std::mutex& fft_mutex,
		InterfaceParameters params = InterfaceParameters{});

	void RestorePrinter(
		std::string& filename,
		const DataHeader_New& header,
		AnalysisResults& analysis,
		WaveConsole& console,
		std::atomic<bool>& stop_restoration,
		std::atomic<bool>& restoration_finished,
		std::mutex& fft_mutex,
		PrinterParameters params = PrinterParameters{}
	);

	void RestorePrinter2Channels(
		std::string& filename,
		const DataHeader_New& header,
		AnalysisResults& analysis,
		WaveConsole& console,
		std::atomic<bool>& stop_restoration,
		std::atomic<bool>& restoration_finished,
		std::mutex& fft_mutex,
		PrinterParameters params = PrinterParameters{}
	);

	void RestoreHDMI(
		std::string& filename,
		const DataHeader_New& header,
		AnalysisResults& analysis,
		WaveConsole& console,
		std::atomic<bool>& stop_restoration,
		std::atomic<bool>& restoration_finished,
		std::mutex& fft_mutex,
		ScreenParamsHDMI screen_params = ScreenParamsHDMI{}
	);

	void RestoreHDMI20(std::string& filename,
		const DataHeader_New& header,
		AnalysisResults& analysis,
		WaveConsole& console,
		std::atomic<bool>& stop_restoration,
		std::atomic<bool>& restoration_finished,
		std::mutex& fft_mutex,
		ScreenParamsHDMI20 screen_params = ScreenParamsHDMI20{}
	);

	void RestoreHDMI20_phase(std::string& filename,
		const DataHeader_New& header,
		AnalysisResults& analysis,
		WaveConsole& console,
		std::atomic<bool>& stop_restoration,
		std::atomic<bool>& restoration_finished,
		std::mutex& fft_mutex,
		ScreenParamsHDMI20 screen_params = ScreenParamsHDMI20{}
	);

	void RestoreHDIM20_2Channels(
		std::string& filename,
		const DataHeader_New& header,
		AnalysisResults& analysis,
		WaveConsole& console,
		std::atomic<bool>& stop_restoration,
		std::atomic<bool>& restoration_finished,
		std::mutex& fft_mutex,
		ScreenParamsHDMI20 screen_params = ScreenParamsHDMI20{}
	);

	void RestoreHDIM20_2Channels_Phase(
		std::string& filename,
		const DataHeader_New& header,
		AnalysisResults& analysis,
		WaveConsole& console,
		std::atomic<bool>& stop_restoration,
		std::atomic<bool>& restoration_finished,
		std::mutex& fft_mutex,
		ScreenParamsHDMI20 screen_params = ScreenParamsHDMI20{}
	);

	// Используется для поиска точного значения пиксельной частоты
	// что нужно для восстановления по фазе
	// 
	// @param fc - центральная частот, Гц
	// @param f_cadr - кадровая частота, Гц
	// @param fd - частота дискретизации, Гц
	// @param f_pix_rough - грубая оценка пиксельной, Гц
	// @param num_frames - число кадров в файле
	double getExactPixelFrequency(
		const std::string& filename,
		double fc, double f_cadr, double fd, double f_pix_rough,
		unsigned int num_frames,
		AnalysisResults& analysis,
		bool is_in_normal = true
	);
#if 1
	// Тестовая функция по реализации метода, предложенного в статье
	double getPixelClockFromCorr(const std::string& filename,
		double fc, double fd, double f_cadr, double f_pix,
		bool is_in_normal = true);



#endif
	void GetModulationFrequency_int(
		std::vector<std::complex<int16_t>>& data,
		double fd,
		const int window_length,
		const int hop_size,
		double& result,
		Spectrum& freq_saw_sp
	);

	void GetModulationFrequency_float(
		std::vector<std::complex<float>>& data,
		double fd,
		const int window_length,
		const int hop_size,
		double& result,
		Spectrum& freq_saw_sp
	);
	// Функции GetStringFrequency ищут строчную частоту в диапазоне от 60 до 70 кГц. 
	// В будущем надо писать нечто более универсальное
	void GetStringFrequency_int(const std::vector<std::complex<int16_t>>& data, double fd, double& result);
	void GetStringFrequency_float(const std::vector<std::complex<float>>& data, double fd, double& result);
	
	// Функция восстановления изображения из файла
	// @param filename - имя файла с данными, по которому восстанавливается изображение
	// @param string_freq - строчная частота, Гц
	// @param out_folder - папка, в которую сохраняются изображения
	// @param frames - сколько кадров нужно восстановить. По умолчанию 0 - все, что есть в файле
	bool RestoreImage(
		const std::string& filename,
		double string_freq,
		double modulation_frequency,
		std::string& out_folder,
		int windowSize,
		int hopSize,
		const int frames = 0);

	double GetExactStringFrequency_Corvet(const std::string& filename, DataHeader& header,
		const int horTotal, const int verTotal,
		double frequensy_range, double frequency_step, double string_frequency,
		double modulation_frequency, const int window_length, const int hop_size);

	double GetExactStringFrequency_USRP(const std::string& filename, DataHeader& header,
		const int horTotal, const int verTotal,
		double frequensy_range, double frequency_step, double string_frequency,
		double modulation_frequency, const int window_length, const int hop_size);

	// Функция поиска точной строчной частоты для Corvet
	double GetExactStringFrequency_Final(
		std::vector<std::complex<int16_t>>& data,
		int shift,
		double frequency_range, double frequency_step,
		DataHeader& header, const int horTotal,
		const int verTotal, double string_frequency,
		double modulation_frequency,
		const int hop_size, AnalysisResults& analysis, double freq_dev_ampl);
	
	template <typename T>
	double GetExactStringFrequency_HDMI(
		std::vector<std::complex<T>>& data,
		double frequency_range, double frequency_step,
		DataHeader& header, const int horTotal, const int verTotal,
		double string_frequency, AnalysisResults& analysis)
	{
		double dev_start = string_frequency - frequency_range / 2.0;
		double dev_stop = string_frequency + frequency_range / 2.0;
		double fd = header.sampling_frequency;
		size_t num_tries = static_cast<size_t>((dev_stop - dev_start) / frequency_step);
		size_t k = 0;
		std::vector<double> result(num_tries);

		// Прочитать можно только один раз. Потом с этим работать
		unsigned samples_per_cadr = std::ceil(fd / (dev_start)) * verTotal;
		std::vector<double> tmp_data(samples_per_cadr);
		std::vector<double> tmp_data2(horTotal * verTotal);
		analysis.hor_sum_freqs.resize(num_tries);
		int freq_num = 0;

		// Восстановление одного кадра
		for (double freq_dev = dev_start; freq_dev < dev_stop; freq_dev = freq_dev + frequency_step)
		{
			analysis.hor_sum_freqs[freq_num] = freq_dev;
			freq_num++;
			std::cout << freq_dev << std::endl;
			double deviated = freq_dev;

			double fPix = deviated * horTotal;
			double fCadr = deviated / verTotal;

			// samples_per_cadr должно быть кратно числу строк в кадре
			samples_per_cadr = std::ceil(fd / deviated) * verTotal;

			for (size_t j = 0; j < samples_per_cadr; j++)
			{
				tmp_data[j] = sqrt(data[j].real() * data[j].real() + data[j].imag() * data[j].imag());
			}

			float rec_dt = 1 / fd;
			std::vector<double> rec_time(samples_per_cadr);
			for (size_t j = 0; j < samples_per_cadr; j++)
			{
				rec_time[j] = j * rec_dt;
			}

			float dt = 1 / fPix;
			size_t L = horTotal * verTotal;
			std::vector<double> true_time(L);
			for (size_t j = 0; j < L; j++)
				true_time[j] = j * dt;

			tmp_data2 = interp(rec_time, tmp_data, true_time);
			std::vector<double> colSum(horTotal);
			for (size_t j = 0; j < horTotal; j++)
			{
				for (size_t i = 0; i < verTotal; i++)
				{
					colSum[j] += tmp_data2[i * horTotal + j];
				}
			}
			auto dif_sum = diffSig(colSum);
			auto sum_max = std::ranges::max_element(colSum);
			auto sum_min = std::ranges::min_element(colSum);

			auto maximum = std::max_element(dif_sum.begin(), dif_sum.end());
			auto minimum = std::min_element(dif_sum.begin(), dif_sum.end());
			result[k] = *maximum - *minimum + (*sum_max - *sum_min);
			k++;
		}
		result = moving_average(result, 10);
		analysis.hor_sum_vector = result;
		auto ind = std::distance(result.begin(), std::max_element(result.begin(), result.end()));
		auto new_string_frequency = dev_start + ind * frequency_step;
		std::cout << "New String " << new_string_frequency << std::endl;
		auto new_frequency_range = frequency_range / 5.0;
		auto new_frequency_step = frequency_step / 5.0;
		return new_string_frequency;
	}
	// Функция поиска точной строчной частоты для USRP
	double GetExactStringFrequency_Final(
		std::vector<std::complex<float>>& data,
		int shift,
		double frequency_range, double frequency_step,
		DataHeader& header, const int horTotal,
		const int verTotal, double string_frequency,
		double modulation_frequency,
		const int hop_size,
		AnalysisResults& analysis, 
		double freq_dev_ampl);
	
	double GetExactLineFrequency_New(std::vector<std::complex<float>>& data,
									 DataHeader& header, const int horTotal, const int verTotal, double line_freq,
									 AnalysisResults& analysis);

	// Функция восстановления сигнала, сделанного сторонним приемником
	void RestoreImageCustom(std::string& filename, 
		const float center_frequency, const float fd,
		AnalysisResults& analysis,
		WaveConsole& console,
		std::atomic<bool>& stop_restoration,
		std::atomic<bool>& restoration_finished,
		std::mutex& fft_mutex, 
		bool find_exact_frequency,
		const int window_length, const int hop_size, 
		ScreenParamsLVDS screenParams, bool restore_as_HDMI = false,
		std::filesystem::path current_folder = std::filesystem::path{},
		bool invert_phase = false, 
		double phase_shift = 0.0,
		bool sync_with_phase = false);

	void RestoreImageCorvet(std::string& filename,
		DataHeader& header,
		AnalysisResults& analysis,
		WaveConsole& console,
		std::atomic<bool>& stop_restoration,
		std::mutex& fft_mutex,
		bool find_exact_frequency,
		const int window_length, const int hop_size, unsigned frames,
		const float f_cadr, const float f_string, const float f_mod,
		const int in_verTotal, const int in_horTotal,
		unsigned buffer_size,
		float freq_dev_ampl = 0.0f);

	// Восстановление LVDS для Агата и USRP
	void RestoreImageLVDS(
		std::string& filename,
		DataHeader_New& header,
		AnalysisResults& analysis,
		WaveConsole& console,
		std::atomic<bool>& stop_restoration,
		std::atomic<bool>& restoration_finished,
		std::mutex& fft_mutex,
		std::filesystem::path current_folder,
		bool find_exact_frequency = false,
		ScreenParamsLVDS screenParams = ScreenParamsLVDS{},
		const int window_length = 256, const int hop_size = 30
	);

	void RestoreImageLVDS2Channels(std::string& filename,
		DataHeader_New& header,
		AnalysisResults& analysis,
		WaveConsole& console,
		std::atomic<bool>& stop_restoration,
		std::atomic<bool>& restoration_finished,
		std::mutex& fft_mutex,
		std::filesystem::path current_folder,
		bool find_exact_frequency,
		ScreenParamsLVDS screenParams = ScreenParamsLVDS{},
		const int window_length = 256,
		const int hop_size = 30);
	
	void RestoreImageCorvet_HDMI(
		std::string& filename,
		DataHeader& header,
		AnalysisResults& analysis,
		WaveConsole& console,
		std::atomic<bool>& stop_restoration,
		std::mutex& fft_mutex,
		bool find_exact_frequency,
		unsigned frames,
		const float f_cadr, const float f_string,
		const int in_verTotal, const int in_horTotal,
		unsigned buffer_size
	);

	cv::Mat1b shiftImage(cv::Mat1b& src_image, int dx, int dy);

	// если find_exact_frequency == true то используется долгий алгоритм поиска точной строчной частоты
	void RestoreImageUSRP(std::string& filename, DataHeader& header, AnalysisResults& analysis, WaveConsole& console,
		std::atomic<bool>& stop_restoration, std::mutex& fft_mutex,
		bool find_exact_frequency = false,
		const int window_length = 256, const int hop_size = 30, unsigned frames = 0,
		const float f_cadr = 0.0f, const float f_string = 0.0f, const float f_mod = 0.0f,
		const int in_verTotal = 0, const int in_horTotal = 0,
		float freq_dev_ampl = 0.0f);

	void RestoreImageUSRP_HDMI(std::string& filename,
		DataHeader& header, AnalysisResults& analysis,
		WaveConsole& console,
		std::atomic<bool>& stop_restoration, std::mutex& fft_mutex,
		bool find_exact_frequency = false,
		unsigned frames = 0,
		const float f_cadr = 0.0f, const float f_string = 0.0f,
		const int in_verTotal = 0, const int in_horTotal = 0);

	// Синхронизирует изображения
	void RTSynchronize(
		std::vector<double>& imageData,
		const std::vector<float>& etalon_string,
		const std::vector<float>& etalon_cadr,
		const int horTotal, const int verTotal);


	struct ShiftDims
	{
		int shift_hor = 0;
		int shift_ver = 0;
	};

	void Synchronize(
		std::vector<double>& imageData,
		const std::vector<float>& etalon_string,
		const std::vector<float>& etalon_cadr,
		const int horTotal, const int verTotal,
		ShiftDims& output,
		bool is_lvds = false);

	void GetScreenParams(std::string& filename, DataHeader& header, AnalysisResults& analysis, double f_mod = 0.0);
	void GetScreenParams_HDMI(std::string& filename, DataHeader& header, AnalysisResults& analysis);
	void GetScreenParams_Custom(AnalysisResults& analysis, float fd, std::vector<double>& decimated_signal, double f_mod = 0.0);
	// Запись промежуточных результатов восстановления в файл
	void SaveAnalysisResult(const std::filesystem::path& path, const AnalysisResults& analysisResults);
	// Чтение промежуточных результатов восстановления из файла
	bool ReadAnalysisResult(const std::filesystem::path& path, AnalysisResults& analysisResults);
	// Удаляет мощную синусоиду из спектрограммы
	void ProcessSpectrogram(std::vector<std::vector<double>>& spe, AnalysisResults& analysis);
	
	struct ImageVectors
	{
		std::vector<double>& first_frame;
		std::vector<std::complex<double>>& sigIQ;
		std::vector<std::complex<double>>& frame_int;
		std::vector<double>& abs_interp;
		std::vector<double>& phase_data;
		std::vector<std::complex<double>>& complexDataEtalon;
		std::vector<std::complex<double>>& comdiv;
		std::vector<std::complex<double>>& tmp_comdiv;
		std::complex<double>& comMean;

		std::vector<float>& etalon_string;
		std::vector<float>& etalon_cadr;

		std::vector<float>& avg_image; // амплитудное накопленное изображение
		std::vector<float>& every_cadr_image; // амплитудное изображение
		std::vector<float>& avg_image_phase; // фазовое изображение

		std::vector<std::complex<double>> filtered_etalon;
		std::vector<std::complex<double>> filtered_sigIQ;
	};

	// Хранение для данных с двух каналов
	struct ImageVectors2Channels
	{
		std::vector<std::complex<double>>& frame_int_2_chan;
		std::vector<double>& abs_interp_2_chan;
		std::vector<double>& phase_data_2_chan;
		std::vector<float>& avg_image_2_chan;
		std::vector<float>& every_image_2_chan;
		std::vector<float>& phase_image_2_chan;
		std::vector<float>& every_phase_image_2_chan;
	};

	struct ImageVectorsHDMI20
	{
		std::vector<double>& first_frame;
		std::vector<std::complex<double>>& sigIQ;
		std::vector<std::complex<double>>& frame_int;
		std::vector<double>& abs_interp;
		std::vector<double>& scrambler;
		std::vector<double>& scrambler_ch2;
		std::vector<float>& etalon_string;
		std::vector<float>& etalon_cadr;
		std::vector<float>& avg_image; // амплитудное изображение
		std::vector<float>& every_cadr_image; // амплитудное изображение
	};

	// Функция содержит конкретно цикл восстановления изображений для интерфейса HDMI
	void RestoreAndSaveImages_HDMI(
		unsigned int frame_amount,
		unsigned int frame_length,
		AnalysisResults& analysis,
		WaveConsole& console,
		std::ifstream& file,
		std::atomic_bool& stop_restoration,
		std::atomic_bool& restoration_finished,
		ImageVectors& vectors,
		ScreenParamsHDMI screenParams,
		std::filesystem::path& sum_out_folder,
		std::filesystem::path& every_out_folder,
		bool is_in_normal = true
	);

	void RestoreAndSaveImages_HDMI_2Channels
	(
		unsigned int frame_amount,
		unsigned int frame_length,
		AnalysisResults& analysis,
		WaveConsole& console,
		std::ifstream& file,
		std::ifstream& file_2,
		std::atomic_bool& stop_restoration,
		std::atomic_bool& restoration_finished,
		ImageVectors& vectors,
		ImageVectors2Channels& vectors_2chan,
		ScreenParamsHDMI screenParams,
		std::filesystem::path& sum_out_folder,
		std::filesystem::path& every_out_folder,
		bool is_in_normal = true
	);

	std::string FindSecondChannelFilename(std::string& filename, WaveConsole& console);

	// Функция содержит конкретно цикл восстановления изображений для интерфейса HDMI 2.0
	void RestoreAndSaveImagesHDMI20(
		unsigned int frame_amount,
		unsigned int frame_length,
		AnalysisResults& analysis,
		WaveConsole& console,
		std::ifstream& file,
		std::atomic_bool& stop_restoration,
		std::atomic_bool& restoration_finished,
		ImageVectorsHDMI20& vectors,
		ScreenParamsHDMI20 screenParams,
		std::filesystem::path& sum_out_folder,
		std::filesystem::path& every_out_folder,
		bool is_in_normal
	);
	void RestoreAndSaveImagesHDMI20_phase(
		unsigned int frame_amount,
		unsigned int frame_length,
		AnalysisResults& analysis,
		WaveConsole& console,
		std::ifstream& file,
		std::atomic_bool& stop_restoration,
		std::atomic_bool& restoration_finished,
		ImageVectorsHDMI20& vectors,
		ScreenParamsHDMI20 screenParams,
		std::filesystem::path& sum_out_folder,
		std::filesystem::path& every_out_folder,
		bool is_in_normal
	);
	void RestoreAndSaveImagesHDMI20_2Channels(
		unsigned int frame_amount,
		unsigned int frame_length,
		AnalysisResults& analysis,
		WaveConsole& console,
		std::ifstream& file,
		std::ifstream& file2,
		std::atomic_bool& stop_restoration,
		std::atomic_bool& restoration_finished,
		ImageVectorsHDMI20& vectors,
		ScreenParamsHDMI20 screenParams,
		std::filesystem::path& sum_out_folder,
		std::filesystem::path& every_out_folder,
		bool is_in_normal
	);

	void RestoreAndSaveImagesHDMI20_2Channels_Phase(
		unsigned int frame_amount,
		unsigned int frame_length,
		AnalysisResults& analysis,
		WaveConsole& console,
		std::ifstream& file,
		std::ifstream& file2,
		std::atomic_bool& stop_restoration,
		std::atomic_bool& restoration_finished,
		ImageVectorsHDMI20& vectors,
		ScreenParamsHDMI20 screenParams,
		std::filesystem::path& sum_out_folder,
		std::filesystem::path& every_out_folder,
		bool is_in_normal
	);

	void Restore2ChanHDMI(
		std::string& filename,
		const DataHeader_New& header,
		AnalysisResults& analysis,
		WaveConsole& console,
		std::atomic<bool>& stop_restoration,
		std::atomic<bool>& restoration_finished,
		std::mutex& fft_mutex,
		ScreenParamsHDMI screen_params);
	

#if 0
	void RestoreImageCorvet_HDMI(std::string& filename,
		DataHeader& header, AnalysisResults& analysis,
		WaveConsole& console, std::atomic<bool>& stop_restoration,
		std::mutex& fft_mutex,
		bool find_exact_frequency, unsigned frames, const float f_cadr,
		const float f_string, const int in_verTotal, const int in_horTotal, unsigned buffer_size)
	{
		// Заполнить эту функцию

		auto data = readCorvetDataFile(filename);
		size_t N = data.size() / buffer_size;
		if (data.empty())
		{
			std::cout << "Failed to open data file!\n";
			console.AddLog(u8"Не удалось открыть файл!");
			return;
		}
		// Файл может быть очень большим. Поэтому читаем его часть, которой, однако, достаточно
		// для точного определения частоты пилы
		size_t analysis_size = 2e7;
		std::vector<std::complex<int16_t>> decimate_data;
		if (analysis_size > data.size())
		{
			decimate_data = std::move(data);
			analysis_size = decimate_data.size();
		}
		else
		{
			decimate_data.resize(analysis_size);
			for (size_t i = 0; i < analysis_size; i++)
			{
				decimate_data[i] = data[i];
			}
		}
		if (!data.empty())
			data.clear();

		// Рассчитываем и сохраняем спектр сигнала
		{
			std::lock_guard lk{ fft_mutex };
			analysis.signal_fft = getSpectr(decimate_data, FFT_SIZE);
			analysis.central_frequency = header.center_frequency / 1e6;
			analysis.sampling_frequency = header.sampling_frequency;
		}

		double string_frequency = 0.0;
		std::string minifile = filename.substr(0, filename.size() - 5) + ".mini";

		// Ищет только кадровую, строковую и размеры экрана
		{
			std::lock_guard lk{ fft_mutex };
			GetScreenParams_HDMI(minifile, header, analysis);
		}

		int horTotal = analysis.m_monitor_parameters.horTotal;
		int verTotal = analysis.m_monitor_parameters.verTotal;
		string_frequency = analysis.m_monitor_parameters.string_frequency;

		{
			std::string log = "Number of columns " + std::to_string(verTotal);
			console.AddLog(log.c_str());
			log = "Number of rows " + std::to_string(horTotal);
			console.AddLog(log.c_str());
			log = "String frequency from function 'GetScreenParams' " + std::to_string(string_frequency);
			console.AddLog(log.c_str());
		}
		// Создание папок для хранения восстановленных изображений
		auto str_len = filename.size();
		auto Name = filename.substr(str_len - 13, 8);
		auto string_center = std::to_string(static_cast<int>(header.center_frequency / 1e6));
		auto string_center_s = std::to_string(static_cast<int>(header.center_frequency / 1e6));

		std::filesystem::path test_path = filename;
		std::filesystem::path folder_path = test_path.parent_path();
		std::filesystem::path tmp_path = folder_path;
		std::filesystem::path another_tmp_path = folder_path;

		std::filesystem::path every_out_folder = tmp_path.append(Name + "\\Every\\");
		std::filesystem::path sum_out_folder = folder_path.append(Name + "\\Sum\\");

		std::filesystem::create_directories(every_out_folder);
		std::filesystem::create_directories(sum_out_folder);

		double freq_range = 75.0;
		double freq_step = 0.25;

		double fd = header.sampling_frequency;
		if (f_string > 0.0f)
		{
			string_frequency = f_string;
			analysis.m_monitor_parameters.string_frequency = f_string;
		}

		int samples_per_cadr = std::ceil(fd / (string_frequency)) * verTotal;
		if (find_exact_frequency)
		{
			samples_per_cadr = std::ceil(fd / (string_frequency - freq_range / 2.0)) * verTotal;
		}

		std::ifstream file(filename, std::ios::binary);
		if (!file.is_open()) {
			console.AddLog("Failed to open signal file.");
			return;
		}

		// Читаем один кадр
		std::vector<std::complex<int16_t>> temp_data(samples_per_cadr);
		file.seekg(samples_per_cadr * sizeof(std::complex<int16_t>));
		file.read(reinterpret_cast<char*>(temp_data.data()), sizeof(std::complex<int16_t>) * samples_per_cadr);
		file.seekg(0, std::ios::beg);

		if (find_exact_frequency)
		{
			console.AddLog("Exact frequency calculation started");
			string_frequency = GetExactStringFrequency_HDMI(temp_data, freq_range, freq_step, header, horTotal, verTotal, string_frequency, analysis);
			std::string log = "Exact frequency calculation finishes. Result frequency: " + std::to_string(string_frequency);
			console.AddLog(log.c_str());
		}

		{
			std::string log = "\n\n";
			console.AddLog(log.c_str());
			console.AddLog("Restored image parameters: ");
			log = "Image size: " + std::to_string(horTotal) + "x" + std::to_string(verTotal);
			console.AddLog(log.c_str());
			log = "Refresh rate: " + std::to_string(analysis.m_monitor_parameters.fcadr);
			console.AddLog(log.c_str());
			log = "String frequency: " + std::to_string(string_frequency);
			console.AddLog(log.c_str());
			log = "\n\n";
			console.AddLog(log.c_str());
		}

		double fPix = string_frequency * horTotal;
		double fCadr = string_frequency / verTotal;

		samples_per_cadr = std::ceil(fd / string_frequency) * verTotal;

		// Рассчитываем cколько "кадров" в одном файле
		file.seekg(0, std::ios::end);
		std::streampos fileSize = file.tellg();
		file.seekg(0, std::ios::beg);
		size_t number_of_frames = std::floor(static_cast<double>(fileSize) / sizeof(std::complex<int16_t>) / samples_per_cadr);
		if (frames <= number_of_frames && frames != 0)
			number_of_frames = frames;
		std::cout << number_of_frames << std::endl;
		{
			std::string log = "Current file contains " + std::to_string(number_of_frames) + " frames";
			console.AddLog(log.c_str());
		}

		// Выделение памяти для данных
		std::vector<std::complex<int16_t>> complexData(samples_per_cadr);

		// Покадровое восстановление
		// Заглушки для синхронизации
		std::vector<float> etalon_string(horTotal);
		std::vector<float> etalon_cadr(verTotal);
		std::vector<float> avg_image(horTotal * verTotal);
		std::vector<float> avg_phase_image(horTotal * verTotal);

		console.AddLog(u8"------ НАЧАТО ВОССТАНОВЛЕНИЕ ------");
		// Цикл восстановления кадров
		for (size_t i = 0; i < number_of_frames - 1; i++)
		{
			// Условие досрочного завершения восстановления
			if (stop_restoration.load())
			{
				file.close();
				console.AddLog(u8" ----- ВОССТАНОВЛЕНИЕ ПРЕРВАНО ----- ");
				stop_restoration.store(false);
				return;
			}

			file.read(reinterpret_cast<char*>(complexData.data()), sizeof(std::complex<int16_t>) * samples_per_cadr);
			// Заполняем вспомогательные данные
			// Для HDMI здесь что-то интересное придется далать
			if (analysis.IsEmpty())
			{
				double last_time = *(analysis.m_freq_saw.time.end() - 1);
				analysis.m_is_emty = false;
			}

			// Вектор амплитудных значений
			std::vector<double> tmp_data(complexData.size());
			std::vector<double> phase_data(complexData.size());
			std::vector<double> tmp_data2;

			double f_pix = f_string * horTotal;
			std::vector<std::complex<double>> data_with_correction(complexData.size());
			const std::complex<double> comp_i(0.0, 1.0);
			for (size_t j = 0; j < tmp_data.size(); j++)
			{
				tmp_data[j] = sqrt(complexData[j].real() * complexData[j].real() + complexData[j].imag() * complexData[j].imag());
				phase_data[j] = wave::arg<int16_t>(complexData[j]) * 180 / 3.14159265;
			}

			float rec_dt = 1 / fd;
			std::vector<double> rec_time(samples_per_cadr);
			for (size_t j = 0; j < samples_per_cadr; j++)
			{
				rec_time[j] = j * rec_dt;
			}
			float dt = 1 / fPix;
			size_t L = horTotal * verTotal;
			std::vector<double> true_time(L);
			for (size_t j = 0; j < L; j++)
			{
				true_time[j] = j * dt;
			}
			tmp_data2 = interp(rec_time, tmp_data, true_time);
			auto phase_data2 = interp(rec_time, phase_data, true_time);
			auto max_elem = std::max_element(tmp_data2.begin(), tmp_data2.end());
			auto max_phase_elem = std::max_element(phase_data2.begin(), phase_data2.end());
			std::vector<char> image(L);
			std::vector<char> phase_image(L);
			for (size_t j = 0; j < L; j++)
			{
				image[j] = static_cast<char>(tmp_data2[j] / (*max_elem) * 255);
				phase_image[j] = static_cast<char>(phase_data2[j] / (*max_phase_elem) * 255);
			}


			// Код синхронизации
			const float alpha = 0.05;
			if (i == 0)
			{
				continue;
			}
			else if (i == 1)
			{
				for (size_t j = 0; j < horTotal; j++)
				{
					for (size_t i = 0; i < verTotal; i++)
					{
						etalon_cadr[i] += (float)tmp_data2[i * horTotal + j];
						etalon_string[j] += (float)tmp_data2[i * horTotal + j];
						avg_image[i * horTotal + j] = (float)tmp_data2[i * horTotal + j];
						avg_phase_image[i * horTotal + j] = (float)phase_data2[i * horTotal + j];
					}
				}
				analysis.hor_sum = etalon_cadr;
				analysis.ver_sum = etalon_string;
			}
			else
			{
				// Проверочный код. Пока оставлю
				std::vector<float> tmp_ealon(verTotal);
				for (size_t j = 0; j < horTotal; j++)
				{
					for (size_t i = 0; i < verTotal; i++)
					{
						tmp_ealon[i] += (float)tmp_data2[i * horTotal + j];
					}
				}
				analysis.ver_sum = tmp_ealon;
				// Проверочный код до сюда
				ShiftDims shift_dims{};
				Synchronize(tmp_data2, etalon_string, etalon_cadr, horTotal, verTotal, shift_dims);
				phase_data2 = circshift(phase_data2, phase_data2.size(), 1, -(shift_dims.shift_ver * horTotal + shift_dims.shift_hor), 0);

				for (size_t j = 0; j < L; j++)
				{
					//avg_image[j] = ((1 - alpha) * avg_image[j] + alpha * (float)tmp_data2[j]);
					avg_image[j] += (float)tmp_data2[j];
					avg_phase_image[j] += (float)phase_data2[j];
				}
			}

			auto m_el = std::max_element(avg_image.begin(), avg_image.end());
			auto m_phase_el = std::max_element(avg_phase_image.begin(), avg_phase_image.end());


			std::vector<char> tmp_image(avg_image.size());
			std::vector<char> tmp_phase_image(avg_image.size());
			for (size_t elem = 0; elem < avg_image.size(); elem++)
			{
				tmp_image[elem] = static_cast<char>(avg_image[elem] / (*m_el) * 255);
				tmp_phase_image[elem] = static_cast<char>(avg_phase_image[elem] / (*m_phase_el) * 255);
			}

			// Сохранение данных
			std::string	file_number = (i < 100) ? ((i < 10) ? "00" + std::to_string(i) : "0" + std::to_string(i)) : std::to_string(i);
			std::string avg_filename = sum_out_folder.string() + file_number + ".png";
			//std::string avg_phase_filename = sum_out_folder.string() + file_number + "_phase.png";
			std::string filename = every_out_folder.string() + file_number + ".png";
			//std::string phase_filename = every_out_folder.string() + file_number + "_phase.png";

			stbi_write_png(filename.c_str(), horTotal, verTotal, 1, image.data(), horTotal);
			//	stbi_write_png(phase_filename.c_str(), horTotal, verTotal, 1, phase_image.data(), horTotal);

			cv::Mat1b dst(verTotal, horTotal);
			cv::Mat1b dst_phase(verTotal, horTotal);

			cv::Mat1b zero_mat = cv::Mat::zeros(verTotal, horTotal, CV_8UC1);
			cv::Mat hsv_img, chans[3];

			imadjust(tmp_image, dst, horTotal, verTotal, 1);
			imadjust(tmp_phase_image, dst_phase, horTotal, verTotal, 1);

			chans[0] = dst_phase;
			chans[1] = zero_mat;
			chans[2] = dst;

			cv::merge(chans, 3, hsv_img);
			cv::imwrite(avg_filename, dst);
			//	cv::imwrite(avg_phase_filename, dst_phase);
			if (i == (number_of_frames - 2))
			{
				std::string result_filename = another_tmp_path.filename().string() + "_" + Name + ".png";
				auto full_save_path = another_tmp_path.append(result_filename);
				cv::imwrite(full_save_path.string().c_str(), dst);
			}

			std::string log = "Restored " + std::to_string(i) + " frames";
			console.AddLog(log.c_str());
		}
		std::string analysis_filename = another_tmp_path.filename().string() + "_" + Name + ".an";
		auto full_analysys_path = another_tmp_path.parent_path().append(analysis_filename);
		SaveAnalysisResult(full_analysys_path, analysis);
		std::cout << full_analysys_path.string() << std::endl;
		std::cout << string_frequency << std::endl;
		std::cout << f_string << std::endl;
		file.close();
		console.AddLog(u8"------ ВОССТАНОВЛЕНИЕ ЗАВЕРШЕНО ------");
		return;

	}
#endif
}
