#pragma once

#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"
#include <stdio.h>
#include <cstddef>
#include <vector>
#include <cmath>
#include <numeric>

#include <GLFW/glfw3.h> // Will drag system OpenGL headers
#include <gsl/gsl_interp.h>
#include <opencv2/opencv.hpp>

constexpr int DEFAULT_FFT_EXPONENT = 15;
constexpr int MIN_FFT_EXPONENT = 11;
constexpr int MAX_FFT_EXPONENT = 16;
constexpr size_t DEFAULT_FFT_SIZE = size_t{1} << DEFAULT_FFT_EXPONENT;
const std::vector<double> FREQUENCIES = std::vector<double>{5, 10, 60, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200,
2300, 2400, 2500, 2600, 2700, 2800, 2900, 3000, 3100, 3200, 3300, 3400, 3500, 3600, 3700, 3800, 3900, 4000, 4100, 4200, 4300, 4400, 4500, 4600, 4700, 4800, 4900,
5000, 5100, 5200, 5300, 5400, 5500, 5600, 5700, 5800, 5900, 6000, 6100 };



const std::vector<double> CALIBRATION_COEFFICIENTS = std::vector<double>{ -89.25,-89.25, -85.34, -85.26,-83.87,-84.35,-84.12,-90.93,-91.42,-90.74,-88.92,-88.76,-88.78,-86.92,-87.25,-86.40,-84.09,-88.26,-87.70,-86.66,-87.57,-87.00,-84.34,-85.24,-84.23,-82.33,-83.05,-81.93,-80.66,-81.13,-80.41,-78.67,-79.06,-79.49,-79.38,
-79.98,-79.33,-78.71,-80.38,-79.19,-77.03,-77.82,-76.49,-75.89,-75.33,-73.47,-73.95,-73.14,-72.52,-72.86,-72.66,-72.43,-72.73,-73.22,-71.93,-72.42,-72.88,-70.58,-71.29,-70.49,-67.79,-66.98,-63.42 };

template<typename T>
struct CrossCorrelation
{
    std::vector<T> corr_coeff;
    std::vector<T> lags;
    void clear()
    {
        corr_coeff.clear();
        lags.clear();
    }
};

template<typename T>
std::vector<T> log_lin_interp(std::vector<T> frequencies)
{
	std::vector<T> result(frequencies.size());
	for (int i = 0; i < result.size(); i++)
	{
		T f = frequencies[i];
		for (int j = 0; j < FREQUENCIES.size()-1; j++)
		{
			if (f >= FREQUENCIES[j] && f < FREQUENCIES[j + 1])
			{
				T a = CALIBRATION_COEFFICIENTS[j];
				T b = CALIBRATION_COEFFICIENTS[j + 1];
				T f_a = FREQUENCIES[j];
				T f_b = FREQUENCIES[j + 1];
				result[i] = (b*std::log10(f/f_a) + a*std::log10(f_b/f))/std::log10(f_b / f_a);
				break;
			}
		}
	}
	return result;
}

// Шаблон функции для изменения значения колесиком мыши
template<typename T>
void changeWithMouseWheel(T& value, double accuracy = 1)
{
	if (!ImGui::IsItemHovered())
		return;
	ImGui::SetItemKeyOwner(ImGuiKey_MouseWheelY);
	if (ImGui::IsItemHovered())
	{
		T wheel = static_cast<T>(ImGui::GetIO().MouseWheel);
		if (wheel)
		{
			if (ImGui::IsItemActive())
			{
				ImGui::SetKeyboardFocusHere(-1);
			}
			else
			{
				value += wheel / accuracy;
			}
		}
	}
}

template<typename T>
void VectorPadding(std::vector<T>& input, size_t L)
{
    if (input.size() < L)
        input.resize(L);
    return;
}

template<typename T>
std::vector<T> circshift(const std::vector<T>& in, int xdim, int ydim, int xshift, int yshift)
{
    std::vector<T> out(in.size());
    for (int i = 0; i < xdim; i++) {
        int ii = (i + xshift) % xdim;
        if (ii < 0) ii = xdim + ii;
        for (int j = 0; j < ydim; j++) {
            int jj = (j + yshift) % ydim;
            if (jj < 0) jj = ydim + jj;
            out[ii * ydim + jj] = in[i * ydim + j];
        }
    }
    return out;
}

// Переменная n - длина корреляции
template<typename T>
float find_coefficient(const std::vector<T>& X, const std::vector<T>& Y, int n) {
    T sum_X = 0, sum_Y = 0, sum_XY = 0;
    T squareSum_X = 0, squareSum_Y = 0;
    for (int i = 0; i < n; i++) {
        sum_X = sum_X + X[i];
        sum_Y = sum_Y + Y[i];
        sum_XY = sum_XY + X[i] * Y[i];
        squareSum_X = squareSum_X + X[i] * X[i];
        squareSum_Y = squareSum_Y + Y[i] * Y[i];
    }
    float corr = (float)(n * sum_XY - sum_X * sum_Y) / sqrt((n * squareSum_X - sum_X * sum_X) * (n * squareSum_Y - sum_Y * sum_Y));
    return corr;
}

template<typename T>
CrossCorrelation<T> shiftCorrelation(const std::vector<T>& X, const std::vector<T>& Y, int n)
{
    CrossCorrelation<T> corr_function;
    corr_function.corr_coeff.resize(n);
    corr_function.lags.resize(n);
    std::vector<T> tmp(n);
    for (int i = 0; i < n; i++)
        tmp[i] = X[i];
    int shift = 0;
    for (int i = 0; i < n; i++)
    {
        // Каждый раз сдвигаем на 1 отсчёт, не на n
        tmp = circshift(tmp, n, 1, shift, 0);
       
        //tmp = vector_shift(tmp, shift);
        corr_function.corr_coeff[i] = find_coefficient(tmp, Y, n);
        corr_function.lags[i] = i;
        shift = 1;
    }
    return corr_function;
}



template<typename T>
std::vector<T> diffSig(std::vector<T>& sig)
{
    int N = sig.size();
    std::vector<T> result(N);
    for (size_t i = 0; i < N - 1; i++) {
        result[i] = sig[i + 1] - sig[i];
    }
    //result[N] = result[N-1];
    return result;
}

template<typename T>
std::vector<T> elem_dif(std::vector<T>& vec1, std::vector<T>& vec2)
{
    if (vec1.size() != vec2.size())
        return std::vector<T>();
    std::vector<T> result(vec1.size());
    for (size_t i = 0; i < vec1.size(); i++)
    {
        result[i] = vec1[i] - vec2[i];
    }
    return result;
}

template<typename T>
double var(std::vector<T>& data)
{
    if (data.size() <= 1)
        return 0.0;

    double sum = std::accumulate(data.begin(), data.end(), 0.0);
    double mean = sum / data.size();

    double sumSquareDiff = 0.0;
    for (auto& value : data)
    {
        sumSquareDiff += std::pow(value - mean, 2);
    }
    return sumSquareDiff / (data.size() - 1);
}

std::vector<double> interp(std::vector<double>& x, std::vector<double>& y, std::vector<double>& x_new);
std::vector<double> interp_double(std::vector<double>& y, double sample_rate_old, double sample_rate_new, int L = 0);
void RTSynchronize(std::vector<double>& imageData, const std::vector<float>& etalon_string, const std::vector<float>& etalon_cadr, const int horTotal, const int verTotal);
std::vector <float> PlaneOnHor(std::vector<double>& sig, int verT, int horT);
std::vector<float> FindDHor(std::vector<float> vec1, std::vector<float> vec2);
std::vector <float> PlaneOnVer(std::vector<double>& sig, int verT, int horT);
void imadjust(const std::vector<char>& src, cv::Mat1b& dst, int horTotal, int verTotal, int tol = 1, cv::Vec2i in = cv::Vec2i(0,255), cv::Vec2i out = cv::Vec2i(0, 255));

