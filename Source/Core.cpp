#include "Core.h"

std::vector<double> interp(std::vector<double>& x, std::vector<double>& y, std::vector<double>& x_new)
{
    std::vector<double> y_new(x_new.size());
    const gsl_interp_type* t = gsl_interp_linear;
    gsl_interp* interp = gsl_interp_alloc(t, y.size());
    gsl_interp_accel* acc = gsl_interp_accel_alloc();

    gsl_interp_init(interp, x.data(), y.data(), x.size());
    for (size_t i = 0; i < x_new.size(); i++)
    {
        y_new[i] = gsl_interp_eval(interp, x.data(), y.data(), x_new[i], acc);
    }
    gsl_interp_free(interp);
    gsl_interp_accel_free(acc);
    return y_new;
}

std::vector<double> interp_double(std::vector<double>& y, double sample_rate_old, double sample_rate_new, int L)
{
    if (L == 0)
    {
        size_t length = y.size();
        std::vector<double> old_time(length);
        for (size_t j = 0; j < length; j++)
        {
            old_time[j] = j / sample_rate_old;
        }
        auto num_new_samples = std::ceil(*(old_time.end() - 1) * sample_rate_new);
        std::vector<double> new_time(num_new_samples);
        for (size_t j = 0; j < num_new_samples; j++)
        {
            new_time[j] = j / sample_rate_new;
        }
        auto abs_data = interp(old_time, y, new_time);

        return abs_data;
    }
    else
    {
        std::vector<double> new_time(L);
        for (size_t j = 0; j < L; j++)
        {
            new_time[j] = j / sample_rate_new;
        }
        int M = y.size();
        double dt = (L - 1) / sample_rate_new * M;

        std::vector<double> old_time(M);
        for (size_t j = 0; j < M; j++)
        {
            old_time[j] = j * dt;
        }
        std::vector<double> output = interp(old_time, y, new_time);

        return output;
    }

}

void RTSynchronize(std::vector<double>& imageData, const std::vector<float>& etalon_string, const std::vector<float>& etalon_cadr, const int horTotal, const int verTotal)
{
    std::vector<float> tmp_string(horTotal);
    std::vector<float> tmp_cadr(verTotal);
    tmp_string = PlaneOnHor(imageData, verTotal, horTotal);
    auto c_string = FindDHor(etalon_string, tmp_string);
    auto ind_hor = std::distance(c_string.begin(), std::min_element(c_string.begin(), c_string.end()));
    tmp_cadr = PlaneOnVer(imageData, verTotal, horTotal);
    CrossCorrelation corr_ver = shiftCorrelation(etalon_cadr, tmp_cadr, verTotal);
    auto ind_ver = std::distance(corr_ver.corr_coeff.begin(), std::max_element(corr_ver.corr_coeff.begin(), corr_ver.corr_coeff.end()));
    imageData = circshift(imageData, imageData.size(), 1, -ind_ver * horTotal + ind_hor, 0);
    return;
}

std::vector<float> PlaneOnHor(std::vector<double>& sig, int verT, int horT)
{
    std::vector<float> out(horT);
    for (size_t i = 0; i < verT; i++)
    {
        for (size_t j = 0; j < horT; j++)
        {
            out[j] += (float)sig[i * horT + j];
        }
    }
    return out;

}

std::vector<float> FindDHor(std::vector<float> vec1, std::vector<float> vec2)
{
    size_t N = vec1.size();
    std::vector<float> result(N);
    for (size_t i = 0; i < N; i++)
    {
        auto tmp = circshift(vec2, vec2.size(), 1, i, 0);
        auto tmp2 = elem_dif(vec1, tmp);
        result[i] = var(tmp2);
    }
    return result;
}

std::vector<float> PlaneOnVer(std::vector<double>& sig, int verT, int horT)
{
    std::vector<float> out(verT);
    for (size_t i = 0; i < verT; i++)
    {
        for (size_t j = 0; j < horT; j++)
        {
            out[i] += (float)sig[i * horT + j];
        }
    }
    return out;

}

void imadjust(const std::vector<char>& src, cv::Mat1b& dst, int horTotal, int verTotal, int tol, cv::Vec2i in, cv::Vec2i out)
{
    // Создаем матрицу с исходным изображением
    cv::Mat1b src_mat = cv::Mat1b(verTotal, horTotal, CV_8UC1);
    memcpy(src_mat.data, src.data(), src.size() * sizeof(char));

    dst = src_mat.clone();
    tol = std::max(0, std::min(100, tol));

    if (tol > 0)
    {
        // Рассчитываем границы гистограммы
        // Рассчитываем гистограмму
        std::vector<int> hist(255, 0);
        for (int r = 0; r < verTotal; ++r)
        {
            for (int c = 0; c < horTotal; ++c)
            {
                hist[src_mat(r, c)]++;
            }
        }

        std::vector <int> cum = hist;
        for (int i = 1; i < hist.size(); ++i)
        {
            cum[i] = cum[i - 1] + hist[i];
        }

        int total = src_mat.rows * src_mat.cols;
        int low_bound = total * tol / 100;
        int upp_bound = total * (100 - tol) / 100;
        in[0] = std::distance(cum.begin(), std::lower_bound(cum.begin(), cum.end(), low_bound));
        in[1] = std::distance(cum.begin(), std::lower_bound(cum.begin(), cum.end(), upp_bound));
    }

    // Stretching
    float scale = float(out[1] - out[0]) / float(in[1] - in[0]);
    for (int r = 0; r < dst.rows; ++r)
    {
        for (int c = 0; c < dst.cols; ++c)
        {
            int vs = std::max(src_mat(r, c) - in[0], 0);
            int vd = std::min(int(vs * scale + 0.5f) + out[0], out[1]);
            dst(r, c) = cv::saturate_cast<uchar>(vd);
        }
    }
}
