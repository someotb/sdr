#include "modulation.hpp"
#include "fftlib.hpp"

#include <algorithm>
#include <cmath>
#include <complex.h>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fftw3.h>
#include <pstl/glue_algorithm_defs.h>
#include <stdexcept>
#include <vector>
#include <algorithm>
#include <iostream>

std::complex<float> map_symbol(const std::vector<int> &bits, size_t &offset, ModulationType mod)
{
    switch (mod)
    {
    case ModulationType::BPSK:
    {
        int b = bits[offset]; ++offset;
        float v = (1.0f - 2.0f * b); 
        return std::complex<float>(v, v) / std::sqrt(2.0f);
    }

    case ModulationType::QPSK:
    {
        int b0 = bits[offset % bits.size()]; ++offset;
        int b1 = bits[offset % bits.size()]; ++offset;
        float real = (1.0 - 2.0 * b0);
        float imag = (1.0 - 2.0 * b1);

        return std::complex<float>(real, imag) / std::sqrt(2.0f);
    }

    case ModulationType::QAM16:
    {
        int b0 = bits[offset % bits.size()]; ++offset;
        int b1 = bits[offset % bits.size()]; ++offset;
        int b2 = bits[offset % bits.size()]; ++offset;
        int b3 = bits[offset % bits.size()]; ++offset;
        float real = (1 - 2 * b0) * (2 - (1 - 2 * b2));
        float imag = (1 - 2 * b1) * (2 - (1 - 2 * b3));

        return std::complex<float>(real, imag) / std::sqrt(10.0f);
    }

    default:
        throw std::runtime_error("unsupported modulation type");
    }
}

void build_pss_zadoff_chu(FFT_Context &context, int u)
{
    const int N = context.N;
    const int N_zc = N / 2;
    const float inv_Nzc = 1.0f / N_zc;
    const float pi_u = M_PIf32 * (float)u;

    #pragma omp simd
    for (int i = 0; i < N; ++i)
    {
        context.in[i][0] = 0.0f;
        context.in[i][1] = 0.0f;
    }

    #pragma omp simd
    for (int n = 0; n < N_zc; ++n)
    {
        float phase = -pi_u * n * (n + 1) * inv_Nzc;
        float s, c;
        sincosf(phase, &s, &c);
        context.in[2 * n + 1][0] = c;
        context.in[2 * n + 1][1] = s;
    }
    ifft(context);
}

void build_ofdm_symbol(const std::vector<int> &bits, size_t &offset, FFT_Context &context, const sharedData *sh_data)
{
    std::vector<int> pilot_idxs = {4, 12, 20, 28, 100, 108, 116, 124};
    std::vector<bool> is_pilot(sh_data->subcarrier, false);

    for (auto &x : pilot_idxs)
        is_pilot[x] = true;

    for (int k = 0; k < context.N; ++k)
    {
        if (k > context.N / 2 - 28 && k < context.N / 2 + 27)
        {
            context.in[k][0] = 0;
            context.in[k][1] = 0;
        }
        else if (is_pilot[k])
        {
            context.in[k][0] = 1.0;
            context.in[k][1] = 0.0;
        }
        else if (k == 0)
        {
            context.in[k][0] = 0;
            context.in[k][1] = 0;
        }
        else
        {
            auto s = map_symbol(bits, offset, sh_data->modul_type_TX);
            context.in[k][0] = s.real();
            context.in[k][1] = s.imag();
        }
    }

    ifft(context);
}

void build_ofdm_symbol_no_ifft(const std::vector<int> &bits, size_t &bit_offset, std::vector<float> &bits_mapped, const sharedData *sh_data)
{
    std::vector<int> pilot_idxs = {4, 12, 20, 28, 100, 108, 116, 124};
    std::vector<bool> is_pilot(sh_data->subcarrier, false);
    for (auto &x : pilot_idxs) is_pilot[x] = true;

    int sub = sh_data->subcarrier;
    int num_symbols = sh_data->buffer / (sub * 2); 
    
    bits_mapped.clear();
    bits_mapped.reserve(num_symbols * sub * 2);

    for (int i = 0; i < num_symbols; ++i)
    {
        for (int k = 0; k < sub; ++k)
        {
            if (k > sub / 2 - 28 && k < sub / 2 + 27) continue;
            if (is_pilot[k]) continue;
            if (k == 0) continue;

            auto s = map_symbol(bits, bit_offset, sh_data->modul_type_TX);
            
            bits_mapped.push_back(s.real());
            bits_mapped.push_back(s.imag());
        }
    }
}


void append_symbol(FFT_Context &context, std::vector<int16_t> &tx, int cyclic_prefex, int start)
{
    float SCALE = 16000.f;
    int N = context.N;

    #pragma omp simd
    for (int j = 0; j < cyclic_prefex; ++j)
    {
        int src_idx = N - cyclic_prefex + j;
        tx[start + 2 * j] = static_cast<int16_t>(context.out[src_idx][0] * SCALE);
        tx[start + 2 * j + 1] = static_cast<int16_t>(context.out[src_idx][1] * SCALE);
    }

    int symbol_offset = start + 2 * cyclic_prefex;
    #pragma omp simd
    for (int k = 0; k < N; ++k)
    {
        tx[symbol_offset + 2 * k] = static_cast<int16_t>(context.out[k][0] * SCALE);
        tx[symbol_offset + 2 * k + 1] = static_cast<int16_t>(context.out[k][1] * SCALE);
    }
}

void spectrum(std::vector<std::complex<float>> &in_signal, std::vector<float> &shifted_magnitude, std::vector<float> &argument, FFT_Context &context)
{
    std::vector<float> magnitude(in_signal.size(), 0);

    for (size_t i = 0; i < in_signal.size(); ++i)
    {
        context.in[i][0] = std::real(in_signal[i]) / 32768.0;
        context.in[i][1] = std::imag(in_signal[i]) / 32768.0;
    }

    fft(context);

    for (size_t i = 0; i < in_signal.size(); ++i)
    {
        float real = context.out[i][0];
        float imag = context.out[i][1];
        magnitude[i] = 20.0 * log10(sqrt(real * real + imag * imag) / in_signal.size());
        argument[i] = atan2(imag, real);
    }

    for (size_t i = 0; i < in_signal.size() / 2; ++i)
    {
        shifted_magnitude[i] = magnitude[i + in_signal.size() / 2];
        shifted_magnitude[i + in_signal.size() / 2] = magnitude[i];
    }
}

int zadoff_sync(const float *__restrict signal_re, const float *__restrict signal_im, size_t signal_len, const float *__restrict zc_re, const float *__restrict zc_im, size_t zc_len, float *__restrict out_corr)
{
    float max_norm = -1.f;
    int best_idx = 0;

    for (size_t n = 0; n < signal_len - zc_len; ++n)
    {
        float sum_re = 0.0f;
        float sum_im = 0.0f;

        #pragma omp simd reduction(+ : sum_re, sum_im)
        for (size_t k = 0; k < zc_len; ++k)
        {
            sum_re += signal_re[n + k] * zc_re[k] + signal_im[n + k] * zc_im[k];
            sum_im += signal_im[n + k] * zc_re[k] - signal_re[n + k] * zc_im[k];
        }

        float cur_norm = sum_re * sum_re + sum_im * sum_im;

        if (out_corr)
            out_corr[n] = cur_norm;

        if (cur_norm > max_norm)
        {
            max_norm = cur_norm;
            best_idx = (int)n;
        }
    }
    return best_idx;
}

void remove_pss(sharedData *sh_data, std::vector<std::complex<float>> &out_signal)
{
    int subcarrar = sh_data->subcarrier;
    int cp = sh_data->cyclic_prefex;
    out_signal.clear();
    out_signal.reserve(sh_data->rx_complex.size() - subcarrar - cp);

    if (sh_data->sync_pos == 0)
    {
        for (size_t i = subcarrar; i < sh_data->rx_complex.size(); ++i)
            out_signal.push_back(sh_data->rx_complex[i]);
        return;
    }

    for (size_t i = sh_data->sync_pos + subcarrar; i < sh_data->rx_complex.size(); ++i)
        out_signal.push_back(sh_data->rx_complex[i]);
}

void cfo_correction(std::vector<std::complex<float>> &in_signal, sharedData *sh_data)
{
    sh_data->cfo_offset.clear();
    std::complex<float> corr = 0.0;
    int subcarrar = sh_data->subcarrier;
    int cp = sh_data->cyclic_prefex;
    int ofdm_symbol = subcarrar + cp;
    int cnt_ofdm_symbols = in_signal.size() / ofdm_symbol;
    int sample_rate = 1920000;

    for (int n = 0; n < cnt_ofdm_symbols; ++n)
    {
        int start = n * ofdm_symbol;
        for (int i = 0; i < cp; ++i) 
            corr += conj(in_signal[i + start]) * in_signal[i + start + subcarrar];

        float eps = arg(corr) / (2 * M_PI);
        float delta_f = eps * sample_rate / subcarrar;
        
        for (int i = 0; i < ofdm_symbol; ++i)
        {
            float phase = -2 * M_PIf * delta_f * i / sample_rate;
            sh_data->cfo_offset.push_back(phase);
            in_signal[start + i] *= std::complex<float>(std::cos(phase), std::sin(phase));
        }
    }
}

void remove_cp(std::vector<std::complex<float>> &in_signal, sharedData *sh_data, std::vector<std::complex<float>> &out_signal)
{
    out_signal.clear();
    size_t cnt_ofdm_symbols = in_signal.size() / (sh_data->cyclic_prefex + sh_data->subcarrier);
    out_signal.reserve(in_signal.size() - sh_data->cyclic_prefex * cnt_ofdm_symbols);

    for (size_t i = 0; i < cnt_ofdm_symbols; ++i)
        for (int j = 0; j < sh_data->subcarrier; ++j)
            out_signal.push_back(in_signal[j + (i * sh_data->subcarrier) + sh_data->cyclic_prefex + (i * sh_data->cyclic_prefex)]);
}

void decode(std::vector<std::complex<float>> &in_signal, std::vector<std::complex<float>> &out_signal, FFT_Context &context)
{
    out_signal.clear();
    out_signal.reserve(in_signal.size());

    for (size_t i = 0; i < in_signal.size() / context.N; ++i)
    {
        for (int j = 0; j < context.N; ++j)
        {
            context.in[j][0] = std::real(in_signal[j + (i * context.N)]);
            context.in[j][1] = std::imag(in_signal[j + (i * context.N)]);
        }

        fft(context);

        for (int k = 0; k < context.N; ++k)
            out_signal.push_back(std::complex<float>(context.out[k][0], context.out[k][1]));
    }
}

void equalization(std::vector<std::complex<float>> &in_signal, int subcarrar, std::vector<std::complex<float>> &out_signal)
{
    out_signal.clear();
    out_signal.reserve(in_signal.size());
    int num_symbols = in_signal.size() / subcarrar;
    std::vector<int> pilot_idxs = {4, 12, 20, 28, 100, 108, 116, 124};
    std::vector<bool> is_pilot(subcarrar, false);

    for (auto &x : pilot_idxs)
        is_pilot[x] = true;

    for (int i = 0; i < num_symbols; ++i)
    {
        std::vector<std::complex<float>> H(subcarrar, {0.0, 0.0});
        std::vector<std::complex<float>> equalized(subcarrar, {0.0, 0.0});

        for (size_t p = 0; p < pilot_idxs.size(); ++p)
            H[pilot_idxs[p]] = in_signal[i * subcarrar + pilot_idxs[p]] / std::complex<float>(1, 0);

        for (size_t p = 0; p + 1 < pilot_idxs.size(); ++p)
        {
            int k1 = pilot_idxs[p];
            int k2 = pilot_idxs[p + 1];

            std::complex<float> H1 = H[k1];
            std::complex<float> H2 = H[k2];

            for (int k = k1 + 1; k < k2; ++k)
            {
                if (k > subcarrar / 2 - 28 && k < subcarrar / 2 + 27)
                    continue;

                float alpha = float(k - k1) / float(k2 - k1);
                H[k] = H1 + alpha * (H2 - H1);
            }
        }

        for (int k = 1; k < pilot_idxs.front(); ++k)
        {
            if (k > subcarrar / 2 - 28 && k < subcarrar / 2 + 27)
                continue;

            H[k] = H[pilot_idxs.front()];
        }

        for (int k = pilot_idxs.back() + 1; k < subcarrar; ++k)
        {
            if (k > subcarrar / 2 - 28 && k < subcarrar / 2 + 27)
                continue;

            H[k] = H[pilot_idxs.back()];
        }

        for (int k = 1; k < subcarrar; ++k)
        {
            if (k > subcarrar / 2 - 28 && k < subcarrar / 2 + 27)
                continue;

            equalized[k] = in_signal[i * subcarrar + k] / H[k];
        }

        float phase = 0;
        int pilot_cnt = 0;

        for (size_t k = 0; k < pilot_idxs.size(); ++k)
        {
            phase += std::arg(equalized[pilot_idxs[k]]);
            pilot_cnt++;
        }

        phase /= pilot_cnt;

        for (int k = 1; k < subcarrar; ++k)
        {
            if (k > subcarrar / 2 - 28 && k < subcarrar / 2 + 27)
                continue;

            equalized[k] *= std::exp(std::complex<float>(0.0, -phase));
        }

        for (int k = 1; k < subcarrar; ++k)
        {
            if (k > subcarrar / 2 - 28 && k < subcarrar / 2 + 27)
                continue;

            if (is_pilot[k])
                continue;

            out_signal.push_back(equalized[k]);
        }
    }
}

void split_to_float(const std::complex<float> *__restrict src, float *__restrict dst_re, float *__restrict dst_im, size_t n)
{
    const float *raw_src = reinterpret_cast<const float *>(src);

    #pragma omp simd
    for (size_t i = 0; i < n; ++i)
    {
        dst_re[i] = raw_src[2 * i];
        dst_im[i] = raw_src[2 * i + 1];
    }
}

void split_int16_t_to_float(const int16_t *src, float *dst_re, float *dst_im, size_t num_samples)
{
    #pragma omp simd
    for (size_t i = 0; i < num_samples; ++i)
    {
        dst_re[i] = static_cast<float>(src[2 * i]) / 32768.0f;
        dst_im[i] = static_cast<float>(src[2 * i + 1]) / 32768.0f;
    }
}

void check_bits(std::vector<float> &in_signal, std::vector<float> &bits, sharedData *sh_data)
{
    float eps = 0.6f;
    for (size_t i = 0; i < in_signal.size(); ++i)
    {
        if (std::abs(in_signal[i] - bits[i]) > eps)
            sh_data->err_cnt++;
    }
}
