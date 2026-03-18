#include "modulation.hpp"
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

// According to 3GPP TS 38.211 section 5.1.3:
std::complex<float> map_symbol(std::deque<int> &fifo, ModulationType mod)
{
    switch (mod)
    {
    case ModulationType::BPSK:
    {
        int b = fifo.front();
        fifo.pop_front();
        float v = (1.0 - 2.0 * b);
        return std::complex<float>(v, 0.0);
    }

    case ModulationType::QPSK:
    {
        int b0 = fifo.front();
        fifo.pop_front();
        int b1 = fifo.front();
        fifo.pop_front();

        float real = (1.0 - 2.0 * b0);
        float imag = (1.0 - 2.0 * b1);

        return std::complex<float>(real, imag) / std::sqrt(2.f);
    }

    case ModulationType::QAM16:
    {
        int b0 = fifo.front();
        fifo.pop_front();
        int b1 = fifo.front();
        fifo.pop_front();
        int b2 = fifo.front();
        fifo.pop_front();
        int b3 = fifo.front();
        fifo.pop_front();

        float real = (1 - 2 * b0) * (2 - (1 - 2 * b2));
        float imag = (1 - 2 * b1) * (2 - (1 - 2 * b3));

        return std::complex<float>(real, imag) / std::sqrt(10.f);
    }

    default:
        throw std::runtime_error("unsupported modulation type");
    }
}

int bits_per_symbol(ModulationType type)
{
    switch (type)
    {
    case ModulationType::BPSK:
        return 1;
    case ModulationType::QPSK:
        return 2;
    case ModulationType::QAM16:
        return 4;
    default:
        throw std::invalid_argument("Unsapported modulation type");
    }
}

void fft(FFT_Context &context)
{
    fftwf_execute(context.plan_forward);
}

void ifft(FFT_Context &context)
{
    fftwf_execute(context.plan_backward);

    float scale = 1.0f / context.N;
    #pragma omp simd
    for (int i = 0; i < context.N; ++i)
    {
        context.out[i][0] *= scale;
        context.out[i][1] *= scale;
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

void build_ofdm_symbol(std::deque<int> &bit_fifo, FFT_Context &context, ModulationType mod)
{
    size_t needed = context.N * bits_per_symbol(mod);
    std::deque<int> pilots = {1, -1, 1, 1, -1, 1, -1, -1};

    while (bit_fifo.size() < needed)
        bit_fifo.push_back(rand() & 1);

    for (int k = 0; k < context.N; ++k)
    {
        if (k > context.N / 2 - 28 && k < context.N / 2 + 27)
        {
            context.in[k][0] = 0;
            context.in[k][1] = 0;
        }
        else if (k == 5 || k == 15 || k == 25 || k == 35 || k == 92 || k == 102 || k == 112 || k == 122)
        {
            context.in[k][0] = 2.0;
            context.in[k][1] = 0.0;
        }
        else if (k == 0)
        {
            context.in[k][0] = 0;
            context.in[k][1] = 0;
        }
        else
        {
            auto s = map_symbol(bit_fifo, mod);
            context.in[k][0] = s.real();
            context.in[k][1] = s.imag();
        }
    }

    ifft(context);
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

int zadoff_sync(const float *__restrict signal_re, const float *__restrict signal_im, size_t signal_len, const float *__restrict zc_re, const float *__restrict zc_im, size_t zc_len, float* __restrict out_corr)
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

void remove_pss(std::vector<std::complex<float>> &in_signal, int cp, int subcarrar, int pos, std::vector<std::complex<float>> &out_signal)
{
    out_signal.clear();
    out_signal.reserve(in_signal.size() - subcarrar - cp);

    if (pos == 0)
    {
        for (size_t i = subcarrar; i < in_signal.size(); ++i)
            out_signal.push_back(in_signal[i]);
        return;
    }

    for (size_t i = pos + subcarrar; i < in_signal.size(); ++i)
        out_signal.push_back(in_signal[i]);
}

void cfo_correction(std::vector<std::complex<float>> &in_signal, int subcarrar, int cp, std::vector<float> &cfo_offset)
{
    cfo_offset.clear();
    std::complex<float> corr = 0.0;
    int ofdm_symbol = subcarrar + cp;
    int cnt_ofdm_symbols = in_signal.size() / ofdm_symbol;

    for (size_t n = 0; n < in_signal.size(); n += ofdm_symbol)
    {
        for (size_t i = n; i < n + cp; ++i)
            corr += in_signal[i] * conj(in_signal[i + subcarrar]);

        float eps = arg(corr) / (2 * M_PI);
        float mean_eps = eps / cnt_ofdm_symbols;

        for (size_t i = n; i < n + ofdm_symbol; ++i)
        {
            in_signal[i] *= std::complex<float>(std::cos(mean_eps), std::sin(mean_eps));
            cfo_offset.push_back(mean_eps);
        }
    }
}

void remove_cp(std::vector<std::complex<float>> &in_signal, int cp, int subcarrar, std::vector<std::complex<float>> &out_signal)
{
    out_signal.clear();
    size_t cnt_ofdm_symbols = in_signal.size() / (cp + subcarrar);
    out_signal.reserve(in_signal.size() - cp * cnt_ofdm_symbols);

    for (size_t i = 0; i < cnt_ofdm_symbols; ++i)
    {
        for (int j = 0; j < subcarrar; ++j)
        {
            out_signal.push_back(in_signal[j + (i * subcarrar) + cp + (i * cp)]);
        }
    }
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

void equalization(std::vector<std::complex<float>> &in_signal, int subcarrar)
{
    int num_symbols = in_signal.size() / subcarrar;
    std::vector<std::complex<float>> pilot_vals;
    std::vector<int> pilot_idxs = {5, 15, 25, 35, 92, 102, 112, 122};

    for (size_t i = 0; i < pilot_idxs.size(); ++i)
        pilot_vals.push_back(std::complex<float>(2.0, 0.0));

    for (int i = 0; i < num_symbols; ++i)
    {
        float symbols_phase = 0;
        int offset = i * subcarrar;
        for (size_t j = 0; j < pilot_idxs.size(); ++j)
        {
            int pilot_idx = pilot_idxs[j];
            std::complex<float> rx_pilot = in_signal[offset + pilot_idx];
            symbols_phase += std::arg(rx_pilot * std::conj(pilot_vals[j]));
        }

        float avg_phase = symbols_phase / pilot_idxs.size();

        for (int k = 0; k < subcarrar; ++k)
            in_signal[offset + k] *= std::polar(1.f, -avg_phase);
    }
}

void remove_pilots(std::vector<std::complex<float>> &in_signal, int subcarar)
{
    std::vector<int> pilot_idxs = {5, 15, 25, 35, 92, 102, 112, 122};
    for (size_t i = 0; i < in_signal.size() / subcarar; ++i)
        for (int j = 0; j < subcarar; ++j)
            for (size_t k = 0; k < pilot_idxs.size(); ++k)
                if ((j + i * subcarar) == (pilot_idxs[k] + i * subcarar))
                    in_signal[j + i * subcarar] = {0.0, 0.0};
}

void split_to_float(const std::complex<float>* __restrict src, float* __restrict dst_re, float* __restrict dst_im, size_t n)
{
    const float* raw_src = reinterpret_cast<const float*>(src);

    #pragma omp simd
    for (size_t i = 0; i < n; ++i)
    {
        dst_re[i] = raw_src[2 * i];
        dst_im[i] = raw_src[2 * i + 1];
    }
}

void split_int16_t_to_float(const int16_t* src, float* dst_re, float* dst_im, size_t num_samples) {
    #pragma omp simd
    for (size_t i = 0; i < num_samples; ++i)
    {
        dst_re[i] = static_cast<float>(src[2 * i]) / 32768.0f;
        dst_im[i] = static_cast<float>(src[2 * i + 1]) / 32768.0f;
    }
}

