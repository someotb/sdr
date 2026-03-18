#pragma once

#include <vector>
#include <complex>
#include <deque>
#include <cstdint>
#include <fftw3.h>
#include <cmath>

enum class ModulationType
{
    BPSK,
    QPSK,
    QAM16
};

struct FFT_Context
{
    int N;
    fftwf_complex *in;
    fftwf_complex *out;
    fftwf_plan plan_forward;
    fftwf_plan plan_backward;

    FFT_Context(int n) : N(n)
    {
        in = fftwf_alloc_complex(N);
        out = fftwf_alloc_complex(N);

        plan_forward = fftwf_plan_dft_1d(N, in, out, FFTW_FORWARD, FFTW_MEASURE);
        plan_backward = fftwf_plan_dft_1d(N, in, out, FFTW_BACKWARD, FFTW_MEASURE);
    }

    ~FFT_Context()
    {
        fftwf_destroy_plan(plan_forward);
        fftwf_destroy_plan(plan_backward);
        fftwf_free(in);
        fftwf_free(out);
    }
};

/**
 * @brief Modulation mapper according to 3GPP TS 38.211 specification
 *
 * @param bits Input bit sequence
 * @param symbols Output std::complex symbols
 * @param modulation_type Type of modulation to use
 */

std::complex<float> map_symbol(std::deque<int> &fifo, ModulationType mod);

int bits_per_symbol(ModulationType type);

void fft(FFT_Context &context);

void ifft(FFT_Context &context);

void build_pss_zadoff_chu(FFT_Context &context, int u);

void build_ofdm_symbol(std::deque<int> &bit_fifo, FFT_Context &context, ModulationType mod);

void append_symbol(FFT_Context &context, std::vector<int16_t> &tx, int cyclic_prefex, int start);

void spectrum(std::vector<std::complex<float>> &in_signal, std::vector<float> &shifted_magnitude, std::vector<float> &argument, FFT_Context &context);

int zadoff_sync(const float *__restrict signal_re, const float *__restrict signal_im, size_t signal_len, const float *__restrict zc_re, const float *__restrict zc_im, size_t zc_len, float* __restrict out_corr);

void remove_pss(std::vector<std::complex<float>> &in_signal, int cp, int subcarrar, int pos, std::vector<std::complex<float>> &out_signal);

void cfo_correction(std::vector<std::complex<float>> &in_signal, int subcarrar, int cp, std::vector<float> &cfo_offset);

void remove_cp(std::vector<std::complex<float>> &signal, int cp, int subcarrar, std::vector<std::complex<float>> &signal_fft);

void decode(std::vector<std::complex<float>> &in_signal, std::vector<std::complex<float>> &out_signal, FFT_Context &context);

void equalization(std::vector<std::complex<float>> &in_signal, int subcarrar);

void remove_pilots(std::vector<std::complex<float>> &in_signal, int subcarar);

void split_to_float(const std::complex<float>* __restrict src, float* __restrict dst_re, float* __restrict dst_im, size_t n);

void split_int16_t_to_float(const int16_t* __restrict src, float* __restrict dst_re, float* __restrict dst_im, size_t n);