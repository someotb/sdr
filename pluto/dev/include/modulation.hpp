#pragma once

#include "sharedData.hpp"
#include "fftlib.hpp"

#include <vector>
#include <complex>
#include <deque>
#include <cstdint>
#include <cmath>

std::complex<float> map_symbol(const std::vector<int> &bits, size_t &offset, ModulationType mod);

void build_pss_zadoff_chu(FFT_Context &context, int u);

void build_ofdm_symbol(const std::vector<int> &bits, size_t &offset, FFT_Context &context, const sharedData *sh_data);

void append_symbol(FFT_Context &context, std::vector<int16_t> &tx, int cyclic_prefex, int start);

void spectrum(std::vector<std::complex<float>> &in_signal, std::vector<float> &shifted_magnitude, std::vector<float> &argument, FFT_Context &context);

int zadoff_sync(const float *__restrict signal_re, const float *__restrict signal_im, size_t signal_len, const float *__restrict zc_re, const float *__restrict zc_im, size_t zc_len, float* __restrict out_corr);

void remove_pss(std::vector<std::complex<float>> &in_signal, int cp, int subcarrar, int pos, std::vector<std::complex<float>> &out_signal);

void cfo_correction(std::vector<std::complex<float>> &in_signal, int subcarrar, int cp, std::vector<float> &cfo_offset);

void remove_cp(std::vector<std::complex<float>> &signal, int cp, int subcarrar, std::vector<std::complex<float>> &signal_fft);

void decode(std::vector<std::complex<float>> &in_signal, std::vector<std::complex<float>> &out_signal, FFT_Context &context);

void equalization(std::vector<std::complex<float>> &in_signal, int subcarrar, std::vector<std::complex<float>> &out_signal);

void split_to_float(const std::complex<float>* __restrict src, float* __restrict dst_re, float* __restrict dst_im, size_t n);

void split_int16_t_to_float(const int16_t* __restrict src, float* __restrict dst_re, float* __restrict dst_im, size_t n);