#pragma once

#include "sharedData.hpp"
#include "fftlib.hpp"
#include "types.hpp"

#include <random>
#include <vector>
#include <complex>
#include <cstdint>
#include <cmath>

#include "prbs15.hpp"

int bits_per_symbol(ModulationType mod);

std::complex<float> map_symbol(const std::vector<int> &bits, size_t &offset, ModulationType mod);

void demap_symbols(std::vector<float> &in, std::vector<float> &out, ModulationType mod);

void build_pss_zadoff_chu(FFT_Context &context, sharedData *sh_data);

void build_ofdm_symbol(PRBS15 &gen, FFT_Context &context, const sharedData *sh_data);

void append_symbol(FFT_Context &context, std::vector<int16_t> &tx, int cyclic_prefex, int start);

void spectrum(std::vector<std::complex<float>> &in_signal, std::vector<float> &shifted_magnitude, std::vector<float> &argument, FFT_Context &context);

int zadoff_sync(const float *__restrict signal_re, const float *__restrict signal_im, size_t signal_len, const float *__restrict zc_re, const float *__restrict zc_im, size_t zc_len, float* __restrict out_corr);

void remove_pss(sharedData *sh_data, std::vector<std::complex<float>> &out_signal);

void cfo_correction(std::vector<std::complex<float>> &in_signal, sharedData *sh_data);

void remove_cp(std::vector<std::complex<float>> &signal, sharedData *sh_data, std::vector<std::complex<float>> &signal_fft);

void decode(std::vector<std::complex<float>> &in_signal, std::vector<std::complex<float>> &out_signal, FFT_Context &context);

void equalization(std::vector<std::complex<float>> &in_signal, const sharedData *sh_data, std::vector<std::complex<float>> &out_signal);

void split_to_float(const std::complex<float>* __restrict src, float* __restrict dst_re, float* __restrict dst_im, size_t n);

void split_int16_t_to_float(const int16_t *src, float *dst_re, float *dst_im, size_t num_samples);

void get_bits_to_check(sharedData *sh_data);

void check_bits(std::vector<float> &in_signal, sharedData *sh_data);
