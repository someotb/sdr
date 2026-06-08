#pragma once

#include "common.hpp"

#include <cmath>
#include <complex>
#include <cstdint>
#include <vector>

int bits_per_symbol(ModulationType mod);

std::complex<float> map_symbol(std::vector<uint8_t> &in, ModulationType mod, int &offset);

void demap_symbols(const std::vector<float> &in, std::vector<float> &out, ModulationType mod);

void build_pss_zadoff_chu(FFT_Context &context, sharedData &sd);

void gen_bits(std::vector<uint8_t> &out, const sharedData &sd);

void update_bits(std::vector<uint8_t> &out, const sharedData &sd);

std::vector<int16_t> build_tx_buffer(std::vector<uint8_t> bits, const std::vector<int16_t> &zc_sec, const sharedData &sd, FFT_Context &context);

void build_ofdm_symbol(std::vector<uint8_t> &in, FFT_Context &context, const sharedData &sd, int &offset);

void add_cp(FFT_Context &context, std::vector<int16_t> &tx, int cyclic_prefex, int start);

void spectrum(std::vector<std::complex<float>> &in_signal, std::vector<float> &shifted_magnitude, std::vector<float> &argument, FFT_Context &context);

int zadoff_sync(const float *signal_re, const float *signal_im, size_t signal_len, const float *zc_re,
                const float *zc_im, size_t zc_len, float *out_corr);

void remove_pss(sharedData &sd, std::vector<std::complex<float>> &in_signal, std::vector<std::complex<float>> &out_signal);

std::vector<std::complex<float>> cfo_est(const std::vector<std::complex<float>> &signal, sharedData &sd);

void remove_cp(std::vector<std::complex<float>> &signal, sharedData &sd, std::vector<std::complex<float>> &signal_fft);

void decode(std::vector<std::complex<float>> &in_signal, std::vector<std::complex<float>> &out_signal, FFT_Context &context);

void equalization(std::vector<std::complex<float>> &in_signal, sharedData &sd, std::vector<std::complex<float>> &out_signal);

void split_to_float(const std::complex<float> *src, float *dst_re, float *dst_im, size_t n);

void split_int16_t_to_float(const int16_t *src, float *dst_re, float *dst_im, size_t num_samples);

std::vector<uint8_t> str_to_bits(const std::string &in);

std::string bits_to_str(const std::vector<float> &bits);

void calculate_pilots(sharedData &sd);
