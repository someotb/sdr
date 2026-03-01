#pragma once

#include <vector>
#include <complex>
#include <deque>
#include <cstdint>
#include <fftw3.h>
#include <cmath>

enum class ModulationType {
    BPSK,
    QPSK,
    QAM16
};

/**
 * @brief Modulation mapper according to 3GPP TS 38.211 specification
 *
 * @param bits Input bit sequence
 * @param symbols Output std::complex symbols
 * @param modulation_type Type of modulation to use
 */

std::complex<double> map_symbol(std::deque<int>& fifo, ModulationType mod);

 /**
  * @brief Upsample function
  *
  * @param symbols Input std::complex symbols
  * @param symbols Output std::complex symbols_ups
  * @param L Upsample factor
  */

void UpSampler(const std::vector<std::complex<double>>& symbols, std::vector<std::complex<double>>& symbols_ups, int L);

 /**
  * @brief Filter function
  *
  * @param symbols_ups Input std::complex symbols
  * @param impulse Impulse response
  */

void filter(std::vector<std::complex<double>>& symbols_ups, const std::vector<std::complex<double>>& impulse);

 /**
  * @brief Bits function
  *
  * @param type Type of modulation
  */

void fft(fftw_complex* in, fftw_complex* out, int N);

void ifft(fftw_complex* in, fftw_complex* out, int N);

void build_pss_symbol(fftw_complex* in, fftw_complex* out, int subcarrier);

void build_ofdm_symbol(std::deque<int>& bit_fifo, fftw_complex* in, fftw_complex* out, ModulationType mod, int subcarrier);

void append_symbol(fftw_complex* out, std::vector<int16_t>& tx, int subcarrier, int cyclic_prefex);

int bits_per_symbol(ModulationType type);
