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

struct zadoff_chu_state
{
    int index = 0;
    std::vector<double> index_arr;
};

/**
 * @brief Modulation mapper according to 3GPP TS 38.211 specification
 *
 * @param bits Input bit sequence
 * @param symbols Output std::complex symbols
 * @param modulation_type Type of modulation to use
 */

std::complex<double> map_symbol(std::deque<int> &fifo, ModulationType mod);

int bits_per_symbol(ModulationType type);

void fft(fftw_complex *in, fftw_complex *out, int N);

void ifft(fftw_complex *in, fftw_complex *out, int N);

void build_pss_zadoff_chu(fftw_complex *in, fftw_complex *out, int subcarrier, int u);

void build_ofdm_symbol(std::deque<int> &bit_fifo, fftw_complex *in, fftw_complex *out, ModulationType mod, int subcarrier);

void append_symbol(fftw_complex *out, std::vector<int16_t> &tx, int subcarrier, int cyclic_prefex, int start);

void spectrum(std::vector<std::complex<double>> &in_signal, std::vector<double> &shifted_magnitude, std::vector<double> &argument);

zadoff_chu_state zadoff_sync(std::vector<std::complex<double>> &signal, std::vector<int16_t> &zadoff_chu_seq);

void remove_pss(std::vector<std::complex<double>> &in_signal, int cp, int subcarrar, int pos, std::vector<std::complex<double>> &out_signal);

void cfo_correction(std::vector<std::complex<double>> &in_signal, int subcarrar, int cp, std::vector<double> &cfo_offset);

void remove_cp(std::vector<std::complex<double>> &signal, int cp, int subcarrar, std::vector<std::complex<double>> &signal_fft);

void decode(std::vector<std::complex<double>> &in_signal, int subcarrar, std::vector<std::complex<double>> &out_signal);

void equalization(std::vector<std::complex<double>> &in_signal, int subcarrar);
