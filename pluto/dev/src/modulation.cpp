#include "modulation.hpp"
#include <algorithm>
#include <cmath>
#include <complex.h>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iterator>
#include <stdexcept>
#include <vector>
#include <algorithm>

// According to 3GPP TS 38.211 section 5.1.3:
std::complex<double> map_symbol(std::deque<int>& fifo, ModulationType mod) {
    switch(mod) {
        case ModulationType::BPSK: {
            int b = fifo.front(); fifo.pop_front();
            double v = (1.0 - 2.0*b);
            return {v, 0.0};
        }

        case ModulationType::QPSK: {
            int b0 = fifo.front(); fifo.pop_front();
            int b1 = fifo.front(); fifo.pop_front();

            double real = (1.0 - 2.0*b0);
            double imag = (1.0 - 2.0*b1);

            return std::complex<double>(real, imag) / std::sqrt(2.0);
        }

        case ModulationType::QAM16: {
            int b0=fifo.front(); fifo.pop_front();
            int b1=fifo.front(); fifo.pop_front();
            int b2=fifo.front(); fifo.pop_front();
            int b3=fifo.front(); fifo.pop_front();

            double real = (1 - 2 * b0) * (2 - (1 - 2 * b2));
            double imag = (1 - 2 * b1) * (2 - (1 - 2 * b3));

            return std::complex<double>(real, imag) / std::sqrt(10.0);
        }

        default:
            throw std::runtime_error("unsupported modulation type");
    }
}


void UpSampler(const std::vector<std::complex<double>>& symbols, std::vector<std::complex<double>>& symbols_ups, int L) {
    const std::complex<double> i0 (0, 0);
    size_t len_symbols = symbols.size();
    symbols_ups.assign(len_symbols * L, i0);
    for (size_t i = 0; i < len_symbols; i++) {
        symbols_ups[i * L] = symbols[i];
    }
}

void filter(std::vector<std::complex<double>>& symbols_ups, const std::vector<std::complex<double>>& impulse) {
    size_t n = symbols_ups.size();
    size_t L = impulse.size();
    std::vector<std::complex<double>> output(n, 0.0);

    for (size_t i = 0; i < n; i++) {
        size_t max_j = std::min(L, i + 1);
        for (size_t j = 0; j < max_j; j++) {
            output[i] += impulse[j] * symbols_ups[i - j];
        }
    }

    symbols_ups.swap(output);
}

void fft(fftw_complex* in, fftw_complex* out, int N) {
    fftw_plan plan = fftw_plan_dft_1d(N, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
    fftw_execute(plan);
    fftw_destroy_plan(plan);
}

void ifft(fftw_complex* in, fftw_complex* out, int N) {
    fftw_plan plan = fftw_plan_dft_1d(N, in, out, FFTW_BACKWARD, FFTW_ESTIMATE);
    fftw_execute(plan);
    for (int i = 0; i < N; ++i) {
        out[i][0] /= N;
        out[i][1] /= N;
    }
    fftw_destroy_plan(plan);
}

void build_ofdm_symbol(std::deque<int>& bit_fifo, fftw_complex* in, fftw_complex* out, ModulationType mod, int subcarrier) {
    size_t needed = subcarrier * bits_per_symbol(mod);

    while(bit_fifo.size() < needed)
        bit_fifo.push_back(rand() & 1);

    for(int k = 0; k < subcarrier; ++k) {
        auto s = map_symbol(bit_fifo, mod);

        in[k][0] = s.real();
        in[k][1] = s.imag();
    }

    ifft(in, out, subcarrier);
}

void append_symbol(fftw_complex* out, std::vector<int16_t>& tx, int subcarrier, int cyclic_prefex) {
    double SCALE = 1e3;
    // cylic prefex
    for(int i = subcarrier - cyclic_prefex; i < subcarrier; ++i) {
        tx.push_back(out[i][0] * SCALE);
        tx.push_back(out[i][1] * SCALE);
    }

    // symbol
    for(int i = 0; i < subcarrier; ++i) {
        tx.push_back(out[i][0] * SCALE);
        tx.push_back(out[i][1] * SCALE);
    }
}

int bits_per_symbol(ModulationType type) {
    switch(type) {
        case ModulationType::BPSK: return 1;
        case ModulationType::QPSK: return 2;
        case ModulationType::QAM16: return 4;
        default: throw std::invalid_argument("Unsapported modulation type");
    }
}
