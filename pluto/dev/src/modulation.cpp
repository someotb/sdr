#include "modulation.hpp"
#include <algorithm>
#include <cmath>
#include <complex.h>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <numeric>
#include <stdexcept>
#include <vector>

// According to 3GPP TS 38.211 section 5.1.3:
void modulate(const vector<int16_t>& bits, vector<complex<double>>& symbols, ModulationType modulation_type) {
    switch (modulation_type) {
        case ModulationType::BPSK:
            {
                symbols.resize(bits.size());
                for (size_t i = 0; i < bits.size(); ++i) {
                    double real = (1.0 - 2.0 * bits[i]) / sqrt(2.0);
                    double imag = (1.0 - 2.0 * bits[i]) / sqrt(2.0);
                    symbols[i] = complex<double>(real, imag);
                }
            }
            break;

        case ModulationType::QPSK:
            {
                symbols.resize(bits.size() / 2);
                for (size_t i = 0; i < bits.size(); i += 2) {
                    double real = (1.0 - 2.0 * bits[i]) / sqrt(2.0);
                    double imag = (1.0 - 2.0 * bits[i+1]) / sqrt(2.0);
                    symbols[i / 2] = complex<double>(real, imag);
                }
            }
            break;

        case ModulationType::QAM16:
            {
                symbols.resize(bits.size() / 4);
                for (size_t i = 0; i < bits.size(); i += 4) {
                    double real = (1 - 2 * bits[i]) * (2 - (1 - 2 * bits[i + 2])) / sqrt(10);
                    double imag = (1 - 2 * bits[i + 1]) * (2 - (1 - 2 * bits[i + 3])) / sqrt(10);
                    symbols[i / 4] = complex<double>(real, imag);
                }
            }
            break;
        default:
            throw invalid_argument("Unsupported modulation type");
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

void UpSampler(const vector<complex<double>>& symbols, vector<complex<double>>& symbols_ups, int L) {
    const complex<double> i0 (0, 0);
    size_t len_symbols = symbols.size();
    symbols_ups.assign(len_symbols * L, i0);
    for (size_t i = 0; i < len_symbols; i++) {
        symbols_ups[i * L] = symbols[i];
    }
}

void filter(vector<complex<double>>& symbols_ups, const vector<complex<double>>& impulse) {
    size_t n = symbols_ups.size();
    size_t L = impulse.size();
    vector<complex<double>> output(n, 0.0);

    for (size_t i = 0; i < n; i++) {
        size_t max_j = min(L, i + 1);
        for (size_t j = 0; j < max_j; j++) {
            output[i] += impulse[j] * symbols_ups[i - j];
        }
    }

    symbols_ups.swap(output);
}

void filter_int16_t(vector<int16_t>& symbols_ups, const vector<int16_t>& impulse, vector<int16_t>& output) {
    size_t n = symbols_ups.size() / 2;
    size_t L = impulse.size();
    fill(output.begin(), output.end(), 0);
    for (size_t i = 0; i < n; i++) {
        size_t max_j = min(L, i + 1);
        for (size_t j = 0; j < max_j; j++) {
            output[i * 2] += impulse[j] * symbols_ups[(i - j) * 2];
            output[i * 2 + 1] += impulse[j] * symbols_ups[(i - j) * 2 + 1];
        }
    }
}

void norm(std::vector<double>& rx) {
    double scale = 1.0 / 32768.0;
    for (size_t i = 0; i < rx.size(); ++i) rx[i] *= scale;
}

void symbols_sync(const vector<double>& rx_buffer_after_convolve, vector<int>& offset, double& BnTs, double& Kp) {
    vector<double> impulse(10, 1);
    vector<double> real_pa(rx_buffer_after_convolve.size() / 2);
    vector<double> imag_pa(rx_buffer_after_convolve.size() / 2);

    double zeta = sqrt(2.0)/2.0;
    int Nsp = 10;
    int tmp_offset = 0;
    double p2 = 0.0;

    for (size_t i = 0; i < rx_buffer_after_convolve.size() / 2; ++i) {
        real_pa[i] = rx_buffer_after_convolve[2 * i];
        imag_pa[i] = rx_buffer_after_convolve[2 * i + 1];
    }

    double teta = (BnTs / Nsp) / (zeta + 1.0 / (4.0 * zeta));
    double K1 = (4 * zeta * teta) / ((1 + 2 * zeta * teta + teta * teta) * Kp);
    double K2 = (4 * teta * teta) / ((1 + 2 * zeta * teta + teta * teta) * Kp);

    for (size_t ns = 0; ns < rx_buffer_after_convolve.size() / 2; ns += 10) {
        int n = tmp_offset;

        if (ns + n + Nsp >= real_pa.size()) break;

        double real_part = (real_pa[ns + n] - real_pa[Nsp + ns + n]) * real_pa[n + static_cast<int>(Nsp / 2) + ns];
        double imag_part = (imag_pa[ns + n] - imag_pa[Nsp + ns + n]) * imag_pa[n + static_cast<int>(Nsp / 2) + ns];
        double error = real_part + imag_part;

        double p1 = error * K1;
        p2 += p1 + error * K2;

        while(p2 > 1.0) p2 -= 1.0;
        while(p2 < 0.0) p2 += 1.0;

        tmp_offset = static_cast<int>(floor(p2 * Nsp));

        offset[ns / 10] = tmp_offset;
    }
}
