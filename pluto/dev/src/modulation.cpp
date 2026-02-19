#include "modulation.hpp"
#include <algorithm>
#include <cmath>
#include <complex.h>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <stdexcept>
#include <vector>

// According to 3GPP TS 38.211 section 5.1.3:
void modulate(const std::vector<int16_t>& bits, std::vector<std::complex<double>>& symbols, ModulationType modulation_type) {
    switch (modulation_type) {
        case ModulationType::BPSK:
            {
                symbols.resize(bits.size());
                for (size_t i = 0; i < bits.size(); ++i) {
                    double real = (1.0 - 2.0 * bits[i]) / sqrt(2.0);
                    double imag = (1.0 - 2.0 * bits[i]) / sqrt(2.0);
                    symbols[i] = std::complex<double>(real, imag);
                }
            }
            break;

        case ModulationType::QPSK:
            {
                symbols.resize(bits.size() / 2);
                for (size_t i = 0; i < bits.size(); i += 2) {
                    double real = (1.0 - 2.0 * bits[i]) / sqrt(2.0);
                    double imag = (1.0 - 2.0 * bits[i + 1]) / sqrt(2.0);
                    symbols[i / 2] = std::complex<double>(real, imag);
                }
            }
            break;

        case ModulationType::QAM16:
            {
                symbols.resize(bits.size() / 4);
                for (size_t i = 0; i < bits.size(); i += 4) {
                    double real = (1 - 2 * bits[i]) * (2 - (1 - 2 * bits[i + 2])) / sqrt(10);
                    double imag = (1 - 2 * bits[i + 1]) * (2 - (1 - 2 * bits[i + 3])) / sqrt(10);
                    symbols[i / 4] = std::complex<double>(real, imag);
                }
            }
            break;
        default:
            throw std::invalid_argument("Unsupported modulation type");
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

void filter_int16_t(std::vector<int16_t>& symbols_ups, const std::vector<int16_t>& impulse, std::vector<int16_t>& output) {
    size_t n = symbols_ups.size() / 2;
    size_t L = impulse.size();
    fill(output.begin(), output.end(), 0);
    for (size_t i = 0; i < n; i++) {
        size_t max_j = std::min(L, i + 1);
        for (size_t j = 0; j < max_j; j++) {
            output[i * 2] += impulse[j] * symbols_ups[(i - j) * 2];
            output[i * 2 + 1] += impulse[j] * symbols_ups[(i - j) * 2 + 1];
        }
    }
}

void norm(std::vector<double>& rx) {
    double max_val = 0.0;
    for (double e : rx) max_val = std::max(max_val, std::abs(e));
    if (max_val == 0.0) return;
    double scale = 1.0 / max_val;
    for (double& e : rx) e *= scale;
}

void symbols_sync(const std::vector<double>& rx_buffer_after_convolve, std::vector<int>& offset, double& BnTs, double& Kp, int& Nsp) {
    std::vector<double> real_pa(rx_buffer_after_convolve.size() / 2);
    std::vector<double> imag_pa(rx_buffer_after_convolve.size() / 2);

    double zeta = sqrt(2.0)/2.0;
    int tmp_offset = 0;
    double p2 = 0.0;

    for (size_t i = 0; i < rx_buffer_after_convolve.size() / 2; ++i) {
        real_pa[i] = rx_buffer_after_convolve[2 * i];
        imag_pa[i] = rx_buffer_after_convolve[2 * i + 1];
    }

    double teta = (BnTs) / (zeta + 1.0 / (4.0 * zeta));
    double K1 = (4 * zeta * teta) / ((1 + 2 * zeta * teta + teta * teta) * Kp);
    double K2 = (4 * teta * teta) / ((1 + 2 * zeta * teta + teta * teta) * Kp);

    for (size_t ns = 0; ns < real_pa.size(); ns += Nsp) {
        int n = tmp_offset;

        if (ns + n + Nsp >= real_pa.size()) break;

        double real_part = (real_pa[ns + n] - real_pa[Nsp + ns + n]) * real_pa[n + static_cast<int>(Nsp / 2) + ns];
        double imag_part = (imag_pa[ns + n] - imag_pa[Nsp + ns + n]) * imag_pa[n + static_cast<int>(Nsp / 2) + ns];
        double error = real_part + imag_part;

        double p1 = error * K1;
        p2 += p1 + error * K2;

        while(p2 > 1.0) p2 -= 1.0;
        while(p2 < 0.0) p2 += 1.0;

        tmp_offset = static_cast<int>(p2 * Nsp);

        offset[ns / Nsp] = tmp_offset;
    }
}
