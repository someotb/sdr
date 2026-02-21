#include "modulation.hpp"
#include <algorithm>
#include <cmath>
#include <complex.h>
#include <complex>
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

void filter_double(std::vector<double>& symbols_ups, const std::vector<double>& impulse, std::vector<double>& output) {
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
    for (double& e : rx) e /= 32768.0;
}

void norm_after_conv(std::vector<double>& rx, int sps) {
    for (double& e : rx) e /= sps;
}

void GardnerState::gardner_step(
    std::vector<double>& real_pa,
    std::vector<double>& imag_pa,
    std::vector<int>& offset,
    double& BnTs,
    int& Nsp)
{
    double zeta = sqrt(2.0)/2.0;

    double teta = BnTs / (zeta + 1.0/(4.0*zeta));
    double K1 = (4*zeta*teta)/(1+2*zeta*teta+teta*teta);
    double K2 = (4*teta*teta)/(1+2*zeta*teta+teta*teta);

    size_t out_index = 0;

    for (size_t n = Nsp; n + Nsp < real_pa.size(); ++n)
    {
        if (mu >= 1.0)
        {
            mu -= 1.0;

            int idx = n;

            double early_r = real_pa[idx - Nsp];
            double late_r  = real_pa[idx];
            double mid_r   = real_pa[idx - Nsp/2];

            double early_i = imag_pa[idx - Nsp];
            double late_i  = imag_pa[idx];
            double mid_i   = imag_pa[idx - Nsp/2];

            double error =
                (early_r - late_r)*mid_r +
                (early_i - late_i)*mid_i;

            integrator += K2 * error;
            double control = integrator + K1 * error;

            mu += control;

            offset[out_index++] = (int)(mu * Nsp);
        }
        mu += 1.0 / omega;
    }
}

void CostasState::costas_step(double& I_orig, double& Q_orig, double& I_new, double& Q_new, double& Kp, double& Ki, ModulationType& mod_type) {
    double cos_p = cos(phase);
    double sin_p = sin(phase);
    double error = 0.0;

    double I_rot = I_orig * cos_p + Q_orig * sin_p;
    double Q_rot = -I_orig * sin_p + Q_orig * cos_p;

    if (mod_type == ModulationType::BPSK) {
        error = (I_rot >= 0 ? 1 : -1) * Q_rot;
    } else if (mod_type == ModulationType::QPSK) {
        error = (I_rot >= 0 ? 1 : -1) * Q_rot - (Q_rot >= 0 ? 1 : -1) * I_rot;
    } else {
        double I_d = QAM16slicer(I_rot);
        double Q_d = QAM16slicer(Q_rot);
        error = I_d * Q_rot - Q_d * I_rot;
    }

    freq += Ki * error;
    phase += freq + Kp * error;

    if (phase > M_PI) phase -= 2*M_PI;
    if (phase < -M_PI) phase += 2*M_PI;

    I_new = I_rot;
    Q_new = Q_rot;
}

double CostasState::get_phase() {
    return phase;
}
double CostasState::get_freq() {
    return freq;
}

double CostasState::QAM16slicer(double x) {
    if (x < -2.0) return -3.0;
    else if (x < 0.0) return -1.0;
    else if (x < 2.0) return 1.0;
    else return 3.0;
}

void CostasState::reset_costas_state() {
    phase = 0.0;
    freq = 0.0;
}
