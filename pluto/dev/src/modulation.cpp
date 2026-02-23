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

void filter_double(std::vector<double>& in, std::vector<double>& h, std::vector<double>& out) {
    size_t n = in.size() / 2;
    size_t L = h.size();
    fill(out.begin(), out.end(), 0);
    for (size_t i = 0; i < n; i++) {
        size_t max_j = std::min(L, i + 1);
        for (size_t j = 0; j < max_j; j++) {
            out[i * 2] += h[j] * in[(i - j) * 2];
            out[i * 2 + 1] += h[j] * in[(i - j) * 2 + 1];
        }
    }
}

void norm_max(std::vector<double>& rx) {
    auto it = std::max_element(rx.begin(), rx.end());
    auto max_elemet = *it;
    for (auto& e : rx) e /= max_elemet;
}

std::vector<double> rrc(int& sps, int& span, double& alpha) {
    int N = span * sps + 1;
    std::vector<double> h(N);

    for (int n = 0; n < N; ++n) {
        double t = (n - N / 2.0) / static_cast<double>(sps);

        if (fabs(t) < 1e-8) {
            h[n] = 1 - alpha + (4 * alpha / M_PI);
        } else if (fabs(fabs(t) - 1.0 / (4 * alpha)) < 1e-8) {
            h[n] = alpha / std::sqrt(2.0) * ((1 + 2 / M_PI) * std::sin(M_PI / (4 * alpha)) + (1 - 2 / M_PI) * cos(M_PI / (4 * alpha)));
        } else {
            double numerator = sin(M_PI * t * (1 - alpha)) + 4 * alpha * t * cos(M_PI * t * (1 + alpha));
            double denominator = M_PI * t * (1 - (4 * alpha * t) * (4 * alpha * t));
            h[n] = numerator / denominator;
        }
    }
    return h;
}


void GardnerState::gather(std::vector<double>& real_p, std::vector<double>& imag_p, std::vector<std::complex<double>>& gather) {
    for (size_t i = 0; i < real_p.size(); ++i) {
        gather[i] = std::complex(real_p[i], imag_p[i]);
    }
}

std::vector<std::complex<double>> GardnerState::gardnerr(std::vector<std::complex<double>>& input, double& BnTs, int& SPS, double& Kp) {
    size_t N = input.size();
    size_t M = N / SPS;

    std::vector<std::complex<double>> output(M);

    double zeta = std::sqrt(2.0) / 2.0;
    double teta = (BnTs / 10.0) / (zeta + 1.0 / (4.0 * zeta));
    double K1 = (-4.0 * zeta * teta) / ((1.0 + 2.0 * zeta * teta + teta * teta) * (Kp + 1e-15));
    double K2 = (-4.0 * teta * teta) / ((1.0 + 2.0 * zeta * teta + teta * teta) * (Kp + 1e-15));

    double p2 = 0.0;
    int offset = 0;

    for (size_t i = 0; i < M; ++i) {
        size_t base = SPS * i;

        size_t idx0 = base + offset;
        size_t idx1 = base + offset + SPS;
        size_t idxm = base + offset + SPS / 2;

        if (idx1 >= N || idxm >= N)
            break;

        std::complex<double> s1 = input[idx1];
        std::complex<double> s0 = input[idx0];
        std::complex<double> sm = input[idxm];

        double e = (std::real(s1) - std::real(s0)) * std::real(sm) + (std::imag(s1) - std::imag(s0)) * std::imag(sm);

        double p1 = e * K1;
        p2 += p1 + e * K2;
        p2 -= std::floor(p2);

        int new_offset = (int)std::round(p2 * SPS);

        offset = new_offset;

        size_t read_idx = SPS * i + offset;
        output[i] = input[read_idx];
    }

    return output;
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
