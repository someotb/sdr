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
    double SCALE = 1e4;
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
    int N = span * sps - 1;
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
