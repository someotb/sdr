#include "modulation.hpp"
#include <algorithm>
#include <cmath>
#include <complex.h>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fftw3.h>
#include <pstl/glue_algorithm_defs.h>
#include <stdexcept>
#include <vector>
#include <algorithm>
#include <iostream>

// According to 3GPP TS 38.211 section 5.1.3:
std::complex<double> map_symbol(std::deque<int> &fifo, ModulationType mod)
{
    switch (mod)
    {
    case ModulationType::BPSK:
    {
        int b = fifo.front();
        fifo.pop_front();
        double v = (1.0 - 2.0 * b);
        return {v, 0.0};
    }

    case ModulationType::QPSK:
    {
        int b0 = fifo.front();
        fifo.pop_front();
        int b1 = fifo.front();
        fifo.pop_front();

        double real = (1.0 - 2.0 * b0);
        double imag = (1.0 - 2.0 * b1);

        return std::complex<double>(real, imag) / std::sqrt(2.0);
    }

    case ModulationType::QAM16:
    {
        int b0 = fifo.front();
        fifo.pop_front();
        int b1 = fifo.front();
        fifo.pop_front();
        int b2 = fifo.front();
        fifo.pop_front();
        int b3 = fifo.front();
        fifo.pop_front();

        double real = (1 - 2 * b0) * (2 - (1 - 2 * b2));
        double imag = (1 - 2 * b1) * (2 - (1 - 2 * b3));

        return std::complex<double>(real, imag) / std::sqrt(10.0);
    }

    default:
        throw std::runtime_error("unsupported modulation type");
    }
}

int bits_per_symbol(ModulationType type)
{
    switch (type)
    {
    case ModulationType::BPSK:
        return 1;
    case ModulationType::QPSK:
        return 2;
    case ModulationType::QAM16:
        return 4;
    default:
        throw std::invalid_argument("Unsapported modulation type");
    }
}

void fft(fftw_complex *in, fftw_complex *out, int N)
{
    fftw_plan plan = fftw_plan_dft_1d(N, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
    fftw_execute(plan);
    fftw_destroy_plan(plan);
}

void ifft(fftw_complex *in, fftw_complex *out, int N)
{
    fftw_plan plan = fftw_plan_dft_1d(N, in, out, FFTW_BACKWARD, FFTW_ESTIMATE);
    fftw_execute(plan);
    for (int i = 0; i < N; ++i)
    {
        out[i][0] /= N;
        out[i][1] /= N;
    }
    fftw_destroy_plan(plan);
}

void build_pss_zadoff_chu(fftw_complex *in, fftw_complex *out, int subcarrier, int u)
{
    if (subcarrier <= 0)
        throw std::invalid_argument("subcarrier must be positive");

    for (int i = 0; i < subcarrier; ++i)
    {
        in[i][0] = 0.0;
        in[i][1] = 0.0;
    }

    int N_zc = subcarrier / 2;

    for (int n = 0; n < N_zc; ++n)
    {
        double phase = -M_PI * u * n * (n + 1) / N_zc;
        int freq_idx = 2 * n + 1;
        in[freq_idx][0] = cos(phase);
        in[freq_idx][1] = sin(phase);
    }

    ifft(in, out, subcarrier);
}

void build_ofdm_symbol(std::deque<int> &bit_fifo, fftw_complex *in, fftw_complex *out, ModulationType mod, int subcarrier)
{
    size_t needed = subcarrier * bits_per_symbol(mod);
    std::deque<int> pilots = {1, -1, 1, 1, -1, 1, -1, -1};

    while (bit_fifo.size() < needed)
        bit_fifo.push_back(rand() & 1);

    for (int k = 0; k < subcarrier; ++k)
    {
        if (k > subcarrier / 2 - 28 && k < subcarrier / 2 + 27)
        {
            in[k][0] = 0;
            in[k][1] = 0;
        }
        else if (k == 5 || k == 15 || k == 25 || k == 35 || k == 92 || k == 102 || k == 112 || k == 122)
        {
            // auto s = map_symbol(pilots, ModulationType::BPSK);
            in[k][0] = 1.0;
            in[k][1] = 0.0;
        }
        else if (k == 0)
        {
            in[k][0] = 0;
            in[k][1] = 0;
        }
        else
        {
            auto s = map_symbol(bit_fifo, mod);
            in[k][0] = s.real();
            in[k][1] = s.imag();
        }
    }

    ifft(in, out, subcarrier);
}

void append_symbol(fftw_complex *out, std::vector<int16_t> &tx, int subcarrier, int cyclic_prefex, int start)
{
    std::vector<int16_t> tmp((subcarrier + cyclic_prefex) * 2, 0);
    double SCALE = 16000;
    // cylic prefex
    int i = subcarrier - cyclic_prefex;
    for (int j = 0; j < cyclic_prefex; ++j)
    {
        if (i < subcarrier)
        {
            tmp[2 * j] = out[i][0] * SCALE;
            tmp[2 * j + 1] = out[i][1] * SCALE;
            ++i;
        }
    }

    // symbol
    for (int k = cyclic_prefex; k < (subcarrier + cyclic_prefex); ++k)
    {
        tmp[2 * k] = out[k - cyclic_prefex][0] * SCALE;
        tmp[2 * k + 1] = out[k - cyclic_prefex][1] * SCALE;
    }

    for (int l = 0; l < subcarrier + cyclic_prefex; ++l)
    {
        tx[start + (2 * l)] = tmp[2 * l];
        tx[start + (2 * l + 1)] = tmp[2 * l + 1];
    }
}

void spectrum(std::vector<std::complex<double>> &in_signal, std::vector<double> &shifted_magnitude, std::vector<double> &argument)
{
    fftw_complex *in_spectre = static_cast<fftw_complex *>(fftw_malloc(sizeof(fftw_complex) * in_signal.size()));
    fftw_complex *out_spectre = static_cast<fftw_complex *>(fftw_malloc(sizeof(fftw_complex) * in_signal.size()));
    std::vector<double> magnitude(in_signal.size(), 0);

    for (size_t i = 0; i < in_signal.size(); ++i)
    {
        in_spectre[i][0] = std::real(in_signal[i]) / 32768.0;
        in_spectre[i][1] = std::imag(in_signal[i]) / 32768.0;
    }

    fft(in_spectre, out_spectre, in_signal.size());

    for (size_t i = 0; i < in_signal.size(); ++i)
    {
        double real = out_spectre[i][0];
        double imag = out_spectre[i][1];
        magnitude[i] = 20.0 * log10(sqrt(real * real + imag * imag) / in_signal.size());
        argument[i] = atan2(imag, real);
    }

    for (size_t i = 0; i < in_signal.size() / 2; ++i)
    {
        shifted_magnitude[i] = magnitude[i + in_signal.size() / 2];
        shifted_magnitude[i + in_signal.size() / 2] = magnitude[i];
    }

    fftw_free(in_spectre);
    fftw_free(out_spectre);
}

zadoff_chu_state zadoff_sync(std::vector<std::complex<double>> &signal, std::vector<int16_t> &zadoff_chu_seq)
{
    if (signal.size() == 0)
        return zadoff_chu_state{};

    std::vector<std::complex<double>> zadoff_chu_compl;
    for (size_t i = 0; i < zadoff_chu_seq.size() / 2; ++i)
        zadoff_chu_compl.push_back(std::complex<double>(static_cast<double>(zadoff_chu_seq[2 * i]), static_cast<double>(zadoff_chu_seq[2 * i + 1])));

    zadoff_chu_state zadoff_c;
    double max_norm = 0.0;
    int best_idx = 0;

    for (size_t n = 0; n < signal.size() - zadoff_chu_compl.size(); ++n)
    {
        std::complex<double> sum = {0.0, 0.0};
        for (size_t k = 0; k < zadoff_chu_compl.size(); ++k)
            sum += signal[n + k] * std::conj(zadoff_chu_compl[k]);

        double cur_norm = std::norm(sum);
        zadoff_c.index_arr.push_back(cur_norm);
        if (cur_norm > max_norm)
        {
            max_norm = cur_norm;
            best_idx = n;
        }
        if (zadoff_c.index_arr.size() >= 1920)
            zadoff_c.index_arr.clear();
    }

    zadoff_c.index = best_idx;
    return zadoff_c;
}

void remove_pss(std::vector<std::complex<double>> &in_signal, int cp, int subcarrar, int pos, std::vector<std::complex<double>> &out_signal)
{
    out_signal.clear();
    out_signal.reserve(in_signal.size() - subcarrar - cp);

    if (pos == 0)
    {
        for (size_t i = subcarrar; i < in_signal.size(); ++i)
            out_signal.push_back(in_signal[i]);
        return;
    }

    for (size_t i = pos + subcarrar; i < in_signal.size(); ++i)
        out_signal.push_back(in_signal[i]);
}

void remove_cp(std::vector<std::complex<double>> &in_signal, int cp, int subcarrar, std::vector<std::complex<double>> &out_signal)
{
    out_signal.clear();
    size_t cnt_ofdm_symbols = in_signal.size() / (cp + subcarrar);
    out_signal.reserve(in_signal.size() - cp * cnt_ofdm_symbols);

    for (size_t i = 0; i < cnt_ofdm_symbols; ++i)
    {
        for (int j = 0; j < subcarrar; ++j)
        {
            out_signal.push_back(in_signal[j + (i * subcarrar) + cp + (i * cp)]);
        }
    }
}

void decode(std::vector<std::complex<double>> &in_signal, int subcarrar, std::vector<std::complex<double>> &out_signal)
{
    out_signal.clear();
    out_signal.reserve(in_signal.size());
    fftw_complex *in_fft = static_cast<fftw_complex *>(fftw_malloc(sizeof(fftw_complex) * subcarrar));
    fftw_complex *out_fft = static_cast<fftw_complex *>(fftw_malloc(sizeof(fftw_complex) * subcarrar));

    for (size_t i = 0; i < in_signal.size() / subcarrar; ++i)
    {
        for (int j = 0; j < subcarrar; ++j)
        {
            in_fft[j][0] = std::real(in_signal[j + (i * subcarrar)]);
            in_fft[j][1] = std::imag(in_signal[j + (i * subcarrar)]);
        }

        fft(in_fft, out_fft, subcarrar);

        for (int k = 0; k < subcarrar; ++k)
        {
            out_signal.push_back(std::complex(out_fft[k][0], out_fft[k][1]));
        }
    }
    fftw_free(in_fft);
    fftw_free(out_fft);
}

void equalization(std::vector<std::complex<double>> &in_signal, int subcarrar)
{
    int num_symbols = in_signal.size() / subcarrar;
    std::vector<std::complex<double>> pilot_vals;
    std::deque<int> pilots = {1, -1, 1, 1, -1, 1, -1, -1};
    std::vector<int> pilot_idxs = {5, 15, 25, 35, 92, 102, 112, 122};

    for (size_t i = 0; i < pilots.size(); ++i)
        pilot_vals.push_back({1.0, 0.0});

    for (int i = 0; i < num_symbols; ++i)
    {
        double symbols_phase = 0;
        int offset = i * subcarrar;
        for (size_t j = 0; j < pilot_idxs.size(); ++j)
        {
            int pilot_idx = pilot_idxs[j];
            std::complex<double> rx_pilot = in_signal[offset + pilot_idx];
            symbols_phase += std::arg(rx_pilot * std::conj(pilot_vals[j]));
        }

        double avg_phase = symbols_phase / pilot_idxs.size();

        for (int k = 0; k < subcarrar; ++k)
            in_signal[offset + k] *= std::polar(1.0, -avg_phase);
    }
}
