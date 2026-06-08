#include "modulation.hpp"
#include "common.hpp"

#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fftw3.h>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef M_PIf
#define M_PIf (float)M_PI
#endif

int bits_per_symbol(ModulationType mod)
{
    switch (mod)
    {
    case ModulationType::BPSK:
        return 1;
    case ModulationType::QPSK:
        return 2;
    case ModulationType::QAM16:
        return 4;
    case ModulationType::QAM64:
        return 6;
    case ModulationType::QAM256:
        return 8;
    default:
        throw std::runtime_error("[BPS] unsupported modulation type");
    }
}

std::complex<float> map_symbol(std::vector<uint8_t> &in, ModulationType mod, int &offset)
{
    if ((size_t)offset > in.size() - 8)
        return {0.0f, 0.0f};

    switch (mod)
    {
    case ModulationType::BPSK:
    {
        float v = 1.0f - 2.0f * in[offset]; offset++;
        return std::complex<float>(v, v) / std::sqrt(2.0f);
    }
    case ModulationType::QPSK:
    {
        float real = 1.0f - 2.0f * in[offset]; offset++;
        float imag = 1.0f - 2.0f * in[offset]; offset++;
        return std::complex<float>(real, imag) / std::sqrt(2.0f);
    }
    case ModulationType::QAM16:
    {
        int b0 = in[offset]; offset++;
        int b1 = in[offset]; offset++;
        int b2 = in[offset]; offset++;
        int b3 = in[offset]; offset++;
        float real = (1.0f - 2.0f * b0) * (2.0f - (1.0f - 2.0f * b2));
        float imag = (1.0f - 2.0f * b1) * (2.0f - (1.0f - 2.0f * b3));
        return std::complex<float>(real, imag) / std::sqrt(10.0f);
    }
    case ModulationType::QAM64:
    {
        int b0 = in[offset]; offset++;
        int b1 = in[offset]; offset++;
        int b2 = in[offset]; offset++;
        int b3 = in[offset]; offset++;
        int b4 = in[offset]; offset++;
        int b5 = in[offset]; offset++;
        float real = (1.0f - 2.0f * b0) * (4.0f - (1.0f - 2.0f * b2) * (2.0f - (1.0f - 2.0f * b4)));
        float imag = (1.0f - 2.0f * b1) * (4.0f - (1.0f - 2.0f * b3) * (2.0f - (1.0f - 2.0f * b5)));
        return std::complex<float>(real, imag) / std::sqrt(42.0f);
    }
    case ModulationType::QAM256:
    {
        int b0 = in[offset]; offset++;
        int b1 = in[offset]; offset++;
        int b2 = in[offset]; offset++;
        int b3 = in[offset]; offset++;
        int b4 = in[offset]; offset++;
        int b5 = in[offset]; offset++;
        int b6 = in[offset]; offset++;
        int b7 = in[offset]; offset++;

        float real = (1.0f - 2.0f * b0) * (8.0f - (1.0f - 2.0f * b2) * (4.0f - (1.0f - 2.0f * b4) * (2.0f - (1.0f - 2.0f * b6))));

        float imag = (1.0f - 2.0f * b1) * (8.0f - (1.0f - 2.0f * b3) * (4.0f - (1.0f - 2.0f * b5) * (2.0f - (1.0f - 2.0f * b7))));

        return std::complex<float>(real, imag) / std::sqrt(170.0f);
    }

    default:
        throw std::runtime_error("Unsupported mod");
    }
}

void demap_symbols(const std::vector<float> &in, std::vector<float> &out, ModulationType mod)
{
    switch (mod)
    {
    case ModulationType::BPSK:
    {
        out.resize(in.size() / 2);
        float sqrt2f = std::sqrt(2.0f);
        for (size_t i = 0; i < in.size() / 2; ++i)
        {
            float real = in[2 * i] * sqrt2f;
            out[i] = (real < 0.0f) ? 1.0f : 0.0f;
        }
        return;
    }
    case ModulationType::QPSK:
    {
        float sqrt2f = std::sqrt(2.0f);
        for (size_t i = 0; i < in.size() / 2; ++i)
        {
            float real = in[2 * i] * sqrt2f;
            float imag = in[2 * i + 1] * sqrt2f;
            float b0 = (real < 0.0f) ? 1.0f : 0.0f;
            float b1 = (imag < 0.0f) ? 1.0f : 0.0f;
            out[2 * i] = b0;
            out[2 * i + 1] = b1;
        }
        return;
    }
    case ModulationType::QAM16:
    {
        out.resize(in.size() * 2);
        float sqrt10f = std::sqrt(10.0f);
        for (size_t i = 0; i < in.size() / 2; ++i)
        {
            float real = in[2 * i] * sqrt10f;
            float imag = in[2 * i + 1] * sqrt10f;
            float b0 = (real < 0.0f) ? 1.0f : 0.0f;
            float b1 = (imag < 0.0f) ? 1.0f : 0.0f;
            float b2 = (std::abs(real) > 2.0f) ? 1.0f : 0.0f;
            float b3 = (std::abs(imag) > 2.0f) ? 1.0f : 0.0f;

            out[4 * i] = b0;
            out[4 * i + 1] = b1;
            out[4 * i + 2] = b2;
            out[4 * i + 3] = b3;
        }
        return;
    }
    case ModulationType::QAM64:
    {
        out.resize(in.size() * 3);
        float sqrt42f = std::sqrt(42.0f);

        for (size_t i = 0; i < in.size() / 2; ++i)
        {
            float real = in[2 * i] * sqrt42f;
            float imag = in[2 * i + 1] * sqrt42f;

            out[6 * i] = (real < 0.0f) ? 1.0f : 0.0f;
            out[6 * i + 1] = (imag < 0.0f) ? 1.0f : 0.0f;

            float abs_real = std::abs(real);
            float abs_imag = std::abs(imag);
            out[6 * i + 2] = (abs_real < 4.0f) ? 0.0f : 1.0f;
            out[6 * i + 3] = (abs_imag < 4.0f) ? 0.0f : 1.0f;

            out[6 * i + 4] = (std::abs(abs_real - 4.0f) < 2.0f) ? 0.0f : 1.0f;
            out[6 * i + 5] = (std::abs(abs_imag - 4.0f) < 2.0f) ? 0.0f : 1.0f;
        }
        return;
    }
    case ModulationType::QAM256:
    {
        out.resize(in.size() * 4);
        float sqrt170f = std::sqrt(170.0f);

        for (size_t i = 0; i < in.size() / 2; ++i)
        {
            float real = in[2 * i] * sqrt170f;
            float imag = in[2 * i + 1] * sqrt170f;

            out[8 * i + 0] = (real < 0.0f) ? 1.0f : 0.0f;
            float abs_r = std::abs(real);

            out[8 * i + 2] = (abs_r < 8.0f) ? 0.0f : 1.0f;
            float level_r_2 = std::abs(abs_r - 8.0f);

            out[8 * i + 4] = (level_r_2 < 4.0f) ? 0.0f : 1.0f;
            float level_r_3 = std::abs(level_r_2 - 4.0f);

            out[8 * i + 6] = (level_r_3 < 2.0f) ? 0.0f : 1.0f;

            out[8 * i + 1] = (imag < 0.0f) ? 1.0f : 0.0f;
            float abs_q = std::abs(imag);

            out[8 * i + 3] = (abs_q < 8.0f) ? 0.0f : 1.0f;
            float level_q_2 = std::abs(abs_q - 8.0f);

            out[8 * i + 5] = (level_q_2 < 4.0f) ? 0.0f : 1.0f;
            float level_q_3 = std::abs(level_q_2 - 4.0f);

            out[8 * i + 7] = (level_q_3 < 2.0f) ? 0.0f : 1.0f;
        }
        return;
    }

    default:
        throw std::runtime_error("[DEMAPPING] unsupported modulation type");
    }
}

void build_pss_zadoff_chu(FFT_Context &context, sharedData &sd)
{
    int N_zc = sd.subcarrier - 1;
    int cf = N_zc % 2;
    int q = 1;

    for (int i = 0; i < context.N; ++i)
    {
        context.in[i][0] = 0.0f;
        context.in[i][1] = 0.0f;
    }

    for (int n = 0; n < N_zc; ++n)
    {
        float phase = -(M_PIf * (float)sd.zadoff_chu_u * (float)n * (float)(n + cf + 2 * q)) / (float)N_zc;
        float sin, cos;
        sin = sinf(phase);
        cos = cosf(phase);
        context.in[n][0] = cos;
        context.in[n][1] = sin;
    }

    context.ifft();
}

void gen_bits(std::vector<uint8_t> &out, const sharedData &sd)
{
    srand(time(NULL));

    out.clear();
    out.reserve(sd.bits_cnt);
    for (int i = 0; i < sd.bits_cnt; ++i)
        out.push_back(rand() % 2);

    return;
}

void update_bits(std::vector<uint8_t> &out, const sharedData &sd)
{
    srand(time(NULL));
    if (sd.bits_cnt > (int)out.size())
        for (int i = (int)out.size(); i < sd.bits_cnt; ++i)
            out.push_back(rand() % 2);
    else
        out.erase(out.begin() + sd.bits_cnt, out.end());

    return;
}

std::vector<int16_t> build_tx_buffer(std::vector<uint8_t> bits, const std::vector<int16_t> &zc_sec, const sharedData &sd, FFT_Context &context)
{
    std::vector<int16_t> tx;
    tx.reserve(sd.buffer);

    for (const auto &zc : zc_sec)
        tx.push_back(zc);

    int cp = sd.cyclic_prefex;
    int subcarr = sd.subcarrier;
    int cnt_ofdm_sym = sd.buffer / ((subcarr + cp) * 2);
    int ofdm_sym_len = (subcarr + cp) * 2;
    int bit_offset = 0;
    int SCALE = 16000;
    for (int i = 1; i < cnt_ofdm_sym; ++i)
    {
        build_ofdm_symbol(bits, context, sd, bit_offset);

        // CP
        for (int j = 0; j < cp; ++j)
        {
            tx[(i * ofdm_sym_len) + (2 * j)] = context.out[j][0] * SCALE;
            tx[(i * ofdm_sym_len) + (2 * j + 1)] = context.out[j][1] * SCALE;
        }

        for (int j = 0; j < subcarr; ++j)
        {
            tx[(i * ofdm_sym_len) + (2 * j) + (2 * cp)] = context.out[j][0] * SCALE;
            tx[(i * ofdm_sym_len) + (2 * j + 1) + (2 * cp)] = context.out[j][1] * SCALE;
        }
    }
    return tx;
}

void build_ofdm_symbol(std::vector<uint8_t> &in, FFT_Context &context, const sharedData &sd, int &offset)
{
    context.N = sd.subcarrier;
    for (int k = 0; k < context.N; ++k)
    {
        if (sd.is_zeros[k])
        {
            context.in[k][0] = 0;
            context.in[k][1] = 0;
        }
        else if (sd.is_pilot[k])
        {
            context.in[k][0] = 1.0f / std::sqrt(2.0f);
            context.in[k][1] = 0.0f / std::sqrt(2.0f);
        }
        else
        {
            auto s = map_symbol(in, sd.modul_type_TX, offset);
            context.in[k][0] = s.real();
            context.in[k][1] = s.imag();
        }
    }

    context.ifft();
}

void add_cp(FFT_Context &context, std::vector<int16_t> &tx, int cyclic_prefex, int start)
{
    float SCALE = 16000.f;
    int N = context.N;

    for (int j = 0; j < cyclic_prefex; ++j)
    {
        int src_idx = N - cyclic_prefex + j;
        tx[start + 2 * j] = static_cast<int16_t>(context.out[src_idx][0] * SCALE);
        tx[start + 2 * j + 1] = static_cast<int16_t>(context.out[src_idx][1] * SCALE);
    }

    int symbol_offset = start + 2 * cyclic_prefex;
    for (int k = 0; k < N; ++k)
    {
        tx[symbol_offset + 2 * k] = static_cast<int16_t>(context.out[k][0] * SCALE);
        tx[symbol_offset + 2 * k + 1] = static_cast<int16_t>(context.out[k][1] * SCALE);
    }
}

void spectrum(std::vector<std::complex<float>> &in_signal, std::vector<float> &shifted_magnitude, std::vector<float> &argument, FFT_Context &context)
{
    std::vector<float> magnitude(in_signal.size(), 0);

    for (size_t i = 0; i < in_signal.size(); ++i)
    {
        context.in[i][0] = std::real(in_signal[i]) / 32768.0;
        context.in[i][1] = std::imag(in_signal[i]) / 32768.0;
    }

    context.fft();

    for (size_t i = 0; i < in_signal.size(); ++i)
    {
        float real = context.out[i][0];
        float imag = context.out[i][1];
        magnitude[i] = 20.0 * log10(sqrt(real * real + imag * imag) / in_signal.size());
        argument[i] = atan2(imag, real);
    }

    for (size_t i = 0; i < in_signal.size() / 2; ++i)
    {
        shifted_magnitude[i] = magnitude[i + in_signal.size() / 2];
        shifted_magnitude[i + in_signal.size() / 2] = magnitude[i];
    }
}

int zadoff_sync(const float *signal_re, const float *signal_im, size_t signal_len, const float *zc_re,
                const float *zc_im, size_t zc_len, float *out_corr)
{
    float max_norm = -1.f;
    int best_idx = 0;

    for (size_t n = 0; n < signal_len - zc_len; ++n)
    {
        float sum_re = 0.0f;
        float sum_im = 0.0f;

        for (size_t k = 0; k < zc_len; ++k)
        {
            sum_re += signal_re[n + k] * zc_re[k] + signal_im[n + k] * zc_im[k];
            sum_im += signal_im[n + k] * zc_re[k] - signal_re[n + k] * zc_im[k];
        }

        float cur_norm = sum_re * sum_re + sum_im * sum_im;

        if (out_corr)
            out_corr[n] = cur_norm;

        if (cur_norm > max_norm)
        {
            max_norm = cur_norm;
            best_idx = (int)n;
        }
    }
    return best_idx;
}

void remove_pss(sharedData &sd, std::vector<std::complex<float>> &in_signal, std::vector<std::complex<float>> &out_signal)
{
    int pss_symbol = sd.subcarrier;
    out_signal.clear();

    if (in_signal.size() <= static_cast<size_t>(pss_symbol))
        return;
    out_signal.reserve(in_signal.size() - pss_symbol);

    int start_idx = sd.sync_pos + pss_symbol;

    if (start_idx >= (int)in_signal.size())
        return;

    int rem_samples = in_signal.size() - start_idx;
    int cnt_samples = rem_samples / pss_symbol;
    int end_idx = std::min(start_idx + (cnt_samples * pss_symbol), (int)in_signal.size());

    for (int i = start_idx; i < end_idx; ++i)
        out_signal.push_back(in_signal[i]);
}

std::vector<std::complex<float>> cfo_est(const std::vector<std::complex<float>> &signal, sharedData &sd)
{
    int N = sd.subcarrier;
    int CP = sd.cyclic_prefex;
    int sym_len = N + CP;
    float fs = sd.sample_rate;

    int n_symbols = signal.size() / sym_len;
    if (n_symbols < 1)
        return {};

    std::complex<float> corr_sum = 0;
    for (int sym = 0; sym < n_symbols; sym++)
    {
        int sym_start = sym * sym_len;
        std::complex<float> corr = 0;
        for (int k = 0; k < CP; k++)
            corr += conj(signal[sym_start + k]) * signal[sym_start + k + N];
        corr_sum += corr;
    }

    float epsilon = std::arg(corr_sum) / (2 * M_PI);
    float delta_f = epsilon * fs / N; // На сколько символ отклоняется от эталона по фазе в Гц

    if ((int)sd.cfo_offset.size() > sd.mtu)
    {
        sd.cfo_offset.erase(sd.cfo_offset.begin());
        sd.cfo_offset.push_back(delta_f);
    }
    else
        sd.cfo_offset.push_back(delta_f);

    std::vector<std::complex<float>> corrected(signal.size());
    for (size_t n = 0; n < signal.size(); n++)
    {
        corrected[n] = signal[n] * std::polar(1.0f, sd.current_phase);
        sd.current_phase -= 2 * M_PI * delta_f / fs;
        if (sd.current_phase < -M_PI) sd.current_phase += 2 * M_PI;
        if (sd.current_phase > M_PI)  sd.current_phase -= 2 * M_PI;
    }

    return corrected;
}

void remove_cp(std::vector<std::complex<float>> &in_signal, sharedData &sd, std::vector<std::complex<float>> &out_signal)
{
    out_signal.clear();
    size_t cnt_ofdm_symbols = in_signal.size() / (sd.cyclic_prefex + sd.subcarrier);
    out_signal.reserve(in_signal.size() - sd.cyclic_prefex * cnt_ofdm_symbols);

    for (size_t i = 0; i < cnt_ofdm_symbols; ++i)
        for (int j = 0; j < sd.subcarrier; ++j)
            out_signal.push_back(in_signal[j + (i * sd.subcarrier) + sd.cyclic_prefex + (i * sd.cyclic_prefex)]);
}

void decode(std::vector<std::complex<float>> &in_signal, std::vector<std::complex<float>> &out_signal, FFT_Context &context)
{
    out_signal.clear();
    out_signal.reserve(in_signal.size());

    for (size_t i = 0; i < in_signal.size() / context.N; ++i)
    {
        for (int j = 0; j < context.N; ++j)
        {
            context.in[j][0] = std::real(in_signal[j + (i * context.N)]);
            context.in[j][1] = std::imag(in_signal[j + (i * context.N)]);
        }

        context.fft();

        for (int k = 0; k < context.N; ++k)
            out_signal.push_back(std::complex<float>(context.out[k][0], context.out[k][1]));
    }
}

void equalization(std::vector<std::complex<float>> &in_signal, sharedData &sd, std::vector<std::complex<float>> &out_signal)
{
    out_signal.clear();
    out_signal.reserve(in_signal.size());

    int subcarrar = sd.subcarrier;
    int num_symbols = in_signal.size() / subcarrar;

    for (int i = 0; i < num_symbols; ++i)
    {
        std::vector<std::complex<float>> H(subcarrar, {0.0f, 0.0f});
        std::vector<std::complex<float>> equalized(subcarrar, {0.0f, 0.0f});
        const std::complex<float> known_pilot_conj(1.0f / std::sqrt(2.0f), 0.0f / std::sqrt(2.0f));

        for (int p_idx : sd.pilot_idxs)
        {
            std::complex<float> received_pilot = in_signal[i * subcarrar + p_idx];
            H[p_idx] = received_pilot * known_pilot_conj;
        }

        for (size_t p = 0; p + 1 < sd.pilot_idxs.size(); ++p)
        {
            int k1 = sd.pilot_idxs[p];
            int k2 = sd.pilot_idxs[p + 1];

            std::complex<float> H1 = H[k1];
            std::complex<float> H2 = H[k2];

            float phi1 = std::arg(H1);
            float phi2 = std::arg(H2);

            float delta_phi = phi2 - phi1;

            if (delta_phi > M_PIf)
                delta_phi -= 2 * M_PI;

            if (delta_phi < -M_PIf)
                delta_phi += 2 * M_PI;

            for (int k = k1 + 1; k < k2; ++k)
            {
                if (sd.is_zeros[k])
                    continue;

                float alpha = float(k - k1) / float(k2 - k1);
                H[k] = std::polar(std::abs(H1) + alpha * (std::abs(H2) - std::abs(H1)), phi1 + alpha * delta_phi);
            }
        }

        for (int k = 0; k < sd.pilot_idxs.front(); ++k)
            H[k] = H[sd.pilot_idxs.front()];

        for (int k = sd.pilot_idxs.back() + 1; k < subcarrar; ++k)
            H[k] = H[sd.pilot_idxs.back()];

        {
            std::lock_guard<std::mutex> lock(sd.sync.channel_estimation_mutex);
            if (sd.channel_estimation.size() >= (size_t)sd.mtu)
                sd.channel_estimation.erase(sd.channel_estimation.begin(),
                                                  sd.channel_estimation.begin() + sd.subcarrier);
            for (const auto& h : H)
                sd.channel_estimation.push_back(h);
        }

        for (int k = 0; k < subcarrar; ++k)
        {
            if (sd.is_zeros[k] || std::abs(H[k]) < 1e-6f)
                equalized[k] = {0, 0};
            else
                equalized[k] = in_signal[i * subcarrar + k] / H[k];
        }

        for (size_t k = 0; k < equalized.size(); ++k)
        {
            if (sd.is_zeros[k] || sd.is_pilot[k])
                continue;

            out_signal.push_back(equalized[k]);
        }
    }
}

void split_to_float(const std::complex<float> *src, float *dst_re, float *dst_im, size_t n)
{
    const float *raw_src = reinterpret_cast<const float *>(src);

    for (size_t i = 0; i < n; ++i)
    {
        dst_re[i] = raw_src[2 * i];
        dst_im[i] = raw_src[2 * i + 1];
    }
}

void split_int16_t_to_float(const int16_t *src, float *dst_re, float *dst_im, size_t num_samples)
{
    for (size_t i = 0; i < num_samples; ++i)
    {
        dst_re[i] = static_cast<float>(src[2 * i]) / 32768.0f;
        dst_im[i] = static_cast<float>(src[2 * i + 1]) / 32768.0f;
    }
}

std::vector<uint8_t> str_to_bits(const std::string &in)
{
    std::vector<uint8_t> bits;
    bits.reserve(in.size() * 8);

    for (uint8_t c : in)
        for (int i = 7; i >= 0; --i)
            bits.push_back((c >> i) & 1);

    return bits;
}

std::string bits_to_str(const std::vector<float> &bits)
{
    std::string message;
    for (size_t i = 0; i + 7 < bits.size(); i += 8)
    {
        uint8_t c = 0;
        for (int j = 0; j < 8; ++j)
            if ((uint8_t)bits[i + j])
                c |= (1 << (7 - j));
        message += (char)c;
    }
    return message;
}

void calculate_pilots(sharedData &sd)
{
    int cnt_pilot = sd.cnt_pilots;
    if (cnt_pilot <= 0)
        throw std::invalid_argument("Count of pilots can not be <= 0");
    if (cnt_pilot > sd.subcarrier)
        throw std::invalid_argument("Too many pilots");

    std::fill(sd.is_pilot.begin(), sd.is_pilot.end(), false);
    sd.pilot_idxs.clear();

    std::vector<int> active;
    for (size_t i = 0; i < sd.is_zeros.size(); ++i)
        if (!sd.is_zeros[i])
            active.push_back(i);

    size_t n = active.size();

    for (int p = 0; p < cnt_pilot; ++p)
    {
        size_t idx = (p * n) / cnt_pilot;
        sd.is_pilot[active[idx]] = true;
    }

    for (size_t i = 0; i < active.size(); ++i)
    {
        bool is_start = (i == 0) || (active[i] - active[i-1] > 1);
        bool is_end = (i == n-1) || (active[i+1] - active[i] > 1);

        if (is_start || is_end)
            sd.is_pilot[active[i]] = true;
    }

    for (size_t i = 0; i < sd.is_pilot.size(); ++i)
        if (sd.is_pilot[i])
            sd.pilot_idxs.push_back(i);
}
