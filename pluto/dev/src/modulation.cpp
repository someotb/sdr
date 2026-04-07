#include "modulation.hpp"
#include "fftlib.hpp"
#include "sharedData.hpp"
#include "types.hpp"
#include "prbs15.hpp"

#include <cmath>
#include <complex.h>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fftw3.h>
#include <stdexcept>
#include <vector>

int bits_per_symbol(ModulationType mod)
{
    switch (mod)
    {
        case ModulationType::BPSK: return 1;
        case ModulationType::QPSK: return 2;
        case ModulationType::QAM16: return 4;
        case ModulationType::QAM64: return 6;
        case ModulationType::QAM128: return 7;
        default:
            throw std::runtime_error("[BPS] unsupported modulation type");
    }
}

std::complex<float> map_symbol(const std::vector<int> &bits, size_t &offset, ModulationType mod)
{
    switch (mod)
    {
        case ModulationType::BPSK:
        {
            int b = bits[offset]; ++offset;
            float v = (1.0f - 2.0f * b);
            return std::complex<float>(v, v) / std::sqrt(2.0f);
        }

        case ModulationType::QPSK:
        {
            int b0 = bits[offset % bits.size()]; ++offset;
            int b1 = bits[offset % bits.size()]; ++offset;
            float real = (1.0f - 2.0f * b0);
            float imag = (1.0f - 2.0f * b1);

            return std::complex<float>(real, imag) / std::sqrt(2.0f);
        }

        case ModulationType::QAM16:
        {
            int b0 = bits[offset % bits.size()]; ++offset;
            int b1 = bits[offset % bits.size()]; ++offset;
            int b2 = bits[offset % bits.size()]; ++offset;
            int b3 = bits[offset % bits.size()]; ++offset;
            float real = (1.0f - 2.0f * b0) * (2.0f - (1.0f - 2.0f * b2));
            float imag = (1.0f - 2.0f * b1) * (2.0f - (1.0f - 2.0f * b3));

            return std::complex<float>(real, imag) / std::sqrt(10.0f);
        }

        case ModulationType::QAM64:
        {
            int b0 = bits[offset % bits.size()]; ++offset;
            int b1 = bits[offset % bits.size()]; ++offset;
            int b2 = bits[offset % bits.size()]; ++offset;
            int b3 = bits[offset % bits.size()]; ++offset;
            int b4 = bits[offset % bits.size()]; ++offset;
            int b5 = bits[offset % bits.size()]; ++offset;
            float real = (1.0f - 2.0f * b0) * (4.0f - (1.0f - 2.0f * b2) * (2.0f - (1.0f - 2.0f * b4)));
            float imag = (1.0f - 2.0f * b1) * (4.0f - (1.0f - 2.0f * b3) * (2.0f - (1.0f - 2.0f * b5)));

            return std::complex<float>(real, imag) / std::sqrt(42.0f);
        }

        default:
            throw std::runtime_error("[MAPPING] unsupported modulation type");
    }
}

std::complex<float> map_symbol_prbs(PRBS15 &gen, ModulationType mod)
{
    switch (mod)
    {
        case ModulationType::BPSK: {
            float v = 1.0f - 2.0f * gen.get_bit();
            return std::complex<float>(v, v) / std::sqrt(2.0f);
        }
        case ModulationType::QPSK: {
            float real = 1.0f - 2.0f * gen.get_bit();
            float imag = 1.0f - 2.0f * gen.get_bit();
            return std::complex<float>(real, imag) / std::sqrt(2.0f);
        }
        case ModulationType::QAM16: {
            int b0 = gen.get_bit();
            int b1 = gen.get_bit();
            int b2 = gen.get_bit();
            int b3 = gen.get_bit();
            float real = (1.0f - 2.0f * b0) * (2.0f - (1.0f - 2.0f * b2));
            float imag = (1.0f - 2.0f * b1) * (2.0f - (1.0f - 2.0f * b3));
            return std::complex<float>(real, imag) / std::sqrt(10.0f);
        }
        case ModulationType::QAM64: {
            int b0 = gen.get_bit();
            int b1 = gen.get_bit();
            int b2 = gen.get_bit();
            int b3 = gen.get_bit();
            int b4 = gen.get_bit();
            int b5 = gen.get_bit();
            float real = (1.0f - 2.0f * b0) * (4.0f - (1.0f - 2.0f * b2) * (2.0f - (1.0f - 2.0f * b4)));
            float imag = (1.0f - 2.0f * b1) * (4.0f - (1.0f - 2.0f * b3) * (2.0f - (1.0f - 2.0f * b5)));
            return std::complex<float>(real, imag) / std::sqrt(42.0f);
        }
        case ModulationType::QAM128: {
            int b0 = gen.get_bit(); int b1 = gen.get_bit();
            int b2 = gen.get_bit(); int b3 = gen.get_bit();
            int b4 = gen.get_bit(); int b5 = gen.get_bit();
            int b6 = gen.get_bit();

            float real = (1.0f - 2.0f * b0) * (8.0f - (1.0f - 2.0f * b2) * (4.0f - (1.0f - 2.0f * b4) * (2.0f - (1.0f - 2.0f * b6))));
            float imag = (1.0f - 2.0f * b1) * (4.0f - (1.0f - 2.0f * b3) * (2.0f - (1.0f - 2.0f * b5)));

                return std::complex<float>(real, imag) / std::sqrt(82.0f);
        }

        default: throw std::runtime_error("Unsupported mod");
    }
}


void demap_symbols(std::vector<float> &in, std::vector<float> &out, ModulationType mod)
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
        case ModulationType::QAM128:
        {
            out.resize(in.size() * 3.5);
            float sqrt53f = std::sqrt(53.0f);

            for (size_t i = 0; i < in.size() / 2; ++i)
            {
                float real = in[2 * i] * sqrt53f;
                float imag = in[2 * i + 1] * sqrt53f;

                out[7 * i] = (real < 0.0f) ? 1.0f : 0.0f;
                float abs_real = std::abs(real);
                out[7 * i + 2] = (abs_real < 8.0f) ? 0.0f : 1.0f;
                float abs_r_8 = std::abs(abs_real - 8.0f);
                out[7 * i + 4] = (abs_r_8 < 4.0f) ? 0.0f : 1.0f;
                out[7 * i + 6] = (std::abs(abs_r_8 - 4.0f) < 2.0f) ? 0.0f : 1.0f;

                out[7 * i + 1] = (imag < 0.0f) ? 1.0f : 0.0f;
                float abs_imag = std::abs(imag);
                out[7 * i + 3] = (abs_imag < 4.0f) ? 0.0f : 1.0f;
                out[7 * i + 5] = (std::abs(abs_imag - 4.0f) < 2.0f) ? 0.0f : 1.0f;
            }
            return;
        }

    default:
        throw std::runtime_error("[DEMAPPING] unsupported modulation type");
    }
}


void build_pss_zadoff_chu(FFT_Context &context, sharedData &sh_data)
{
    int N_zc = sh_data.subcarrier - 1;
    int cf = N_zc % 2;
    int q = 1;

    for (int i = 0; i < context.N; ++i)
    {
        context.in[i][0] = 0.0f;
        context.in[i][1] = 0.0f;
    }

    for (int n = 0; n < N_zc; ++n)
    {
        float phase = - (M_PIf * (float)sh_data.zadoff_chu_u * (float)n * (float)(n + cf + 2 * q)) / (float)N_zc;
        float sin, cos;
        sincosf(phase, &sin, &cos);
        context.in[n][0] = cos;
        context.in[n][1] = sin;
    }

    ifft(context);
}

void build_ofdm_symbol(PRBS15 &gen, FFT_Context &context, const sharedData &sh_data)
{
    for (int k = 0; k < context.N; ++k)
    {
        if (sh_data.is_zeros[k])
        {
            context.in[k][0] = 0;
            context.in[k][1] = 0;
        }
        else if (sh_data.is_pilot[k])
        {
            context.in[k][0] = 1.0;
            context.in[k][1] = 0.0;
        }
        else
        {
            auto s = map_symbol_prbs(gen, sh_data.modul_type_TX);
            context.in[k][0] = s.real();
            context.in[k][1] = s.imag();
        }
    }

    ifft(context);
}

void append_symbol(FFT_Context &context, std::vector<int16_t> &tx, int cyclic_prefex, int start)
{
    float SCALE = 16000.f;
    int N = context.N;

    #pragma omp simd
    for (int j = 0; j < cyclic_prefex; ++j)
    {
        int src_idx = N - cyclic_prefex + j;
        tx[start + 2 * j] = static_cast<int16_t>(context.out[src_idx][0] * SCALE);
        tx[start + 2 * j + 1] = static_cast<int16_t>(context.out[src_idx][1] * SCALE);
    }

    int symbol_offset = start + 2 * cyclic_prefex;
    #pragma omp simd
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

    fft(context);

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

int zadoff_sync(const float *__restrict signal_re, const float *__restrict signal_im, size_t signal_len, const float *__restrict zc_re, const float *__restrict zc_im, size_t zc_len, float *__restrict out_corr)
{
    float max_norm = -1.f;
    int best_idx = 0;

    for (size_t n = 0; n < signal_len - zc_len; ++n)
    {
        float sum_re = 0.0f;
        float sum_im = 0.0f;

        #pragma omp simd reduction(+ : sum_re, sum_im)
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

void remove_pss(sharedData &sh_data, std::vector<std::complex<float>> &out_signal)
{
    int ofdm_symbol = sh_data.subcarrier + sh_data.cyclic_prefex;
    out_signal.clear();
    out_signal.reserve(sh_data.rx_complex.size() - ofdm_symbol);

    if (sh_data.sync_pos < ofdm_symbol)
    {
        int start_idx = sh_data.sync_pos + ofdm_symbol;
        int rem_samples = sh_data.rx_complex.size() - start_idx;
        int cnt_samples = rem_samples / ofdm_symbol;
        int end_idx = start_idx + (cnt_samples * ofdm_symbol);

        for (int i = start_idx; i < end_idx; ++i)
            out_signal.push_back(sh_data.rx_complex[i]);
    }
    else
    {
        int right_start_idx = sh_data.sync_pos + ofdm_symbol;

        int rem_samples = sh_data.rx_complex.size() - right_start_idx;
        int cnt_samples = rem_samples / ofdm_symbol;
        int end_idx = right_start_idx + (cnt_samples * ofdm_symbol);

        for (int i = right_start_idx; i < end_idx; ++i)
            out_signal.push_back(sh_data.rx_complex[i]);

        int left_rem_buf_cnt = int(sh_data.sync_pos / ofdm_symbol);
        int left_start_idx = sh_data.sync_pos - left_rem_buf_cnt * ofdm_symbol;

        for (int i = left_start_idx; i < sh_data.sync_pos; ++i)
            out_signal.push_back(sh_data.rx_complex[i]);

    }
}

void cfo_correction(std::vector<std::complex<float>> &in_signal, sharedData &sh_data)
{
    sh_data.cfo_offset.clear();
    int subcarrar = sh_data.subcarrier;
    int cp = sh_data.cyclic_prefex;
    int sample_rate = sh_data.sample_rate;
    int ofdm_symbol = subcarrar + cp;
    int cnt_ofdm_symbols = in_signal.size() / ofdm_symbol;
    float total_phase = 0.0f;

    for (int n = 0; n < cnt_ofdm_symbols; ++n)
    {
        std::complex<float> corr = 0.0;
        int start = n * ofdm_symbol;
        for (int i = 0; i < cp; ++i)
            corr += conj(in_signal[i + start]) * in_signal[i + start + subcarrar];

        float eps = arg(corr) / (2.0f * M_PIf);
        float delta_f = eps * sample_rate / subcarrar;
        float phase_step = -2 * M_PIf * delta_f / sample_rate;

        for (int i = 0; i < ofdm_symbol; ++i)
        {
            total_phase += phase_step;
            sh_data.cfo_offset.push_back(total_phase);
            in_signal[start + i] *= std::complex<float>(std::cos(total_phase), std::sin(total_phase));
        }
    }
}

void remove_cp(std::vector<std::complex<float>> &in_signal, sharedData &sh_data, std::vector<std::complex<float>> &out_signal)
{
    out_signal.clear();
    size_t cnt_ofdm_symbols = in_signal.size() / (sh_data.cyclic_prefex + sh_data.subcarrier);
    out_signal.reserve(in_signal.size() - sh_data.cyclic_prefex * cnt_ofdm_symbols);

    for (size_t i = 0; i < cnt_ofdm_symbols; ++i)
        for (int j = 0; j < sh_data.subcarrier; ++j)
            out_signal.push_back(in_signal[j + (i * sh_data.subcarrier) + sh_data.cyclic_prefex + (i * sh_data.cyclic_prefex)]);
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

        fft(context);

        for (int k = 0; k < context.N; ++k)
            out_signal.push_back(std::complex<float>(context.out[k][0], context.out[k][1]));
    }
}

void equalization(std::vector<std::complex<float>> &in_signal, const sharedData &sh_data, std::vector<std::complex<float>> &out_signal)
{
    out_signal.clear();
    out_signal.reserve(in_signal.size());

    int subcarrar = sh_data.subcarrier;
    int num_symbols = in_signal.size() / subcarrar;

    for (int i = 0; i < num_symbols; ++i)
    {
        std::vector<std::complex<float>> H(subcarrar, {0.0f, 0.0f});
        std::vector<std::complex<float>> equalized(subcarrar, {0.0f, 0.0f});

        for (int p_idx : sh_data.pilot_idxs) {
            H[p_idx] = in_signal[i * subcarrar + p_idx];
        }

        for (size_t p = 0; p + 1 < sh_data.pilot_idxs.size(); ++p)
        {
            int k1 = sh_data.pilot_idxs[p];
            int k2 = sh_data.pilot_idxs[p + 1];

            std::complex<float> H1 = H[k1];
            std::complex<float> H2 = H[k2];

            for (int k = k1 + 1; k < k2; ++k)
            {
                if (sh_data.is_zeros[k]) continue;

                float alpha = float(k - k1) / float(k2 - k1);
                H[k] = H1 + alpha * (H2 - H1);
            }
        }

        for (int k = 0; k < sh_data.pilot_idxs.front(); ++k)
            H[k] = H[sh_data.pilot_idxs.front()];

        for (int k = sh_data.pilot_idxs.back() + 1; k < subcarrar; ++k)
            H[k] = H[sh_data.pilot_idxs.back()];

        for (int k = 0; k < subcarrar; ++k)
        {
            if (sh_data.is_zeros[k] || std::abs(H[k]) < 1e-6f)
                equalized[k] = {0, 0};
            else
                equalized[k] = in_signal[i * subcarrar + k] / H[k];
        }

        float phase = 0;
        int valid_pilots = 0;
        for (int p_idx : sh_data.pilot_idxs)
        {
            if (std::abs(equalized[p_idx]) > 1e-6f)
            {
                phase += std::arg(equalized[p_idx]);
                valid_pilots++;
            }
        }
        if (valid_pilots > 0) phase /= valid_pilots;

        std::complex<float> phase_corr = std::exp(std::complex<float>(0.0f, -phase));
        for (int k = 0; k < subcarrar; ++k)
            equalized[k] *= phase_corr;

        for (size_t k = 0; k < equalized.size(); ++k)
        {
            if (sh_data.is_zeros[k] || sh_data.is_pilot[k])
                continue;

            out_signal.push_back(equalized[k]);
        }
    }
}


void split_to_float(const std::complex<float> *__restrict src, float *__restrict dst_re, float *__restrict dst_im, size_t n)
{
    const float *raw_src = reinterpret_cast<const float *>(src);

    #pragma omp simd
    for (size_t i = 0; i < n; ++i)
    {
        dst_re[i] = raw_src[2 * i];
        dst_im[i] = raw_src[2 * i + 1];
    }
}

void split_int16_t_to_float(const int16_t *src, float *dst_re, float *dst_im, size_t num_samples)
{
    #pragma omp simd
    for (size_t i = 0; i < num_samples; ++i)
    {
        dst_re[i] = static_cast<float>(src[2 * i]) / 32768.0f;
        dst_im[i] = static_cast<float>(src[2 * i + 1]) / 32768.0f;
    }
}

void check_demapping(const std::vector<float> &in_signal, sharedData& sh_data)
{
    sh_data.bits.clear();
    sh_data.bits.reserve(in_signal.size());

    int ofdm_symbol_len = sh_data.subcarrier + sh_data.cyclic_prefex;
    int right_samples = sh_data.rx_complex.size() - (sh_data.sync_pos + ofdm_symbol_len);
    int right_symbols = std::max(0, right_samples / ofdm_symbol_len);

    if (right_symbols == 0)
        return;

    int bps = bits_per_symbol(sh_data.modul_type_TX);

    PRBS15 ref_gen;
    ref_gen.state = 0xACE1;

    for (int s = 0; s < right_symbols; ++s)
    {
        for (int k = 0; k < sh_data.subcarrier; ++k)
        {
            if (sh_data.is_zeros[k] || sh_data.is_pilot[k])
                continue;

            for (int b = 0; b < bps; ++b)
                sh_data.bits.push_back((float)ref_gen.get_bit());
        }
    }

    size_t len = std::min(sh_data.bits.size(), in_signal.size());
    for (size_t i = 0; i < len; ++i)
        if (sh_data.bits[i] != in_signal[i])
            sh_data.err_cnt++;
}
