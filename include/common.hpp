#pragma once

#include <fftw3.h>
#include <atomic>
#include <complex>
#include <string>
#include <vector>
#include <mutex>

class FFT_Context
{
    private:
        fftwf_plan plan_forward;
        fftwf_plan plan_backward;

    public:
        int N;
        fftwf_complex *in;
        fftwf_complex *out;
        FFT_Context(int n);
        ~FFT_Context();
        void fft();
        void ifft();
};

enum class ModulationType
{
    BPSK,
    QPSK,
    QAM16,
    QAM64,
    QAM256
};

struct PRBS15
{
    uint16_t state = 0xACE1;

    int get_bit()
    {
        int new_bit = ((state >> 14) ^ (state >> 13)) & 1;
        state = (state << 1) | new_bit;
        return new_bit & 1;
    }
};

struct sharedData
{
    ModulationType modul_type_TX = ModulationType::QPSK;
    std::string device;
    std::vector<std::string> devices;
    std::vector<std::complex<float>> rx_complex;
    std::vector<std::complex<float>> rx_complex_fft_gui;
    std::vector<int16_t> tx_buffer;
    std::vector<int16_t> tx_buffer_one_time;
    std::vector<int16_t> tx_buffer_back;
    std::vector<float> demaped_bits;
    std::vector<float> bits;
    std::vector<float> shifted_magnitude;
    std::vector<float> argument;
    std::vector<float> frequency_axis;
    std::vector<float> zadoff_corr_arr;
    std::vector<float> milisecs;
    std::vector<float> cfo_offset;
    std::vector<bool> is_pilot;
    std::vector<bool> is_zeros;
    std::vector<int> pilot_idxs;
    std::string message = "Hello World!";
    std::string dec_message = "Hello World!";

    float rx_gain = 20.f;
    float tx_gain = 80.f;
    float rx_frequency = 2.8e9;
    float tx_frequency = 2.8e9;
    float sample_rate = 1.92e6;
    float rx_bandwidth = 1e6;
    float tx_bandwidth = 1e6;
    int cyclic_prefex = 32;
    int subcarrier = 128;
    int sync_pos = 0;
    int sync_offset = 0;
    int mtu = 1920;
    int buffer = 3840;
    int zadoff_chu_u = 3;
    int err_cnt = 0;
    int cnt_pilots = 4;
    char input_buffer[256] = "";

    struct flags
    {
        std::atomic<bool> constant_mode = false;
        std::atomic<bool> one_time_mode = false;
        std::atomic<bool> changed_send = false;
        std::atomic<bool> changed_rx_gain = false;
        std::atomic<bool> changed_tx_gain = false;
        bool rx_gain_mode = false;
        bool show_input_window = false;
        std::atomic<bool> changed_rx_gain_mode = false;
        std::atomic<bool> changed_rx_freq = false;
        std::atomic<bool> changed_tx_freq = false;
        std::atomic<bool> changed_sample_rate = false;
        std::atomic<bool> changed_rx_bandwidth = false;
        std::atomic<bool> changed_tx_bandwidth = false;
        std::atomic<bool> changed_modulation_type = false;
        std::atomic<bool> changed_pss_symbols = false;
        std::atomic<bool> changed_cont_time = true;
        std::atomic<bool> get_zadoff_pos_loopback = false;
        std::atomic<bool> get_zadoff_pos = false;
        std::atomic<bool> rm_pilots = false;
        std::atomic<bool> debug = false;
        std::atomic<bool> cfo_cor = false;
        std::atomic<bool> equal = false;
        std::atomic<bool> check_bits = false;
        std::atomic<bool> tx_buffer_ready = false;
    } flags;

    struct sync
    {
        std::mutex tx_mutex;
        std::mutex rx_mutex;
        std::mutex magnitude_argument_mutex;
    } sync;


    sharedData(size_t rx_mtu)
    {
        is_pilot.resize(128, false);
        is_zeros.resize(128, false);
        for (int i = 0; i < 128; ++i)
            if ((i > 128 / 2 - 28 and i < 128 / 2 + 28) or i == 0)
                is_zeros[i] = true;

        tx_buffer_one_time.resize(rx_mtu * 2, 0);
        tx_buffer.resize(rx_mtu * 2, 0);
        rx_complex.resize(rx_mtu, 0);
        zadoff_corr_arr.resize(rx_mtu, 0);
        rx_complex_fft_gui.resize(rx_mtu, 0);
        shifted_magnitude.resize(rx_mtu, 0);
        argument.resize(rx_mtu, 0);
        frequency_axis.resize(rx_mtu, 0);
        tx_buffer_back.resize(rx_mtu * 2, 0);
    }
};
