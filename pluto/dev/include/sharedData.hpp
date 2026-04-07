#pragma once

#include "types.hpp"

#include <numeric>
#include <string>
#include <atomic>
#include <vector>
#include <complex>

struct sharedData
{
    ModulationType modul_type_TX = ModulationType::QPSK;
    std::string device;
    std::vector<std::string> devices;
    std::vector<std::complex<float>> rx_complex;
    std::vector<std::complex<float>> rx_complex_fft_gui;
    std::vector<int16_t> tx_buffer;
    std::vector<float> demaped_bits;
    std::vector<float> bits;
    std::vector<float> shifted_magnitude;
    std::vector<float> argument;
    std::vector<float> frequency_axis;
    std::vector<float> zadoff_corr_arr;
    std::vector<float> milisecs;
    std::vector<float> cfo_offset;
    std::vector<int> pilot_idxs = {
        1, 8, 15, 22, 29, 35,
        92, 99, 106, 113, 120, 127
    };
    std::vector<int> zeros_idxs;
    std::vector<bool> is_pilot;
    std::vector<bool> is_zeros;

    float rx_gain = 20.f;
    float tx_gain = 80.f;
    float rx_frequency = 2.3e9;
    float tx_frequency = 2.3e9;
    float sample_rate = 1.92e6;
    float rx_bandwidth = 1e6;
    float tx_bandwidth = 1e6;
    int cyclic_prefex = 32;
    int subcarrier = 128;
    int sync_pos = 0;
    int mtu = 1920;
    int buffer = 3840;
    int zadoff_chu_u = 3;
    int err_cnt = 0;

    struct flags
    {
        std::atomic<bool> form = true;
        std::atomic<bool> read = false;
        std::atomic<bool> dsp = false;
        std::atomic<bool> changed_send = false;
        std::atomic<bool> changed_quit = false;
        std::atomic<bool> changed_rx_gain = false;
        std::atomic<bool> changed_tx_gain = false;
        bool rx_gain_mode = false;
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
    } flags;

    sharedData(size_t rx_mtu)
    {
        is_pilot.resize(128, false);
        for (auto &x : pilot_idxs)
            is_pilot[x] = true;

        zeros_idxs.resize((128 / 2 + 27) - (128 / 2 - 28));
        std::iota(zeros_idxs.begin(), zeros_idxs.end(), (128 / 2 - 28));
        is_zeros.resize(128, false);
        for (auto &x : zeros_idxs)
            is_zeros[x] = true;

        is_zeros[0] = true;

        tx_buffer.resize(rx_mtu * 2, 0);
        rx_complex.resize(rx_mtu, 0);
        zadoff_corr_arr.resize(rx_mtu, 0);
        rx_complex_fft_gui.resize(rx_mtu, 0);
        shifted_magnitude.resize(rx_mtu, 0);
        argument.resize(rx_mtu, 0);
        frequency_axis.resize(rx_mtu, 0);
    }
};
