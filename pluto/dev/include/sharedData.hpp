#pragma once

#include "types.hpp"

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
    std::vector<float> shifted_magnitude;
    std::vector<float> argument;
    std::vector<float> frequency_axis;
    std::vector<float> zadoff_corr_arr;
    std::vector<float> milisecs;
    std::vector<float> cfo_offset;
    std::atomic<bool> form = true;
    std::atomic<bool> read = false;
    std::atomic<bool> dsp = false;
    std::atomic<bool> changed_send = false;
    std::atomic<bool> changed_quit = false;
    std::atomic<bool> changed_rx_gain = false;
    std::atomic<bool> changed_tx_gain = false;
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
    float rx_gain = 20.f;
    float tx_gain = 80.f;
    float rx_frequency = 777e6;
    float tx_frequency = 777e6;
    float sample_rate = 1.92e6;
    float rx_bandwidth = 1e6;
    float tx_bandwidth = 1e6;
    int cyclic_prefex = 32;
    int subcarrier = 128;
    int sync_pos = 0;
    int mtu = 1920;
    int buffer = 3840;
    int zadoff_chu_u = 3;

    sharedData(size_t rx_mtu)
    {
        tx_buffer.resize(rx_mtu * 2, 0);
        rx_complex.resize(rx_mtu, 0);
        zadoff_corr_arr.resize(rx_mtu, 0);
        rx_complex_fft_gui.resize(rx_mtu, 0);
        shifted_magnitude.resize(rx_mtu, 0);
        argument.resize(rx_mtu, 0);
        frequency_axis.resize(rx_mtu, 0);
    }
};