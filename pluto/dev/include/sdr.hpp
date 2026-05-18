#pragma once

#include "common.hpp"

#include <SoapySDR/Constants.h>
#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <cstdint>
#include <vector>
#include <stop_token>

class SDRDevice
{
  public:
    SoapySDRDevice *sdr;
    SoapySDRStream *rxStream;
    SoapySDRStream *txStream;
    size_t rx_mtu;
    size_t tx_mtu;
    double sample_rate = 1.92e6;
    double frequency = 2.8e9;
    double rx_gain = 20.0;
    double tx_gain = 80.0;
    double bandwidth = 1e6;

    std::vector<int16_t> rx_buffer;
    std::vector<int16_t> tx_buffer;

    SDRDevice(const char *usb);
    ~SDRDevice();
};

void run_sdr(std::stop_token stoken, sharedData &sh_data);
