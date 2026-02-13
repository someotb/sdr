#pragma once

#include <SoapySDR/Constants.h>
#include <SoapySDR/Device.h>
#include <cstdint>
#include <vector>

class SDRDevice {
public:
    SoapySDRDevice* sdr;
    SoapySDRStream* rxStream;
    SoapySDRStream* txStream;
    size_t rx_mtu;
    size_t tx_mtu;
    double sample_rate = 1e6;
    double frequency = 826e6;
    double rx_gain = 25.0;
    double tx_gain = 20.0;

    std::vector<int16_t> rx_buffer;
    std::vector<int16_t> tx_buffer;

    SDRDevice(char* usb);
    ~SDRDevice();
};
