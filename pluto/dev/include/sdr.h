#pragma once

#include <SoapySDR/Constants.h>
#include <SoapySDR/Device.h>
#include <cstdint>
#include <vector>

constexpr double SAMPLE_RATE = 1000000.0;
constexpr double CARR_FREQ = 868000000.0;
constexpr double RX_GAIN = 25.0;
constexpr double TX_GAIN = 50.0;

class SDRDevice {
public:
    SoapySDRDevice* sdr;
    SoapySDRStream* rxStream;
    SoapySDRStream* txStream;
    size_t rx_mtu;
    size_t tx_mtu;

    std::vector<int16_t> rx_buffer;
    std::vector<int16_t> tx_buffer;

    SDRDevice(char* usb);
    ~SDRDevice();
};
