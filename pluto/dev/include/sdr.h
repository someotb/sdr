#pragma once

#include <SoapySDR/Device.h>
#include <cstdint>

constexpr int SRATE = 1000000;
constexpr int FREQ = 734750000;
constexpr int RX_GAIN = 40;
constexpr int TX_GAIN = -10;

class SDRDevice {
public:
    SoapySDRDevice* sdr;
    SoapySDRStream* rxStream;
    SoapySDRStream* txStream;
    size_t rx_mtu;
    size_t tx_mtu;
    int sample_rate;
    int carrier_freq;
    int16_t* rx_buffer;

    SDRDevice(char* usb);
    ~SDRDevice();
};
