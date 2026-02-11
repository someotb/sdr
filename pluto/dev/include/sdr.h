#pragma once

#include <SoapySDR/Constants.h>
#include <SoapySDR/Device.h>
#include <cstdint>
#include <vector>

constexpr int SRATE = 1000000;
constexpr int FREQ = 868000000;
constexpr int RX_GAIN = 25;
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
    std::vector<int16_t> rx_buffer;
    std::vector<int16_t> tx_buffer;

    int set_gain_rx(double rx_gain) {
        int sg = SoapySDRDevice_setGain(sdr, SOAPY_SDR_RX, 0, rx_gain);
        return sg;
    }

    int set_gain_tx(double tx_gain) {
        int sg = SoapySDRDevice_setGain(sdr, SOAPY_SDR_TX, 0, tx_gain);
        return sg;
    }

    void set_frequency(double freq) {
        SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_RX, 0, freq, NULL);
        SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_TX, 0, freq, NULL);
    }

    SDRDevice(char* usb);
    ~SDRDevice();
};
