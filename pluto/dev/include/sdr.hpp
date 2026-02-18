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
    double rx_gain = 20.0;
    double tx_gain = 80.0;
    double bandwidth = 1e6;

    vector<int16_t> rx_buffer;
    vector<int16_t> tx_buffer;

    SDRDevice(char* usb): sdr(nullptr), rxStream(nullptr), txStream(nullptr) {
        SoapySDRKwargs args = {};
        SoapySDRKwargs_set(&args, "driver", "plutosdr");
        SoapySDRKwargs_set(&args, "uri", usb);
        SoapySDRKwargs_set(&args, "direct", "1");
        SoapySDRKwargs_set(&args, "timestamp_every", "1920");
        SoapySDRKwargs_set(&args, "loopback", "0");
        sdr = SoapySDRDevice_make(&args);
        SoapySDRKwargs_clear(&args);

        SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_RX, 0, sample_rate);
        SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_RX, 0, frequency, NULL);
        SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_TX, 0, sample_rate);
        SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_TX, 0, frequency, NULL);

        SoapySDRDevice_setBandwidth(sdr, SOAPY_SDR_RX, 0, bandwidth);
        SoapySDRDevice_setBandwidth(sdr, SOAPY_SDR_TX, 0, bandwidth);

        SoapySDRDevice_setGain(sdr, SOAPY_SDR_RX, 0, rx_gain);
        SoapySDRDevice_setGain(sdr, SOAPY_SDR_TX, 0, tx_gain);

        size_t rx_channels[] = {0};
        size_t tx_channels[] = {0};
        size_t channel_count = 1;

        rxStream = SoapySDRDevice_setupStream(sdr, SOAPY_SDR_RX, SOAPY_SDR_CS16, rx_channels, channel_count, NULL);
        txStream = SoapySDRDevice_setupStream(sdr, SOAPY_SDR_TX, SOAPY_SDR_CS16, tx_channels, channel_count, NULL);

        SoapySDRDevice_activateStream(sdr, rxStream, 0, 0, 0);
        SoapySDRDevice_activateStream(sdr, txStream, 0, 0, 0);

        rx_mtu = SoapySDRDevice_getStreamMTU(sdr, rxStream);
        tx_mtu = SoapySDRDevice_getStreamMTU(sdr, txStream);

        rx_buffer.resize(2 * rx_mtu);
        tx_buffer.resize(2 * tx_mtu);
    }

    ~SDRDevice() {
        std::cout << "SDRDevice destructor called\n";
        if (sdr) {
            if (rxStream) {
                SoapySDRDevice_deactivateStream(sdr, rxStream, 0, 0);
                SoapySDRDevice_closeStream(sdr, rxStream);
            }
            if (txStream) {
                SoapySDRDevice_deactivateStream(sdr, txStream, 0, 0);
                SoapySDRDevice_closeStream(sdr, txStream);
            }
            SoapySDRDevice_unmake(sdr);
        }
    }
};
