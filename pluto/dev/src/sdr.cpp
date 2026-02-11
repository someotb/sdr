#include "sdr.h"
#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <stdlib.h>
#include <string.h>

SDRDevice::SDRDevice(char* usb): sdr(nullptr), rxStream(nullptr), txStream(nullptr) {
    SoapySDRKwargs args = {};
    SoapySDRKwargs_set(&args, "driver", "plutosdr");
    SoapySDRKwargs_set(&args, "uri", usb);
    SoapySDRKwargs_set(&args, "direct", "1");
    SoapySDRKwargs_set(&args, "timestamp_every", "1920");
    SoapySDRKwargs_set(&args, "loopback", "0");
    sdr = SoapySDRDevice_make(&args);
    SoapySDRKwargs_clear(&args);

    sample_rate = SRATE;
    carrier_freq = FREQ;

    SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_RX, 0, SRATE);
    SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_RX, 0, FREQ, NULL);
    SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_TX, 0, SRATE);
    SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_TX, 0, FREQ, NULL);

    SoapySDRDevice_setGainMode(sdr, SOAPY_SDR_RX, 0, false);
    SoapySDRDevice_setGain(sdr, SOAPY_SDR_RX, 0, RX_GAIN);
    SoapySDRDevice_setGain(sdr, SOAPY_SDR_TX, 0, TX_GAIN);

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

SDRDevice::~SDRDevice() {
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
