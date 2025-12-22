#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <complex>
#include <cstdint>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <complex.h>
#include <math.h>
#include <vector>
#include <string.h>
#include <algorithm>
#include "modulation.h"

using namespace std;

constexpr int SRATE = 1000000;
constexpr int FREQ = 807000000;
constexpr int RX_GAIN = 40;
constexpr int TX_GAIN = -90;

constexpr int N_BITS = 1920;
constexpr int UPSAMPLE = 10;
constexpr int LEN_SYMBOLS = N_BITS / 2;
constexpr int LEN_SYMBOLS_UPS = LEN_SYMBOLS * UPSAMPLE;
constexpr int SCALE_FACTOR = 1000;
constexpr int BIT_SHIFT = 4;

constexpr size_t N_BUFFERS = 40000;
constexpr long long TIMEOUT = 400000;
constexpr long long TX_DELAY = 4000000;

struct SDRConfig {
    SoapySDRDevice *sdr;
    SoapySDRStream *rxStream;
    SoapySDRStream *txStream;
    size_t rx_mtu;
    size_t tx_mtu;
    int sample_rate;
    int carrier_freq;
    int16_t *rx_buffer;
};

struct SDRConfig init(char *usb){
    struct SDRConfig config = {};
    SoapySDRKwargs args = {};
    SoapySDRKwargs_set(&args, "driver", "plutosdr");
    SoapySDRKwargs_set(&args, "uri", usb);
    SoapySDRKwargs_set(&args, "direct", "1");
    SoapySDRKwargs_set(&args, "timestamp_every", "1920");
    SoapySDRKwargs_set(&args, "loopback", "0");
    config.sdr = SoapySDRDevice_make(&args);
    SoapySDRKwargs_clear(&args);

    config.sample_rate = SRATE;
    config.carrier_freq = FREQ;

    SoapySDRDevice_setSampleRate(config.sdr, SOAPY_SDR_RX, 0, SRATE);
    SoapySDRDevice_setFrequency(config.sdr, SOAPY_SDR_RX, 0, FREQ, NULL);
    SoapySDRDevice_setSampleRate(config.sdr, SOAPY_SDR_TX, 0, SRATE);
    SoapySDRDevice_setFrequency(config.sdr, SOAPY_SDR_TX, 0, FREQ, NULL);

    SoapySDRDevice_setGain(config.sdr, SOAPY_SDR_RX, 0, RX_GAIN);
    SoapySDRDevice_setGain(config.sdr, SOAPY_SDR_TX, 0, TX_GAIN);

    size_t rx_channels[] = {0};
    size_t tx_channels[] = {0};
    size_t channel_count = 1;

    config.rxStream = SoapySDRDevice_setupStream(config.sdr, SOAPY_SDR_RX, SOAPY_SDR_CS16, rx_channels, channel_count, NULL);
    config.txStream = SoapySDRDevice_setupStream(config.sdr, SOAPY_SDR_TX, SOAPY_SDR_CS16, tx_channels, channel_count, NULL);

    SoapySDRDevice_activateStream(config.sdr, config.rxStream, 0, 0, 0);
    SoapySDRDevice_activateStream(config.sdr, config.txStream, 0, 0, 0);

    config.rx_mtu = SoapySDRDevice_getStreamMTU(config.sdr, config.rxStream);
    config.tx_mtu = SoapySDRDevice_getStreamMTU(config.sdr, config.txStream);

    config.rx_buffer = (int16_t*)calloc(2 * config.rx_mtu, sizeof(int16_t));
    return config;
}

int main(int argc, char *argv[]){
    (void) argc;
    struct SDRConfig config = init(argv[1]);

    FILE *tx = fopen("tx.pcm", "wb");
    FILE *rx = fopen("rx.pcm", "wb");
    if (!tx || !rx) { perror("fopen"); return -1; }

    vector<int16_t> bits(N_BITS);
    vector<complex<double>> symbols(LEN_SYMBOLS);
    vector<complex<double>> symbols_ups(LEN_SYMBOLS_UPS);
    vector<complex<double>> impulse(UPSAMPLE, 1.0);
    vector<int16_t> tx_samples(2 * LEN_SYMBOLS_UPS);

    for (int i = 0; i < N_BITS; i++) bits[i] = rand() % 2;

    modulate(bits, symbols, ModulationType::QPSK);
    UpSampler(symbols, symbols_ups, UPSAMPLE);
    filter(symbols_ups, impulse);

    for (size_t i = 0; i < LEN_SYMBOLS_UPS; i++) {
        tx_samples[2*i] = (int16_t)(real(symbols_ups[i]) * SCALE_FACTOR) << BIT_SHIFT;
        tx_samples[2*i+1] = (int16_t)(imag(symbols_ups[i]) * SCALE_FACTOR) << BIT_SHIFT;
    }

    for (size_t i = 0; i < N_BUFFERS; ++i){
        void *rx_buffs[] = {config.rx_buffer};
        const void *tx_buffs[] = {tx_samples.data()};
        int flags = 0;
        long long timeNs = 0;

        int sr = SoapySDRDevice_readStream(config.sdr, config.rxStream, rx_buffs, config.rx_mtu, &flags, &timeNs, TIMEOUT);
        (void)sr;
        fwrite(rx_buffs[0], sizeof(int16_t), 2 * config.rx_mtu, rx);

        long long tx_time = timeNs + TX_DELAY;
        flags = SOAPY_SDR_HAS_TIME;
        int st = SoapySDRDevice_writeStream(config.sdr, config.txStream, tx_buffs, config.tx_mtu, &flags, tx_time, TIMEOUT);
        (void)st;
        fwrite(tx_buffs[0], sizeof(int16_t), 2 * config.tx_mtu, tx);
    }

    SoapySDRDevice_deactivateStream(config.sdr, config.rxStream, 0, 0);
    SoapySDRDevice_deactivateStream(config.sdr, config.txStream, 0, 0);
    SoapySDRDevice_closeStream(config.sdr, config.rxStream);
    SoapySDRDevice_closeStream(config.sdr, config.txStream);
    SoapySDRDevice_unmake(config.sdr);
    free(config.rx_buffer);
    fclose(rx);
    fclose(tx);
    return 0;
}
