#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <complex>
#include <cstdint>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <complex.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <string.h>
#include "modulation.h"
#include "sdr.h"

using namespace std;

constexpr int UPSAMPLE = 10;
constexpr size_t N_BUFFERS = 10000000;
constexpr long long TIMEOUT = 400000;
constexpr long long TX_DELAY = 4000000;

int main(int argc, char *argv[]){
    (void) argc;

    SDRDevice sdr(argv[1]);
    ModulationType modulation = ModulationType::QPSK;
    int bits_size = bits_per_symbol(modulation);

    FILE *tx = fopen("../pcm/tx.pcm", "wb");
    FILE *rx = fopen("../pcm/rx.pcm", "wb");
    if (!tx || !rx) { perror("fopen"); return -1; }

    vector<int16_t> bits(sdr.tx_mtu * bits_size);
    vector<complex<double>> symbols(bits.size() / bits_size);
    vector<complex<double>> impulse(UPSAMPLE, 1.0);
    vector<complex<double>> symbols_ups(sdr.tx_mtu * UPSAMPLE);

    for (size_t i = 0; i < bits.size(); i++) bits[i] = rand() % 2;

    modulate(bits, symbols, modulation);
    UpSampler(symbols, symbols_ups, UPSAMPLE);
    filter(symbols_ups, impulse);

    for (size_t i = 0; i < sdr.tx_mtu; i++) {
        sdr.tx_buffer[2*i] = (real(symbols_ups[i]) * 16000);
        sdr.tx_buffer[2*i+1] = (imag(symbols_ups[i]) * 16000);
    }

    int cnt = 0;
    cout << "Send '" << N_BUFFERS << "' buffers:" << endl;
    for (size_t i = 0; i < N_BUFFERS; ++i) {
        if (i % 520 == 0 && i != 0) {
            cnt++;
            cout << "Seconds: " << cnt << "\t" << "Buffers: " << i << endl;
        }
        void *rx_buffs[] = {sdr.rx_buffer.data()};
        const void *tx_buffs[] = {sdr.tx_buffer.data()};

        int flags = 0;
        long long timeNs = 0;

        int sr = SoapySDRDevice_readStream(sdr.sdr, sdr.rxStream, rx_buffs, sdr.rx_mtu, &flags, &timeNs, TIMEOUT);
        (void)sr;
        fwrite(rx_buffs[0], sizeof(int16_t), 2 * sdr.rx_mtu, rx);

        long long tx_time = timeNs + TX_DELAY;
        flags = SOAPY_SDR_HAS_TIME;

        int st = SoapySDRDevice_writeStream(sdr.sdr, sdr.txStream, tx_buffs, sdr.tx_mtu, &flags, tx_time, TIMEOUT);
        (void)st;
        fwrite(tx_buffs[0], sizeof(int16_t), 2 * sdr.tx_mtu, tx);
    }
    cout << "Transmission of '" << N_BUFFERS << "' buffers completed!" << endl;

    fclose(rx);
    fclose(tx);
    return 0;
}
