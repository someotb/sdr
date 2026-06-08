#include "sdr.hpp"

#include <iostream>
#include <stop_token>
#include <thread>

constexpr long long TIMEOUT = 400000;
constexpr long long TX_DELAY = 4000000;

SDRDevice::SDRDevice(const char *usb) : sdr(nullptr), rxStream(nullptr), txStream(nullptr)
{
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

SDRDevice::~SDRDevice()
{
    std::cout << "SDRDevice Destructor Called\n";
    if (sdr)
    {
        if (rxStream)
        {
            SoapySDRDevice_deactivateStream(sdr, rxStream, 0, 0);
            SoapySDRDevice_closeStream(sdr, rxStream);
        }
        if (txStream)
        {
            SoapySDRDevice_deactivateStream(sdr, txStream, 0, 0);
            SoapySDRDevice_closeStream(sdr, txStream);
        }
        SoapySDRDevice_unmake(sdr);
    }
}

void run_sdr(std::stop_token stoken, sharedData &sd)
{
    size_t length;
    SoapySDRKwargs *results = SoapySDRDevice_enumerate(nullptr, &length);
    sd.devices.clear();

    for (size_t i = 0; i < length; ++i)
    {
        std::string label = static_cast<std::string>(results[i].vals[3]);
        sd.devices.push_back(label);
    }

    if (!sd.devices.empty())
        sd.device = sd.devices[0];

    SoapySDRKwargsList_clear(results, length);

    SDRDevice sdr(sd.device.c_str());

    while (!stoken.stop_requested())
    {
        if (!sd.flags.changed_cont_time)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        std::vector<int16_t> local_tx(sd.buffer);
        {
            std::lock_guard<std::mutex> lock(sd.sync.tx_mutex);
            std::copy(sd.tx_buffer.begin(), sd.tx_buffer.begin() + sd.buffer, local_tx.begin());
        }

        void *tx_buffs[] = {local_tx.data()};
        void *rx_buffs[] = {sdr.rx_buffer.data()};

        int flags = 0;
        long long timeNs = 0;

        int sr = SoapySDRDevice_readStream(sdr.sdr, sdr.rxStream, rx_buffs, sd.mtu, &flags, &timeNs, TIMEOUT);
        if (sr < 0)
            std::cout << "[ERROR] Read stream | Error code: " << sr << "\n";

        long long tx_time = timeNs + TX_DELAY;
        flags = SOAPY_SDR_HAS_TIME;

        if (sd.flags.changed_send)
        {
            int st = SoapySDRDevice_writeStream(sdr.sdr, sdr.txStream, tx_buffs, sd.mtu, &flags, tx_time, TIMEOUT);
            if (st < 0)
                std::cout << "[ERROR] Write stream | Error code: " << st << "\n";
        }

        {
            std::lock_guard<std::mutex> lock(sd.sync.rx_mutex);
            for (int i = 0; i < sd.mtu; ++i)
                sd.rx_complex[i] = std::complex<float>(sdr.rx_buffer[2 * i], sdr.rx_buffer[2 * i + 1]);
        }

        if (sd.flags.changed_rx_gain)
        {
            if (int err; (err = SoapySDRDevice_setGain(sdr.sdr, SOAPY_SDR_RX, 0, sd.rx_gain)) != 0)
                std::cout << "[ERROR] Set RX gain | Error code: " << err << "\n";
            sd.flags.changed_rx_gain = false;
        }

        if (sd.flags.changed_tx_gain)
        {
            if (int err; (err = SoapySDRDevice_setGain(sdr.sdr, SOAPY_SDR_TX, 0, sd.tx_gain)) != 0)
                std::cout << "[ERROR] Set TX gain | Error code: " << err << "\n";
            sd.flags.changed_tx_gain = false;
        }

        if (sd.flags.changed_rx_freq)
        {
            if (int err; (err = SoapySDRDevice_setFrequency(sdr.sdr, SOAPY_SDR_RX, 0, sd.rx_frequency, NULL)) != 0)
                std::cout << "[ERROR] Set RX frequency | Error code: " << err << "\n";
            sd.flags.changed_rx_freq = false;
        }

        if (sd.flags.changed_tx_freq)
        {
            if (int err; (err = SoapySDRDevice_setFrequency(sdr.sdr, SOAPY_SDR_TX, 0, sd.tx_frequency, NULL)) != 0)
                std::cout << "[ERROR] Set TX frequency | Error code: " << err << "\n";
            sd.flags.changed_tx_freq = false;
        }

        if (sd.flags.changed_sample_rate)
        {
            if (int err; (err = SoapySDRDevice_setSampleRate(sdr.sdr, SOAPY_SDR_RX, 0, sd.sample_rate)) != 0)
                std::cout << "[ERROR] Set RX sample rate | Error code: " << err << "\n";
            if (int err; (err = SoapySDRDevice_setSampleRate(sdr.sdr, SOAPY_SDR_TX, 0, sd.sample_rate)) != 0)
                std::cout << "[ERROR] Set TX sample rate | Error code: " << err << "\n";
            sd.flags.changed_sample_rate = false;
        }

        if (sd.flags.changed_rx_bandwidth)
        {
            if (int err; (err = SoapySDRDevice_setBandwidth(sdr.sdr, SOAPY_SDR_RX, 0, sd.rx_bandwidth)) != 0)
                std::cout << "[ERROR] Set RX bandwidth | Error code: " << err << "\n";
            sd.flags.changed_rx_bandwidth = false;
        }

        if (sd.flags.changed_tx_bandwidth)
        {
            if (int err; (err = SoapySDRDevice_setBandwidth(sdr.sdr, SOAPY_SDR_TX, 0, sd.tx_bandwidth)) != 0)
                std::cout << "[ERROR] Set TX bandwidth | Error code: " << err << "\n";
            sd.flags.changed_tx_bandwidth = false;
        }

        if (sd.flags.changed_rx_gain_mode)
        {
            if (int err; (err = SoapySDRDevice_setGainMode(sdr.sdr, SOAPY_SDR_RX, 0, sd.flags.rx_gain_mode)) != 0)
                std::cout << "[ERROR] Set RX gain mode | Error code: " << err << "\n";
            sd.flags.changed_rx_gain_mode = false;
        }
    }
    return;
}
