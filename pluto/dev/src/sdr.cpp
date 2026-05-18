#include "sdr.hpp"

#include <stop_token>
#include <thread>

constexpr long long TIMEOUT = 400000;
constexpr long long TX_DELAY = 4000000;

void run_sdr(std::stop_token stoken, sharedData &sh_data)
{
    size_t length;
    SoapySDRKwargs *results = SoapySDRDevice_enumerate(nullptr, &length);
    sh_data.devices.clear();

    for (size_t i = 0; i < length / 2; ++i)
    {
        std::string label = static_cast<std::string>(results[i].vals[3]);
        sh_data.devices.push_back(label);
    }

    if (!sh_data.devices.empty())
        sh_data.device = sh_data.devices[0];

    SoapySDRKwargsList_clear(results, length);

    SDRDevice sdr(sh_data.device.c_str());

    while (!stoken.stop_requested())
    {
        if (!sh_data.flags.changed_cont_time)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        std::vector<int16_t> local_tx(sh_data.mtu * 2);
        {
            std::lock_guard<std::mutex> lock(sh_data.sync.tx_mutex);
            auto &src = sh_data.flags.one_time_mode ?
                sh_data.tx_buffer_one_time : sh_data.tx_buffer;
            std::copy(src.begin(), src.begin() + sh_data.mtu * 2, local_tx.begin());
        }

        void *tx_buffs[] = {local_tx.data()};
        void *rx_buffs[] = {sdr.rx_buffer.data()};

        int flags = 0;
        long long timeNs = 0;

        int sr = SoapySDRDevice_readStream(sdr.sdr, sdr.rxStream, rx_buffs, sh_data.mtu, &flags, &timeNs, TIMEOUT);
        if (sr < 0)
            std::cout << "[ERROR] Read stream | Error code: " << sr << "\n";

        long long tx_time = timeNs + TX_DELAY;
        flags = SOAPY_SDR_HAS_TIME;

        if (sh_data.flags.changed_send)
        {
            int st = SoapySDRDevice_writeStream(sdr.sdr, sdr.txStream, tx_buffs, sh_data.mtu, &flags, tx_time, TIMEOUT);
            if (st < 0)
                std::cout << "[ERROR] Write stream | Error code: " << st << "\n";
        }

        if (sh_data.flags.read)
        {
            for (int i = 0; i < sh_data.mtu; ++i)
                sh_data.rx_complex[i] = std::complex<float>(sdr.rx_buffer[2 * i], sdr.rx_buffer[2 * i + 1]);

            sh_data.flags.read = false;
            sh_data.flags.dsp = true;
        }

        if (sh_data.flags.changed_rx_gain)
        {
            if (int err; (err = SoapySDRDevice_setGain(sdr.sdr, SOAPY_SDR_RX, 0, sh_data.rx_gain)) != 0)
                std::cout << "[ERROR] Set RX gain | Error code: " << err << "\n";
            sh_data.flags.changed_rx_gain = false;
        }

        if (sh_data.flags.changed_tx_gain)
        {
            if (int err; (err = SoapySDRDevice_setGain(sdr.sdr, SOAPY_SDR_TX, 0, sh_data.tx_gain)) != 0)
                std::cout << "[ERROR] Set TX gain | Error code: " << err << "\n";
            sh_data.flags.changed_tx_gain = false;
        }

        if (sh_data.flags.changed_rx_freq)
        {
            if (int err; (err = SoapySDRDevice_setFrequency(sdr.sdr, SOAPY_SDR_RX, 0, sh_data.rx_frequency, NULL)) != 0)
                std::cout << "[ERROR] Set RX frequency | Error code: " << err << "\n";
            sh_data.flags.changed_rx_freq = false;
        }

        if (sh_data.flags.changed_tx_freq)
        {
            if (int err; (err = SoapySDRDevice_setFrequency(sdr.sdr, SOAPY_SDR_TX, 0, sh_data.tx_frequency, NULL)) != 0)
                std::cout << "[ERROR] Set TX frequency | Error code: " << err << "\n";
            sh_data.flags.changed_tx_freq = false;
        }

        if (sh_data.flags.changed_sample_rate)
        {
            if (int err; (err = SoapySDRDevice_setSampleRate(sdr.sdr, SOAPY_SDR_RX, 0, sh_data.sample_rate)) != 0)
                std::cout << "[ERROR] Set RX sample rate | Error code: " << err << "\n";
            if (int err; (err = SoapySDRDevice_setSampleRate(sdr.sdr, SOAPY_SDR_TX, 0, sh_data.sample_rate)) != 0)
                std::cout << "[ERROR] Set TX sample rate | Error code: " << err << "\n";
            sh_data.flags.changed_sample_rate = false;
        }

        if (sh_data.flags.changed_rx_bandwidth)
        {
            if (int err; (err = SoapySDRDevice_setBandwidth(sdr.sdr, SOAPY_SDR_RX, 0, sh_data.rx_bandwidth)) != 0)
                std::cout << "[ERROR] Set RX bandwidth | Error code: " << err << "\n";
            sh_data.flags.changed_rx_bandwidth = false;
        }

        if (sh_data.flags.changed_tx_bandwidth)
        {
            if (int err; (err = SoapySDRDevice_setBandwidth(sdr.sdr, SOAPY_SDR_TX, 0, sh_data.tx_bandwidth)) != 0)
                std::cout << "[ERROR] Set TX bandwidth | Error code: " << err << "\n";
            sh_data.flags.changed_tx_bandwidth = false;
        }

        if (sh_data.flags.changed_rx_gain_mode)
        {
            if (int err; (err = SoapySDRDevice_setGainMode(sdr.sdr, SOAPY_SDR_RX, 0, sh_data.flags.rx_gain_mode)) != 0)
                std::cout << "[ERROR] Set RX gain mode | Error code: " << err << "\n";
            sh_data.flags.changed_rx_gain_mode = false;
        }
    }
    return;
}
