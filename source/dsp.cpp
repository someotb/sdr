#include "dsp.hpp"
#include "modulation.hpp"

#include <mutex>
#include <stop_token>
#include <thread>
#include <cstring>

void run_dsp(std::stop_token stoken, sharedData &sh_data)
{
    FFT_Context context(sh_data.subcarrier);
    FFT_Context context_spectre(sh_data.mtu);
    FFT_Context zad_off_chu_context(sh_data.subcarrier);

    for (int i = 0; i < sh_data.mtu; ++i)
        sh_data.frequency_axis[i] = (i - sh_data.mtu / 2.0) * sh_data.sample_rate / sh_data.mtu;

    int ofdm_symbol = sh_data.subcarrier + sh_data.cyclic_prefex;
    std::vector<int16_t> zadoff_chu_seq((sh_data.subcarrier + sh_data.cyclic_prefex) * 2);
    std::vector<std::complex<float>> rx_complex_remove_pss;
    std::vector<std::complex<float>> rx_complex_cfo;
    std::vector<std::complex<float>> rx_complex_remove_cp;
    std::vector<std::complex<float>> rx_complex_fft;
    std::vector<std::complex<float>> rx_complex_eq;

    std::vector<float> signal_re(sh_data.mtu, 0);
    std::vector<float> signal_im(sh_data.mtu, 0);

    std::vector<float> zc_re(sh_data.subcarrier + sh_data.cyclic_prefex, 0);
    std::vector<float> zc_im(sh_data.subcarrier + sh_data.cyclic_prefex, 0);
    int zad_of_idx = 0;

    build_pss_zadoff_chu(zad_off_chu_context, sh_data);
    append_symbol(zad_off_chu_context, zadoff_chu_seq, sh_data.cyclic_prefex, 0);
    split_int16_t_to_float(zadoff_chu_seq.data(), zc_re.data(), zc_im.data(), zc_re.size());

    PRBS15 prbs15;

    while (!stoken.stop_requested())
    {
        if (!sh_data.flags.changed_cont_time)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        if (sh_data.flags.constant_mode && !sh_data.flags.one_time_mode)
        {
            int start_idx = 0;
            prbs15.state = 0xACE1;

            calculate_pilots(sh_data);

            if (sh_data.flags.changed_pss_symbols)
            {
                append_symbol(zad_off_chu_context, sh_data.tx_buffer_back, sh_data.cyclic_prefex, 0);
                start_idx = 2 * ofdm_symbol;
            }

            for (int start = start_idx; start <= sh_data.buffer - 2 * ofdm_symbol; start += 2 * ofdm_symbol)
            {
                build_ofdm_symbol_prbs(prbs15, context, sh_data);
                append_symbol(context, sh_data.tx_buffer_back, sh_data.cyclic_prefex, start);
            }

            {
                std::lock_guard<std::mutex> lock(sh_data.sync.tx_mutex);
                sh_data.tx_buffer.swap(sh_data.tx_buffer_back);
            }
        }

        if (sh_data.flags.one_time_mode && !sh_data.flags.constant_mode)
        {
            int start_idx = 0;
            int offset = 0;
            std::vector<uint8_t> message_bits;

            calculate_pilots(sh_data);

            if (!sh_data.message.empty())
                message_bits = str_to_bits(sh_data.message);

            while (message_bits.size() < sh_data.tx_buffer_one_time.size())
                message_bits.push_back(0);

            if (sh_data.flags.changed_pss_symbols)
            {
                append_symbol(zad_off_chu_context, sh_data.tx_buffer_back, sh_data.cyclic_prefex, 0);
                start_idx = 2 * ofdm_symbol;
            }

            for (int start = start_idx; start <= sh_data.buffer - 2 * ofdm_symbol; start += 2 * ofdm_symbol)
            {
                build_ofdm_symbol(message_bits, context, sh_data, offset);
                append_symbol(context, sh_data.tx_buffer_back, sh_data.cyclic_prefex, start);
            }

            {
                std::lock_guard<std::mutex> lock(sh_data.sync.tx_mutex);
                sh_data.tx_buffer_one_time.swap(sh_data.tx_buffer_back);
            }
        }

        std::atomic_signal_fence(std::memory_order_seq_cst);
        auto start = std::chrono::steady_clock::now();
        std::atomic_signal_fence(std::memory_order_seq_cst);

        if (sh_data.flags.get_zadoff_pos_loopback)
        {
            split_to_float(sh_data.rx_complex.data(), signal_re.data(), signal_im.data(), signal_re.size());
            zad_of_idx = zadoff_sync(signal_re.data(), signal_im.data(), signal_re.size(), zc_re.data(), zc_im.data(), zc_re.size(),
                                        sh_data.zadoff_corr_arr.data());
            sh_data.sync_pos = zad_of_idx;
            sh_data.flags.get_zadoff_pos_loopback = false;
        }

        if (sh_data.flags.get_zadoff_pos)
        {
            split_to_float(sh_data.rx_complex.data(), signal_re.data(), signal_im.data(), signal_re.size());
            zad_of_idx = zadoff_sync(signal_re.data(), signal_im.data(), signal_re.size(), zc_re.data(), zc_im.data(), zc_re.size(),
                                        sh_data.zadoff_corr_arr.data());
            if (zad_of_idx > 1920)
                zad_of_idx = 1920;
            sh_data.sync_pos = zad_of_idx - sh_data.sync_offset;
        }

        remove_pss(sh_data, rx_complex_remove_pss);

        if (sh_data.flags.cfo_cor)
            cfo_correction(rx_complex_remove_pss, sh_data);

        remove_cp(rx_complex_remove_pss, sh_data, rx_complex_remove_cp);
        decode(rx_complex_remove_cp, rx_complex_fft, context);
        std::vector<float> signal_eq_float(rx_complex_fft.size());

        if (sh_data.flags.equal)
        {
            equalization(rx_complex_fft, sh_data, rx_complex_eq);
            if (signal_eq_float.size() != rx_complex_eq.size() * 2)
                signal_eq_float.resize(rx_complex_eq.size() * 2);

            std::memcpy(signal_eq_float.data(), rx_complex_eq.data(), rx_complex_eq.size() * sizeof(std::complex<float>));

            sh_data.demaped_bits.resize(signal_eq_float.size());
            demap_symbols(signal_eq_float, sh_data.demaped_bits, sh_data.modul_type_TX);
        }

        if (sh_data.flags.one_time_mode)
            sh_data.dec_message = bits_to_str(sh_data.demaped_bits);

        if (sh_data.flags.check_bits)
            check_demapping(sh_data.demaped_bits, sh_data);

        {
            std::lock_guard<std::mutex> lock(sh_data.sync.rx_mutex);
            sh_data.rx_complex_fft_gui = sh_data.flags.equal ? rx_complex_eq : rx_complex_fft;
        }

        spectrum(sh_data.rx_complex, sh_data.shifted_magnitude, sh_data.argument, context_spectre);

        std::atomic_signal_fence(std::memory_order_seq_cst);
        auto end = std::chrono::steady_clock::now();
        std::atomic_signal_fence(std::memory_order_seq_cst);

        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

        if (sh_data.flags.debug)
        {
            sh_data.milisecs.push_back(duration.count() / 1e3);
            if (sh_data.milisecs.size() == 1920)
                sh_data.milisecs.erase(sh_data.milisecs.begin());
        }
    }
}
