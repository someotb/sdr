#include "dsp.hpp"
#include "modulation.hpp"

#include <mutex>
#include <stop_token>
#include <thread>
#include <cstring>

void run_dsp(std::stop_token stoken, sharedData &sd)
{
    FFT_Context context(sd.subcarrier);
    FFT_Context context_spectre(sd.mtu);
    FFT_Context zad_off_chu_context(sd.subcarrier);

    for (int i = 0; i < sd.mtu; ++i)
        sd.frequency_axis[i] = (i - sd.mtu / 2.0) * sd.sample_rate / sd.mtu;

    const int ofdm_symbol = sd.subcarrier + sd.cyclic_prefex;
    std::vector<int16_t> zadoff_chu_seq((sd.subcarrier + sd.cyclic_prefex) * 2);
    std::vector<std::complex<float>> rx_complex_remove_pss;
    std::vector<std::complex<float>> rx_complex_cfo;
    std::vector<std::complex<float>> rx_complex_remove_cp;
    std::vector<std::complex<float>> rx_complex_fft;
    std::vector<std::complex<float>> rx_complex_eq;
    std::vector<std::complex<float>> rx_local;
    std::vector<uint8_t> bits;
    bits.reserve(sd.bits_cnt);

    std::vector<float> signal_re(sd.mtu, 0);
    std::vector<float> signal_im(sd.mtu, 0);

    std::vector<float> zc_re(sd.subcarrier + sd.cyclic_prefex, 0);
    std::vector<float> zc_im(sd.subcarrier + sd.cyclic_prefex, 0);
    int zad_of_idx = 0;

    std::vector<int16_t> tx_buffer_back;

    build_pss_zadoff_chu(zad_off_chu_context, sd);
    add_cp(zad_off_chu_context, zadoff_chu_seq, sd.cyclic_prefex, 0);
    split_int16_t_to_float(zadoff_chu_seq.data(), zc_re.data(), zc_im.data(), ofdm_symbol);

    while (!stoken.stop_requested())
    {
        if (!sd.flags.changed_cont_time)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        if (sd.flags.pilots_change)
        {
            calculate_pilots(sd);
            sd.flags.pilots_change = false;
        }

        if (sd.flags.constant_mode)
        {
            if (sd.flags.bits_regen)
            {
                gen_bits(bits, sd);
                sd.flags.bits_regen = false;
            }

            if (sd.flags.bits_cnt_change)
            {
                update_bits(bits, sd);
                sd.flags.bits_cnt_change = false;
            }

            tx_buffer_back = build_tx_buffer(bits, zadoff_chu_seq, sd, context);

            {
                std::lock_guard<std::mutex> lock(sd.sync.tx_mutex);
                sd.tx_buffer.swap(tx_buffer_back);
            }
        }

        // if (sd.flags.one_time_mode && !sd.flags.constant_mode)
        // {
        //     int start_idx = 0;
        //     int offset = 0;
        //     std::vector<uint8_t> message_bits;

        //     calculate_pilots(sd);

        //     if (!sd.message.empty())
        //         message_bits = str_to_bits(sd.message);

        //     while (message_bits.size() < sd.tx_buffer_one_time.size())
        //         message_bits.push_back(0);

        //     if (sd.flags.changed_pss_symbols)
        //     {
        //         add_cp(zad_off_chu_context, sd.tx_buffer_back, sd.cyclic_prefex, 0);
        //         start_idx = 2 * ofdm_symbol;
        //     }

        //     for (int start = start_idx; start <= sd.buffer - 2 * ofdm_symbol; start += 2 * ofdm_symbol)
        //     {
        //         build_ofdm_symbol(message_bits, context, sd, offset);
        //         add_cp(context, sd.tx_buffer_back, sd.cyclic_prefex, start);
        //     }

        //     {
        //         std::lock_guard<std::mutex> lock(sd.sync.tx_mutex);
        //         sd.tx_buffer_one_time.swap(sd.tx_buffer_back);
        //     }
        // }

        auto start = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lock(sd.sync.rx_mutex);
            rx_local.assign(sd.rx_complex.begin(), sd.rx_complex.end());
        }

        if (sd.flags.get_zadoff_pos_loopback)
        {
            split_to_float(rx_local.data(), signal_re.data(), signal_im.data(), signal_re.size());
            zad_of_idx = zadoff_sync(signal_re.data(), signal_im.data(), signal_re.size(), zc_re.data(), zc_im.data(), zc_re.size(),
                                        sd.zadoff_corr_arr.data());

            if (zad_of_idx > sd.mtu)
                zad_of_idx = sd.mtu;

            sd.sync_pos = zad_of_idx - sd.sync_offset;
            sd.flags.get_zadoff_pos_loopback = false;
        }

        if (sd.flags.get_zadoff_pos)
        {
            split_to_float(rx_local.data(), signal_re.data(), signal_im.data(), signal_re.size());
            zad_of_idx = zadoff_sync(signal_re.data(), signal_im.data(), signal_re.size(), zc_re.data(), zc_im.data(), zc_re.size(),
                                        sd.zadoff_corr_arr.data());
            if (zad_of_idx > sd.mtu)
                zad_of_idx = sd.mtu;

            if ((int)sd.zd_idx.size() < sd.mtu)
                sd.zd_idx.push_back(zad_of_idx);
            else
            {
                sd.zd_idx.erase(sd.zd_idx.begin());
                sd.zd_idx.push_back(zad_of_idx);
            }

            sd.sync_pos = zad_of_idx + sd.sync_offset + sd.cyclic_prefex;
        }

        remove_pss(sd, rx_local, rx_complex_remove_pss);

        if (sd.flags.cfo_cor)
            rx_complex_remove_pss = cfo_est(rx_complex_remove_pss, sd);

        remove_cp(rx_complex_remove_pss, sd, rx_complex_remove_cp);
        decode(rx_complex_remove_cp, rx_complex_fft, context);
        std::vector<float> signal_eq_float(rx_complex_fft.size());

        if (sd.flags.equal)
        {
            equalization(rx_complex_fft, sd, rx_complex_eq);
            if (signal_eq_float.size() != rx_complex_eq.size() * 2)
                signal_eq_float.resize(rx_complex_eq.size() * 2);

            std::memcpy(signal_eq_float.data(), rx_complex_eq.data(), rx_complex_eq.size() * sizeof(std::complex<float>));

            sd.demaped_bits.resize(signal_eq_float.size());
            demap_symbols(signal_eq_float, sd.demaped_bits, sd.modul_type_TX);
        }

        if (sd.flags.one_time_mode)
            sd.dec_message = bits_to_str(sd.demaped_bits);

        {
            std::lock_guard<std::mutex> lock(sd.sync.rx_mutex);
            sd.rx_complex_fft_gui = sd.flags.equal ? rx_complex_eq : rx_complex_fft;
        }

        {
            std::lock_guard<std::mutex> lock(sd.sync.magnitude_argument_mutex);
            spectrum(rx_local, sd.shifted_magnitude, sd.argument, context_spectre);
        }

        auto end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

        if (sd.flags.debug)
        {
            sd.milisecs.push_back(duration.count() / 1e3);
            if (sd.milisecs.size() == 1920)
                sd.milisecs.erase(sd.milisecs.begin());
        }
    }
}
