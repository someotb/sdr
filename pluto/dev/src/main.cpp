#include "modulation.hpp"
#include "sharedData.hpp"
#include "types.hpp"
#include "sdr.hpp"

#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <SoapySDR/Types.h>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <complex.h>
#include <math.h>
#include <unistd.h>
#include <vector>
#include <string.h>
#include <GL/glew.h>
#include <SDL2/SDL.h>
#include <thread>
#include <atomic>
#include <cmath>
#include <iostream>
#include "imgui.h"
#include "implot.h"
#include "backends/imgui_impl_opengl3.h"
#include "backends/imgui_impl_sdl2.h"

constexpr long long TIMEOUT = 400000;
constexpr long long TX_DELAY = 4000000;

void run_backend(sharedData *sh_data)
{
    size_t length;
    SoapySDRKwargs *results = SoapySDRDevice_enumerate(nullptr, &length);
    sh_data->devices.clear();

    for (size_t i = 0; i < length / 2; ++i)
    {
        std::string label = static_cast<std::string>(results[i].vals[3]);
        sh_data->devices.push_back(label);
    }

    if (!sh_data->devices.empty())
        sh_data->device = sh_data->devices[0];

    SoapySDRKwargsList_clear(results, length);

    SDRDevice sdr(sh_data->device.c_str());

    while (sh_data->changed_quit == false)
    {
        if (!sh_data->changed_cont_time)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        if (sh_data->read)
        {
            void *tx_buffs[] = {sh_data->tx_buffer.data()};
            void *rx_buffs[] = {sdr.rx_buffer.data()};

            int flags = 0;
            long long timeNs = 0;

            int sr = SoapySDRDevice_readStream(sdr.sdr, sdr.rxStream, rx_buffs, sdr.rx_mtu, &flags, &timeNs, TIMEOUT);
            if (sr != sh_data->mtu)
                std::cout << "[ERROR] Read stream | Error code: " << sr << "\n";

            long long tx_time = timeNs + TX_DELAY;
            flags = SOAPY_SDR_HAS_TIME;

            if (sh_data->changed_send)
            {
                int st = SoapySDRDevice_writeStream(sdr.sdr, sdr.txStream, tx_buffs, sdr.tx_mtu, &flags, tx_time, TIMEOUT);
                if (st != sh_data->mtu)
                    std::cout << "[ERROR] Write stream | Error code: " << st << "\n";
            }

            for (int i = 0; i < sh_data->mtu; ++i)
                sh_data->rx_complex[i] = std::complex<float>(sdr.rx_buffer[2 * i], sdr.rx_buffer[2 * i + 1]);

            sh_data->read = false;
            sh_data->dsp = true;
        }

        if (sh_data->changed_rx_gain)
        {
            if (int err; (err = SoapySDRDevice_setGain(sdr.sdr, SOAPY_SDR_RX, 0, sh_data->rx_gain)) != 0)
                std::cout << "[ERROR] Set RX gain | Error code: " << err << "\n";
            sh_data->changed_rx_gain = false;
        }

        if (sh_data->changed_tx_gain)
        {
            if (int err; (err = SoapySDRDevice_setGain(sdr.sdr, SOAPY_SDR_TX, 0, sh_data->tx_gain)) != 0)
                std::cout << "[ERROR] Set TX gain | Error code: " << err << "\n";
            sh_data->changed_tx_gain = false;
        }

        if (sh_data->changed_rx_freq)
        {
            if (int err; (err = SoapySDRDevice_setFrequency(sdr.sdr, SOAPY_SDR_RX, 0, sh_data->rx_frequency, NULL)) != 0)
                std::cout << "[ERROR] Set RX frequency | Error code: " << err << "\n";
            sh_data->changed_rx_freq = false;
        }

        if (sh_data->changed_tx_freq)
        {
            if (int err; (err = SoapySDRDevice_setFrequency(sdr.sdr, SOAPY_SDR_TX, 0, sh_data->tx_frequency, NULL)) != 0)
                std::cout << "[ERROR] Set TX frequency | Error code: " << err << "\n";
            sh_data->changed_tx_freq = false;
        }

        if (sh_data->changed_sample_rate)
        {
            if (int err; (err = SoapySDRDevice_setSampleRate(sdr.sdr, SOAPY_SDR_RX, 0, sh_data->sample_rate)) != 0)
                std::cout << "[ERROR] Set RX sample rate | Error code: " << err << "\n";
            if (int err; (err = SoapySDRDevice_setSampleRate(sdr.sdr, SOAPY_SDR_TX, 0, sh_data->sample_rate)) != 0)
                std::cout << "[ERROR] Set TX sample rate | Error code: " << err << "\n";
            sh_data->changed_sample_rate = false;
        }

        if (sh_data->changed_rx_bandwidth)
        {
            if (int err; (err = SoapySDRDevice_setBandwidth(sdr.sdr, SOAPY_SDR_RX, 0, sh_data->rx_bandwidth)) != 0)
                std::cout << "[ERROR] Set RX bandwidth | Error code: " << err << "\n";
            sh_data->changed_rx_bandwidth = false;
        }

        if (sh_data->changed_tx_bandwidth)
        {
            if (int err; (err = SoapySDRDevice_setBandwidth(sdr.sdr, SOAPY_SDR_TX, 0, sh_data->tx_bandwidth)) != 0)
                std::cout << "[ERROR] Set TX bandwidth | Error code: " << err << "\n";
            sh_data->changed_tx_bandwidth = false;
        }
    }
}

void run_dsp(sharedData *sh_data)
{
    FFT_Context context(sh_data->subcarrier);
    FFT_Context context_spectre(sh_data->mtu);
    FFT_Context zad_off_chu_context(sh_data->subcarrier);

    for (int i = 0; i < sh_data->mtu; ++i)
        sh_data->frequency_axis[i] = (i - sh_data->mtu / 2.0) * sh_data->sample_rate / sh_data->mtu;

    int ofdm_symbol = sh_data->subcarrier + sh_data->cyclic_prefex;
    std::vector<int> bits;
    std::vector<int16_t> zadoff_chu_seq((sh_data->subcarrier + sh_data->cyclic_prefex) * 2);
    std::vector<std::complex<float>> rx_complex_remove_pss;
    std::vector<std::complex<float>> rx_complex_cfo;
    std::vector<std::complex<float>> rx_complex_remove_cp;
    std::vector<std::complex<float>> rx_complex_fft;
    std::vector<std::complex<float>> rx_complex_eq;
    std::vector<int> gen_bits;

    std::vector<float> signal_re(sh_data->mtu, 0);
    std::vector<float> signal_im(sh_data->mtu, 0);

    std::vector<float> zc_re(sh_data->subcarrier + sh_data->cyclic_prefex, 0);
    std::vector<float> zc_im(sh_data->subcarrier + sh_data->cyclic_prefex, 0);
    int zad_of_idx = 0;

    build_pss_zadoff_chu(zad_off_chu_context, sh_data);
    append_symbol(zad_off_chu_context, zadoff_chu_seq, sh_data->cyclic_prefex, 0);
    split_int16_t_to_float(zadoff_chu_seq.data(), zc_re.data(), zc_im.data(), zc_re.size());

    PRBS15 prbs15;

    while (sh_data->changed_quit == false)
    {
        if (!sh_data->changed_cont_time)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        if (sh_data->form)
        {
            int start_idx = 0;
            prbs15.state = 0xACE1;

            if (sh_data->changed_pss_symbols)
            {
                append_symbol(zad_off_chu_context, sh_data->tx_buffer, sh_data->cyclic_prefex, 0);
                start_idx = 2 * ofdm_symbol;
            }

            for (int start = start_idx; start <= sh_data->buffer - 2 * ofdm_symbol; start += 2 * ofdm_symbol)
            {
                build_ofdm_symbol(prbs15, context, sh_data);
                append_symbol(context, sh_data->tx_buffer, sh_data->cyclic_prefex, start);
            }

            sh_data->form = false;
            sh_data->read = true;
        }
        if (sh_data->dsp)
        {
            std::atomic_signal_fence(std::memory_order_seq_cst);
            auto start = std::chrono::steady_clock::now();
            std::atomic_signal_fence(std::memory_order_seq_cst);

            if (sh_data->get_zadoff_pos_loopback)
            {
                split_to_float(sh_data->rx_complex.data(), signal_re.data(), signal_im.data(), signal_re.size());
                zad_of_idx = zadoff_sync(signal_re.data(), signal_im.data(), signal_re.size(), zc_re.data(), zc_im.data(), zc_re.size(), sh_data->zadoff_corr_arr.data());
                sh_data->sync_pos = zad_of_idx;
                sh_data->get_zadoff_pos_loopback = false;
            }

            if (sh_data->get_zadoff_pos)
            {
                split_to_float(sh_data->rx_complex.data(), signal_re.data(), signal_im.data(), signal_re.size());
                zad_of_idx = zadoff_sync(signal_re.data(), signal_im.data(), signal_re.size(), zc_re.data(), zc_im.data(), zc_re.size(), sh_data->zadoff_corr_arr.data());
                sh_data->sync_pos = zad_of_idx;
            }

            remove_pss(sh_data, rx_complex_remove_pss);

            if (sh_data->cfo_cor)
                cfo_correction(rx_complex_remove_pss, sh_data);

            remove_cp(rx_complex_remove_pss, sh_data, rx_complex_remove_cp);
            decode(rx_complex_remove_cp, rx_complex_fft, context);
            std::vector<float> signal_eq_float(rx_complex_fft.size());

            if (sh_data->equal)
            {
                equalization(rx_complex_fft, sh_data, rx_complex_eq);
                if (signal_eq_float.size() != rx_complex_eq.size() * 2)
                    signal_eq_float.resize(rx_complex_eq.size() * 2);

                std::memcpy(signal_eq_float.data(), rx_complex_eq.data(), rx_complex_eq.size() * sizeof(std::complex<float>));

                sh_data->demaped_bits.resize(signal_eq_float.size());
                demap_symbols(signal_eq_float, sh_data->demaped_bits, sh_data->modul_type_TX);
            }

            if (sh_data->check_bits)
            {
                get_bits_to_check(sh_data);
                check_bits(sh_data->demaped_bits, sh_data);
            }

            sh_data->rx_complex_fft_gui = sh_data->equal ? rx_complex_eq : rx_complex_fft;

            spectrum(sh_data->rx_complex, sh_data->shifted_magnitude, sh_data->argument, context_spectre);

            std::atomic_signal_fence(std::memory_order_seq_cst);
            auto end = std::chrono::steady_clock::now();
            std::atomic_signal_fence(std::memory_order_seq_cst);

            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

            if (sh_data->debug)
            {
                sh_data->milisecs.push_back(duration.count() / 1e3);
                if (sh_data->milisecs.size() == 1920)
                    sh_data->milisecs.erase(sh_data->milisecs.begin());
            }

            sh_data->dsp = false;
            sh_data->form = true;
        }
    }
}

void run_gui(sharedData *sh_data)
{
    std::vector<float> bandwidths = {2e5f, 1e6f, 2e6f, 3e6f, 4e6f, 5e6f, 6e6f, 7e6f, 8e6f, 9e6f, 10e6f};
    int cur_rx_bandwidth = 1;
    int cur_tx_bandwidth = 1;

    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER);
    SDL_Window *window = SDL_CreateWindow(
        "GUI", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        1920, 1080, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    SDL_GL_SetSwapInterval(1);

    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO &io = ImGui::GetIO();

    static const ImVec4 plot_colors[4] = {
        ImVec4(0.26f, 0.65f, 0.98f, 1.00f),
        ImVec4(1.00f, 0.50f, 0.20f, 1.00f)
    };

    ImPlot::AddColormap("PlotPalete", plot_colors, 2);

    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init("#version 330");

    bool running = true;
    while (running)
    {
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT)
            {
                running = false;
            }
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();
        ImGui::DockSpaceOverViewport(0, nullptr, ImGuiDockNodeFlags_None);

        const float *raw_data = reinterpret_cast<const float *>(sh_data->rx_complex.data());
        const float *dsp_data = reinterpret_cast<const float *>(sh_data->rx_complex_fft_gui.data());

        ImPlot::PushColormap("PlotPalete");
        if (ImGui::Begin("Scatter Raw"))
        {
            if (ImPlot::BeginPlot("Raw Samples", ImVec2(ImGui::GetContentRegionAvail())))
            {
                ImPlot::PlotScatter("I/Q",
                                    raw_data,
                                    raw_data + 1,
                                    sh_data->rx_complex.size(), 0, 0, sizeof(std::complex<float>));
                ImPlot::EndPlot();
            }
        }
        ImGui::End();

        if (ImGui::Begin("Scatter FFT"))
        {
            if (ImPlot::BeginPlot("Samples After FFT", ImVec2(ImGui::GetContentRegionAvail())))
            {
                ImPlot::PlotScatter("I/Q",
                                    dsp_data,
                                    dsp_data + 1,
                                    sh_data->rx_complex_fft_gui.size(), 0, 0, sizeof(std::complex<float>));
                ImPlot::EndPlot();
            }
        }
        ImGui::End();

        if (ImGui::Begin("Plot Raw"))
        {
            if (ImPlot::BeginPlot("Raw I/Q samples", ImVec2(ImGui::GetContentRegionAvail())))
            {
                ImPlot::PlotLine("I", raw_data, sh_data->rx_complex.size(), 1.0, 0, 0, 0, sizeof(std::complex<float>));
                ImPlot::PlotLine("Q", raw_data + 1, sh_data->rx_complex.size(), 1.0, 0, 0, 0, sizeof(std::complex<float>));
                ImPlot::EndPlot();
            }
        }
        ImGui::End();

        if (ImGui::Begin("Plot FFT"))
        {
            if (ImPlot::BeginPlot("I/Q Samples After FFT", ImVec2(ImGui::GetContentRegionAvail())))
            {
                ImPlot::PlotLine("I", dsp_data, sh_data->rx_complex_fft_gui.size(), 1.0, 0, 0, 0, sizeof(std::complex<float>));
                ImPlot::PlotLine("Q", dsp_data + 1, sh_data->rx_complex_fft_gui.size(), 1.0, 0, 0, 0, sizeof(std::complex<float>));
                ImPlot::EndPlot();
            }
        }
        ImGui::End();

        if (ImGui::Begin("Argument"))
        {
            if (ImPlot::BeginPlot("Signal Argument", ImVec2(ImGui::GetContentRegionAvail())))
            {
                ImPlot::PlotLine("Argument", sh_data->frequency_axis.data(), sh_data->argument.data(), sh_data->argument.size());
                ImPlot::EndPlot();
            }
        }
        ImGui::End();

        if (ImGui::Begin("Magnitude"))
        {
            if (ImPlot::BeginPlot("Signal Magnitude", ImVec2(ImGui::GetContentRegionAvail())))
            {
                ImPlot::PlotLine("Magnitude", sh_data->frequency_axis.data(), sh_data->shifted_magnitude.data(), sh_data->shifted_magnitude.size());
                ImPlot::EndPlot();
            }
        }
        ImGui::End();

        if (sh_data->debug)
        {
            const float *debug_latency = reinterpret_cast<const float *>(sh_data->milisecs.data());
            const float *zadoff_chu = reinterpret_cast<const float *>(sh_data->zadoff_corr_arr.data());
            const float *cfo_cor = reinterpret_cast<const float *>(sh_data->cfo_offset.data());
            const float *dem_bits = reinterpret_cast<const float *>(sh_data->demaped_bits.data());

            if (ImGui::Begin("Latency"))
            {
                if (ImPlot::BeginPlot("Latency", ImVec2(ImGui::GetContentRegionAvail())))
                {
                    ImPlot::PlotLine("Latency", debug_latency, sh_data->milisecs.size());
                    ImPlot::EndPlot();
                }
            }
            ImGui::End();

            if (ImGui::Begin("Zadoff-Chu"))
            {
                if (ImPlot::BeginPlot("Zadoff-Chu Correlation Array", ImVec2(ImGui::GetContentRegionAvail())))
                {
                    ImPlot::PlotLine("Zadoff-Chu", zadoff_chu, sh_data->zadoff_corr_arr.size());
                    ImPlot::EndPlot();
                }
            }
            ImGui::End();

            if (ImGui::Begin("CFO"))
            {
                if (ImPlot::BeginPlot("CFO Correction Array", ImVec2(ImGui::GetContentRegionAvail())))
                {
                    ImPlot::PlotLine("CFO", cfo_cor, sh_data->cfo_offset.size());
                    ImPlot::EndPlot();
                }
            }
            ImGui::End();

            if (ImGui::Begin("Demapped Bits"))
            {
                if (ImPlot::BeginPlot("Demapped Bits", ImVec2(ImGui::GetContentRegionAvail())))
                {
                    ImPlot::PlotLine("RX", dem_bits, sh_data->demaped_bits.size());
                    ImPlot::PlotLine("TX", sh_data->bits.data(), sh_data->bits.size());
                    ImPlot::EndPlot();
                }
            }
            ImGui::End();

            if (ImGui::Begin("Error Counter"))
                ImGui::Text("Error Counter: %d", sh_data->err_cnt);
            ImGui::End();
        }
        ImPlot::PopColormap();

        if (ImGui::BeginMainMenuBar())
        {
            if (ImGui::BeginMenu("Control Panel"))
            {
                ImGui::SeparatorText("Processing Blocks");

                const char *label_time = sh_data->changed_cont_time ? "Programm | Running" : "Programm | Stopped";
                const char *sdr_mode = sh_data->changed_send ? "SDR Mode | Transmission" : "SDR Mode | Receiving";
                const char *pss_mode = sh_data->changed_pss_symbols ? "PSS Symbol [ON]" : "PSS Symbol [OFF]";
                const char *zadoff_chu = sh_data->get_zadoff_pos ? "Direct Mode [ON]" : "Direct Mode [OFF]";
                const char *cfo_correct = sh_data->cfo_cor ? "CFO Correction [ON]" : "CFO Correction [OFF]";
                const char *equal_mode = sh_data->equal ? "Equalization [ON]" : "Equalization [OFF]";
                const char *check_bit = sh_data->check_bits ? "Bits Check [ON]" : "Bits Check [OFF]";
                const char *modulation_type = nullptr;

                if (ImGui::Button(label_time, ImVec2(ImGui::GetContentRegionAvail().x, 0.f)))
                    sh_data->changed_cont_time = !sh_data->changed_cont_time;

                if (ImGui::Button(sdr_mode, ImVec2(ImGui::GetContentRegionAvail().x, 0.f)))
                    sh_data->changed_send = !sh_data->changed_send;

                if (ImGui::Button(pss_mode, ImVec2(ImGui::GetContentRegionAvail().x, 0.f)))
                    sh_data->changed_pss_symbols = !sh_data->changed_pss_symbols;

                if (ImGui::Button(check_bit, ImVec2(ImGui::GetContentRegionAvail().x / 2, 0.f)))
                    sh_data->check_bits = !sh_data->check_bits;

                ImGui::SameLine();

                if (ImGui::Button("Reset Error", ImVec2(ImGui::GetContentRegionAvail().x, 0.f)))
                    sh_data->err_cnt = 0;

                ImGui::SeparatorText("ZadOff-Chu");
                if (ImGui::Button("Loopback Mode", ImVec2(ImGui::GetContentRegionAvail().x, 0.f)))
                    sh_data->get_zadoff_pos_loopback = !sh_data->get_zadoff_pos_loopback;
                if (ImGui::Button(zadoff_chu, ImVec2(ImGui::GetContentRegionAvail().x, 0.f)))
                    sh_data->get_zadoff_pos = !sh_data->get_zadoff_pos;
                ImGui::InputInt("Sync Pos", &sh_data->sync_pos, 1, 1e1);
                ImGui::InputInt("U Value ", &sh_data->zadoff_chu_u, 1, 10);

                ImGui::SeparatorText("DSP Module");
                if (ImGui::Button(cfo_correct, ImVec2(ImGui::GetContentRegionAvail().x, 0.f)))
                    sh_data->cfo_cor = !sh_data->cfo_cor;

                if (ImGui::Button(equal_mode, ImVec2(ImGui::GetContentRegionAvail().x, 0.f)))
                    sh_data->equal = !sh_data->equal;

                ImGui::SeparatorText("Pre Modulation");
                switch (sh_data->modul_type_TX)
                {
                case ModulationType::BPSK:
                    modulation_type = "Modulation: BPSK";
                    break;
                case ModulationType::QPSK:
                    modulation_type = "Modulation: QPSK";
                    break;
                case ModulationType::QAM16:
                    modulation_type = "Modulation: QAM16";
                    break;
                }

                ImGui::SetNextItemWidth(-FLT_MIN);
                if (ImGui::BeginCombo("##Modulation Type", modulation_type))
                {
                    if (ImGui::Selectable("BPSK", sh_data->modul_type_TX == ModulationType::BPSK))
                    {
                        sh_data->modul_type_TX = ModulationType::BPSK;
                        sh_data->changed_modulation_type = true;
                    }
                    if (ImGui::Selectable("QPSK", sh_data->modul_type_TX == ModulationType::QPSK))
                    {
                        sh_data->modul_type_TX = ModulationType::QPSK;
                        sh_data->changed_modulation_type = true;
                    }
                    if (ImGui::Selectable("QAM16", sh_data->modul_type_TX == ModulationType::QAM16))
                    {
                        sh_data->modul_type_TX = ModulationType::QAM16;
                        sh_data->changed_modulation_type = true;
                    }
                    ImGui::EndCombo();
                }

                ImGui::InputInt("Cycle Prefix", &sh_data->cyclic_prefex, 1);
                ImGui::InputInt("Subcarrier", &sh_data->subcarrier, 1);

                ImGui::SeparatorText("SDR Configuration");
                ImGui::SetNextItemWidth(-FLT_MIN);
                if (ImGui::BeginCombo("##SDR Device", sh_data->device.c_str()))
                {
                    for (const auto &dev : sh_data->devices)
                    {
                        bool is_selected = (sh_data->device == dev);
                        if (ImGui::Selectable(dev.c_str(), is_selected))
                            sh_data->device = dev;
                        if (is_selected)
                            ImGui::SetItemDefaultFocus();
                    }
                    ImGui::EndCombo();
                }

                if (ImGui::DragFloat("RX Gain", &sh_data->rx_gain, 0.25f, 0.f, 73.f))
                    sh_data->changed_rx_gain = true;

                if (ImGui::DragFloat("TX Gain", &sh_data->tx_gain, 0.25f, 0.f, 89.f))
                    sh_data->changed_tx_gain = true;

                if (ImGui::InputFloat("RX Frequency", &sh_data->rx_frequency, 1e2, 1e3))
                    sh_data->changed_rx_freq = true;

                if (ImGui::InputFloat("TX Frequency", &sh_data->tx_frequency, 1e2, 1e3))
                    sh_data->changed_tx_freq = true;

                if (ImGui::SliderInt("RX Bandwidth", &cur_rx_bandwidth, 0, bandwidths.size() - 1, std::to_string(bandwidths[cur_rx_bandwidth]).c_str()))
                {
                    sh_data->rx_bandwidth = bandwidths[cur_rx_bandwidth];
                    sh_data->changed_rx_bandwidth = true;
                }

                if (ImGui::SliderInt("TX Bandwidth", &cur_tx_bandwidth, 0, bandwidths.size() - 1, std::to_string(bandwidths[cur_tx_bandwidth]).c_str()))
                {
                    sh_data->tx_bandwidth = bandwidths[cur_tx_bandwidth];
                    sh_data->changed_tx_bandwidth = true;
                }

                if (ImGui::InputFloat("Sample Rate", &sh_data->sample_rate, 1e5, 1e6))
                    sh_data->changed_sample_rate = true;

                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Debug"))
            {
                if (ImGui::Button("Debug Mode"))
                    sh_data->debug = !sh_data->debug;
                ImGui::EndMenu();
            }

            float window_width = ImGui::GetWindowWidth();
            ImGui::SameLine(window_width - ImGui::CalcTextSize("FPS: 0000 (0000.000 ms)").x);
            ImGui::Text("FPS: %.f (%.3f ms)", io.Framerate, 1000.0f / io.Framerate);

            ImGui::EndMainMenuBar();
        }

        ImGui::Render();
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);
    }
    sh_data->changed_quit = true;
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();
}

int main()
{
    sharedData sd(1920);

    std::thread gui_thread(run_gui, &sd);
    std::thread backend_thread(run_backend, &sd);
    std::thread dsp_thread(run_dsp, &sd);

    dsp_thread.join();
    backend_thread.join();
    gui_thread.join();

    return 0;
}
