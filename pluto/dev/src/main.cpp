#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <SoapySDR/Types.h>
#include <algorithm>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <iterator>
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
#include <fstream>
#include "imgui.h"
#include "implot.h"
#include "backends/imgui_impl_opengl3.h"
#include "backends/imgui_impl_sdl2.h"
#include "modulation.hpp"
#include "sdr.hpp"

constexpr long long TIMEOUT = 400000;
constexpr long long TX_DELAY = 4000000;

struct sharedData
{
    ModulationType modul_type_TX = ModulationType::QPSK;
    ModulationType modul_type_RX = ModulationType::QPSK;
    std::string device;
    std::vector<std::string> devices;
    std::vector<std::complex<double>> rx_complex;
    std::vector<std::complex<double>> rx_complex_fft_gui;
    std::vector<int16_t> tx_buffer;
    std::vector<double> shifted_magnitude;
    std::vector<double> argument;
    std::vector<double> frequency_axis;
    std::vector<double> M_arra;
    std::atomic<bool> form = true;
    std::atomic<bool> read = false;
    std::atomic<bool> dsp = false;
    bool changed_send = false;
    bool changed_quit = false;
    bool changed_rx_gain = false;
    bool changed_tx_gain = false;
    bool changed_rx_freq = false;
    bool changed_tx_freq = false;
    bool changed_sample_rate = false;
    bool changed_rx_bandwidth = false;
    bool changed_tx_bandwidth = false;
    bool changed_modulation_type = false;
    bool changed_pss_symbols = false;
    bool changed_cont_time = true;
    bool get_schmiddle_pos = false;
    bool remove_pss_symbol = false;
    bool schmiddle_sync = false;
    float rx_gain = 20.f;
    float tx_gain = 80.f;
    double rx_frequency = 777e6;
    double tx_frequency = 777e6;
    double sample_rate = 1.92e6;
    float rx_bandwidth = 1e6;
    float tx_bandwidth = 1e6;
    int cyclic_prefex = 32;
    int subcarrier = 128;
    int sync_pos = 0;
    int mtu = 1920;
    int buffer = 3840;
    int zadoff_chu_u = 3;

    sharedData(size_t rx_mtu)
    {
        tx_buffer.resize(rx_mtu * 2, 0);
        rx_complex.resize(rx_mtu, 0);
        M_arra.resize(rx_mtu, 0);
        rx_complex_fft_gui.resize(rx_mtu, 0);
        shifted_magnitude.resize(rx_mtu, 0);
        argument.resize(rx_mtu, 0);
        frequency_axis.resize(rx_mtu, 0);
    }
};

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

            size_t sr = SoapySDRDevice_readStream(sdr.sdr, sdr.rxStream, rx_buffs, sdr.rx_mtu, &flags, &timeNs, TIMEOUT);
            if (sr != sdr.tx_mtu)
                std::cout << "[ERROR] Read stream | Error code: " << sr << "\n";

            for (int i = 0; i < sh_data->mtu; ++i)
                sh_data->rx_complex[i] = std::complex<double>(sdr.rx_buffer[2 * i], sdr.rx_buffer[2 * i + 1]);

            long long tx_time = timeNs + TX_DELAY;
            flags = SOAPY_SDR_HAS_TIME;

            if (sh_data->changed_send)
            {
                size_t st = SoapySDRDevice_writeStream(sdr.sdr, sdr.txStream, tx_buffs, sdr.tx_mtu, &flags, tx_time, TIMEOUT);
                if (st != sdr.tx_mtu)
                    std::cout << "[ERROR] Write stream | Error code: " << st << "\n";
            }
            sh_data->read = false;
            sh_data->dsp = true;
        }

        if (sh_data->changed_rx_gain)
        {
            if (int err; (err = SoapySDRDevice_setGain(sdr.sdr, SOAPY_SDR_RX, 0, static_cast<double>(sh_data->rx_gain))) != 0)
                std::cout << "[ERROR] Set RX gain | Error code: " << err << "\n";
            sh_data->changed_rx_gain = false;
        }

        if (sh_data->changed_tx_gain)
        {
            if (int err; (err = SoapySDRDevice_setGain(sdr.sdr, SOAPY_SDR_TX, 0, static_cast<double>(sh_data->tx_gain))) != 0)
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
            if (int err; (err = SoapySDRDevice_setBandwidth(sdr.sdr, SOAPY_SDR_RX, 0, static_cast<double>(sh_data->rx_bandwidth))) != 0)
                std::cout << "[ERROR] Set RX bandwidth | Error code: " << err << "\n";
            sh_data->changed_rx_bandwidth = false;
        }

        if (sh_data->changed_tx_bandwidth)
        {
            if (int err; (err = SoapySDRDevice_setBandwidth(sdr.sdr, SOAPY_SDR_TX, 0, static_cast<double>(sh_data->tx_bandwidth))) != 0)
                std::cout << "[ERROR] Set TX bandwidth | Error code: " << err << "\n";
            sh_data->changed_tx_bandwidth = false;
        }
    }
}

void run_dsp(sharedData *sh_data)
{
    schmiddle_state schm_state;

    fftw_complex *in_build_ofdm = static_cast<fftw_complex *>(fftw_malloc(sizeof(fftw_complex) * sh_data->subcarrier));
    fftw_complex *out_build_ofdm = static_cast<fftw_complex *>(fftw_malloc(sizeof(fftw_complex) * sh_data->subcarrier));

    for (int i = 0; i < sh_data->mtu; ++i)
    {
        sh_data->frequency_axis[i] = (i - sh_data->mtu / 2.0) * sh_data->sample_rate / sh_data->mtu;
    }

    int ofdm_symbol = sh_data->subcarrier + sh_data->cyclic_prefex;
    std::deque<int> bit_fifo;

    std::vector<std::complex<double>> rx_complex_remove_pss;
    std::vector<std::complex<double>> rx_complex_remove_cp;
    std::vector<std::complex<double>> rx_complex_fft;

    while (bit_fifo.size() < 1)
        bit_fifo.push_back(rand() & 1);

    while (sh_data->changed_quit == false)
    {
        if (!sh_data->changed_cont_time)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        if (sh_data->form)
        {
            if (sh_data->changed_pss_symbols)
            {
                build_pss_zadoff_chu(in_build_ofdm, out_build_ofdm, sh_data->subcarrier, sh_data->zadoff_chu_u);
                append_symbol(out_build_ofdm, sh_data->tx_buffer, sh_data->subcarrier, sh_data->cyclic_prefex, 0);

                for (int start = 2 * (ofdm_symbol); start < (sh_data->buffer - ofdm_symbol); start += 2 * (ofdm_symbol))
                {
                    build_ofdm_symbol(bit_fifo, in_build_ofdm, out_build_ofdm, sh_data->modul_type_TX, sh_data->subcarrier);
                    append_symbol(out_build_ofdm, sh_data->tx_buffer, sh_data->subcarrier, sh_data->cyclic_prefex, start);
                }

                sh_data->form = false;
                sh_data->read = true;
            }
            else
            {
                for (int start = 0; start < (sh_data->buffer - sh_data->subcarrier + sh_data->cyclic_prefex); start += 2 * (sh_data->subcarrier + sh_data->cyclic_prefex))
                {
                    build_ofdm_symbol(bit_fifo, in_build_ofdm, out_build_ofdm, sh_data->modul_type_TX, sh_data->subcarrier);
                    append_symbol(out_build_ofdm, sh_data->tx_buffer, sh_data->subcarrier, sh_data->cyclic_prefex, start);
                }

                sh_data->form = false;
                sh_data->read = true;
            }
        }
        if (sh_data->dsp)
        {
            if (sh_data->get_schmiddle_pos)
            {
                schm_state = schmidl_sync(sh_data->rx_complex, sh_data->subcarrier);
                sh_data->sync_pos = schm_state.M;
                sh_data->M_arra = schm_state.M_arr;
                sh_data->get_schmiddle_pos = false;
            }

            remove_pss(sh_data->rx_complex, sh_data->cyclic_prefex, sh_data->subcarrier, sh_data->sync_pos, rx_complex_remove_pss);
            remove_cp(rx_complex_remove_pss, sh_data->cyclic_prefex, sh_data->subcarrier, rx_complex_remove_cp);
            decode(rx_complex_remove_cp, sh_data->subcarrier, rx_complex_fft);

            sh_data->rx_complex_fft_gui.clear();
            for (size_t i = 0; i < rx_complex_fft.size(); ++i)
                sh_data->rx_complex_fft_gui.push_back(rx_complex_fft[i]);

            spectrum(sh_data->rx_complex, sh_data->shifted_magnitude, sh_data->argument);

            sh_data->dsp = false;
            sh_data->form = true;
        }
    }

    fftw_free(in_build_ofdm);
    fftw_free(out_build_ofdm);
}

void run_gui(sharedData *sh_data)
{
    std::vector<float> bandwidths = {2e5f, 1e6f, 2e6f, 3e6f, 4e6f, 5e6f, 6e6f, 7e6f, 8e6f, 9e6f, 10e6f};
    int cur_rx_bandwidth = 1;
    int cur_tx_bandwidth = 1;

    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER);
    SDL_Window *window = SDL_CreateWindow(
        "Backend start", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        1920, 1080, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    SDL_GL_SetSwapInterval(0);

    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    ImGuiStyle &style = ImGui::GetStyle();
    int wind_pad_x = 2;
    int wind_pad_y = 2;
    int frame_pad_x = 5;
    int frame_pad_y = 4;
    int item_space_x = 10;
    int item_space_y = 6;
    int item_space_x_inn = 2;
    int item_space_y_inn = 4;

    style.WindowRounding = 0.f;
    style.WindowBorderSize = 0.0f;
    style.WindowPadding = ImVec2(wind_pad_x, wind_pad_y);

    style.FrameRounding = 3.f;
    style.FrameBorderSize = 1.0f;
    style.FramePadding = ImVec2(frame_pad_x, frame_pad_y);

    style.ChildRounding = 0.f;
    style.ScrollbarRounding = 0.f;
    style.TabRounding = 0.f;
    style.PopupBorderSize = 0.0f;
    style.ItemSpacing = ImVec2(item_space_x, item_space_y);
    style.ItemInnerSpacing = ImVec2(item_space_x_inn, item_space_y_inn);

    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
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
        
        if (ImGui::Begin("Scatter Raw"))
        {
            if (ImPlot::BeginPlot("Raw Samples"))
            {
                ImPlot::PlotScatter("I/Q",
                                    reinterpret_cast<double *>(sh_data->rx_complex.data()),
                                    reinterpret_cast<double *>(sh_data->rx_complex.data()) + 1,
                                    sh_data->rx_complex.size(), 0, 0, sizeof(std::complex<double>));
                ImPlot::EndPlot();
            }
        }
        ImGui::End();

        if (ImGui::Begin("Scatter FFT"))
        {
            if (ImPlot::BeginPlot("Samples After FFT"))
            {
                ImPlot::PlotScatter("I/Q",
                                    reinterpret_cast<double *>(sh_data->rx_complex_fft_gui.data()),
                                    reinterpret_cast<double *>(sh_data->rx_complex_fft_gui.data()) + 1,
                                    sh_data->rx_complex_fft_gui.size(), 0, 0, sizeof(std::complex<double>));
                ImPlot::EndPlot();
            }
        }
        ImGui::End();

        if (ImGui::Begin("Plot Raw"))
        {
            if (ImPlot::BeginPlot("Raw I/Q samples"))
            {
                ImPlot::PlotLine("I", reinterpret_cast<const double *>(sh_data->rx_complex.data()), sh_data->rx_complex.size(), 1.0, 0, 0, 0, sizeof(std::complex<double>));
                ImPlot::PlotLine("Q", reinterpret_cast<const double *>(sh_data->rx_complex.data()) + 1, sh_data->rx_complex.size(), 1.0, 0, 0, 0, sizeof(std::complex<double>));
                ImPlot::EndPlot();
            }
        }
        ImGui::End();

        if (ImGui::Begin("Plot FFT"))
        {
            if (ImPlot::BeginPlot("I/Q Samples After FFT"))
            {
                ImPlot::PlotLine("I", reinterpret_cast<const double *>(sh_data->rx_complex_fft_gui.data()), sh_data->rx_complex_fft_gui.size(), 1.0, 0, 0, 0, sizeof(std::complex<double>));
                ImPlot::PlotLine("Q", reinterpret_cast<const double *>(sh_data->rx_complex_fft_gui.data()) + 1, sh_data->rx_complex_fft_gui.size(), 1.0, 0, 0, 0, sizeof(std::complex<double>));
                ImPlot::EndPlot();
            }
        }
        ImGui::End();

        if (ImGui::Begin("M Corr"))
        {
            if (ImPlot::BeginPlot("M Correlation Array"))
            {
                ImPlot::PlotLine("Correlation Array", sh_data->M_arra.data(), sh_data->M_arra.size());
                ImPlot::EndPlot();
            }
        }
        ImGui::End();

        if (ImGui::Begin("Magnitude"))
        {
            if (ImPlot::BeginPlot("Signal Magnitude"))
            {
                ImPlot::PlotLine("Magnitude", sh_data->frequency_axis.data(), sh_data->shifted_magnitude.data(), sh_data->shifted_magnitude.size());
                ImPlot::EndPlot();
            }
        }
        ImGui::End();

        if (ImGui::Begin("Argument"))
        {
            if (ImPlot::BeginPlot("Signal Argument"))
            {
                ImPlot::PlotLine("Argument", sh_data->frequency_axis.data(), sh_data->argument.data(), sh_data->argument.size());
                ImPlot::EndPlot();
            }
        }
        ImGui::End();

        if (ImGui::BeginMainMenuBar())
        {
            if (ImGui::BeginMenu("Control Panel"))
            {
                ImGui::SeparatorText("Processing Blocks");

                const char *label_time = sh_data->changed_cont_time ? "Programm | Running" : "Programm | Stopped";
                if (ImGui::Button(label_time, ImVec2(ImGui::GetContentRegionAvail().x, 0.f)))
                    sh_data->changed_cont_time = !sh_data->changed_cont_time;

                const char *mode = sh_data->changed_send ? "SDR Mode | Transmission" : "SDR Mode | Receiving";
                if (ImGui::Button(mode, ImVec2(ImGui::GetContentRegionAvail().x, 0.f)))
                    sh_data->changed_send = !sh_data->changed_send;

                ImGui::Checkbox("PSS Symbols", &sh_data->changed_pss_symbols);
                ImGui::SeparatorText("Schmiddle Cox");
                if (ImGui::Button("Get Schmiddle Sync Pos"))
                    sh_data->get_schmiddle_pos = !sh_data->get_schmiddle_pos;
                ImGui::SameLine();
                ImGui::Text(": %d", sh_data->sync_pos);
                ImGui::InputInt("Sync Pos", &sh_data->sync_pos, 1, 1e1);
                ImGui::Checkbox("Remove PSS", &sh_data->remove_pss_symbol);
                ImGui::Checkbox("Schmiddle Cox Sync", &sh_data->schmiddle_sync);

                ImGui::SeparatorText("Pre Modulation");
                const char *rx_mod_type = nullptr;
                const char *tx_mod_type = nullptr;

                switch (sh_data->modul_type_RX)
                {
                case ModulationType::BPSK:
                    rx_mod_type = "RX Modulation BPSK";
                    break;
                case ModulationType::QPSK:
                    rx_mod_type = "RX Modulation QPSK";
                    break;
                case ModulationType::QAM16:
                    rx_mod_type = "RX Modulation QAM16";
                    break;
                }
                switch (sh_data->modul_type_TX)
                {
                case ModulationType::BPSK:
                    tx_mod_type = "TX Modulation BPSK";
                    break;
                case ModulationType::QPSK:
                    tx_mod_type = "TX Modulation QPSK";
                    break;
                case ModulationType::QAM16:
                    tx_mod_type = "TX Modulation QAM16";
                    break;
                }

                if (ImGui::BeginCombo("RX Modulation Type", rx_mod_type))
                {
                    if (ImGui::Selectable("BPSK", sh_data->modul_type_RX == ModulationType::BPSK))
                        sh_data->modul_type_RX = ModulationType::BPSK;
                    if (ImGui::Selectable("QPSK", sh_data->modul_type_RX == ModulationType::QPSK))
                        sh_data->modul_type_RX = ModulationType::QPSK;
                    if (ImGui::Selectable("QAM16", sh_data->modul_type_RX == ModulationType::QAM16))
                        sh_data->modul_type_RX = ModulationType::QAM16;
                    ImGui::EndCombo();
                }

                if (ImGui::BeginCombo("TX Modulation Type", tx_mod_type))
                {
                    if (ImGui::Selectable("BPSK", sh_data->modul_type_TX == ModulationType::BPSK))
                        sh_data->modul_type_TX = ModulationType::BPSK;
                    if (ImGui::Selectable("QPSK", sh_data->modul_type_TX == ModulationType::QPSK))
                        sh_data->modul_type_TX = ModulationType::QPSK;
                    if (ImGui::Selectable("QAM16", sh_data->modul_type_TX == ModulationType::QAM16))
                        sh_data->modul_type_TX = ModulationType::QAM16;
                    ImGui::EndCombo();
                }

                ImGui::InputInt("Cycle Prefix", &sh_data->cyclic_prefex, 1);
                ImGui::InputInt("Subcarrier", &sh_data->subcarrier, 1);

                ImGui::SeparatorText("SDR Configuration");

                if (ImGui::BeginCombo("SDR Device", sh_data->device.c_str()))
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

                if (ImGui::InputDouble("RX Frequency", &sh_data->rx_frequency, 1e2, 1e3))
                    sh_data->changed_rx_freq = true;

                if (ImGui::InputDouble("TX Frequency", &sh_data->tx_frequency, 1e2, 1e3))
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

                if (ImGui::InputDouble("Sample Rate", &sh_data->sample_rate, 1e5, 1e6))
                    sh_data->changed_sample_rate = true;

                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("View"))
            {
                ImGui::SeparatorText("Theme Colors");
                if (ImGui::Button("Light Theme"))
                    ImGui::StyleColorsLight();
                ImGui::SameLine();
                if (ImGui::Button("Dark Theme"))
                    ImGui::StyleColorsDark();
                ImGui::SameLine();
                if (ImGui::Button("Classic Theme"))
                    ImGui::StyleColorsClassic();

                ImGui::SeparatorText("Window Settings");
                ImGui::InputFloat("Window Rounding", &style.WindowRounding, 1);
                ImGui::InputFloat("Window Border Size", &style.WindowBorderSize, 1);
                ImGui::InputInt("Window Padding X", &wind_pad_x, 1);
                ImGui::InputInt("Window Padding Y", &wind_pad_y, 1);
                style.WindowPadding = ImVec2(wind_pad_x, wind_pad_y);

                ImGui::SeparatorText("Frame Settings");
                ImGui::InputFloat("Frame Rounding", &style.FrameRounding, 1);
                ImGui::InputFloat("Frame Border Size", &style.FrameBorderSize, 1);
                ImGui::InputInt("Frame Padding X", &frame_pad_x, 1);
                ImGui::InputInt("Frame Padding Y", &frame_pad_y, 1);
                style.FramePadding = ImVec2(frame_pad_x, frame_pad_y);

                ImGui::SeparatorText("Child Settings");
                ImGui::InputFloat("Child Rounding", &style.ChildRounding, 1);

                ImGui::SeparatorText("Scrollbar Settings");
                ImGui::InputFloat("Scrollbar Rounding", &style.ScrollbarRounding, 1);

                ImGui::SeparatorText("Tab Settings");
                ImGui::InputFloat("Tab Rounding", &style.TabRounding, 1);

                ImGui::SeparatorText("Popup Settings");
                ImGui::InputFloat("Popup Border Size", &style.PopupBorderSize, 1);

                ImGui::SeparatorText("Item Settings");
                ImGui::InputInt("Item Spacing X", &item_space_x, 1);
                ImGui::InputInt("Item Spacing Y", &item_space_y, 1);
                style.ItemSpacing = ImVec2(item_space_x, item_space_y);

                ImGui::SeparatorText("Item Inner Settings");
                ImGui::InputInt("Item Inner Spacing X", &item_space_x_inn, 1);
                ImGui::InputInt("Item Inner Spacing Y", &item_space_y_inn, 1);
                style.ItemInnerSpacing = ImVec2(item_space_x_inn, item_space_y_inn);
                ImGui::EndMenu();
            }
            float window_width = ImGui::GetWindowWidth();
            ImGui::SameLine(window_width - ImGui::CalcTextSize("FPS: 0000 (0000.000 ms)").x);
            ImGui::Text("FPS: %.f (%.3f ms)", io.Framerate, 1000.0f / io.Framerate);
            ImGui::EndMainMenuBar();
        }

        // ImGui::ShowDemoWindow();

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
