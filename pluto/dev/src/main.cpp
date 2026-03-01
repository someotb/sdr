#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <SoapySDR/Types.h>
#include <algorithm>
#include <complex>
#include <cstddef>
#include <cstdint>
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
#include <cmath>
#include <iostream>
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
    int upsample_koef = 10;
    std::string device;
    std::vector<std::string> devices;
    std::vector<double> real_p;
    std::vector<double> imag_p;
    std::vector<double> shifted_magnitude;
    std::vector<double> argument;
    std::vector<double> frequency_axis;
    bool send = false;
    bool quit = false;
    bool changed_rx_gain = false;
    bool changed_tx_gain = false;
    bool changed_rx_freq = false;
    bool changed_tx_freq = false;
    bool changed_sample_rate = false;
    bool changed_rx_bandwidth = false;
    bool changed_tx_bandwidth = false;
    bool changed_modulation_type = false;
    bool cont_time = true;
    float rx_gain = 20.f;
    float tx_gain = 80.f;
    double rx_frequency = 777e6;
    double tx_frequency = 777e6;
    double sample_rate = 1.92e6;
    float rx_bandwidth = 1e6;
    float tx_bandwidth = 1e6;
    int CYCLIC_PREFEX = 8;
    int SUBCARRIER = 128;

    sharedData(size_t rx_mtu) {
        real_p.resize(rx_mtu, 0);
        imag_p.resize(rx_mtu, 0);
        shifted_magnitude.resize(rx_mtu, 0);
        argument.resize(rx_mtu, 0);
        frequency_axis.resize(rx_mtu, 0);
    }
};

void run_backend(sharedData *sh_data) {
    size_t length;
    SoapySDRKwargs* results = SoapySDRDevice_enumerate(nullptr, &length);

    sh_data->devices.clear();

    for (size_t i = 0; i < length / 2; ++i) {
        std::string label = static_cast<std::string>(results[i].vals[3]);
        sh_data->devices.push_back(label);
    }

    if (!sh_data->devices.empty()) sh_data->device = sh_data->devices[0];

    SoapySDRKwargsList_clear(results, length);

    SDRDevice sdr(sh_data->device.c_str());

    fftw_complex* in1 = reinterpret_cast<fftw_complex*> (fftw_malloc(sizeof(fftw_complex) * sdr.rx_mtu));
    fftw_complex* out1 = reinterpret_cast<fftw_complex*> (fftw_malloc(sizeof(fftw_complex) * sdr.rx_mtu));

    for (size_t i = 0; i < sdr.rx_mtu; ++i) {
        sh_data->frequency_axis[i] = (i - sdr.rx_mtu / 2.0) * sh_data->sample_rate / sdr.rx_mtu;
    }

    std::deque<int> bit_fifo;
    std::vector<double> tx_buffer_double(2 * sdr.tx_mtu, 0);
    std::vector<double> rx_buffer_double(2 * sdr.rx_mtu, 0);
    std::vector<double> magnitude(sdr.rx_mtu, 0);

    fftw_complex* in = nullptr;
    fftw_complex* out = nullptr;
    int current_subcarriers = 0;

    while (bit_fifo.size() < 100000)
        bit_fifo.push_back(rand() & 1);

    while (sh_data->quit == false) {
        if (!sh_data->cont_time) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        if (sh_data->SUBCARRIER != current_subcarriers) {
            if (in) fftw_free(in);
            if (out) fftw_free(out);

            in = reinterpret_cast<fftw_complex*>(fftw_malloc(sizeof(fftw_complex) * sh_data->SUBCARRIER));
            out = reinterpret_cast<fftw_complex*>(fftw_malloc(sizeof(fftw_complex) * sh_data->SUBCARRIER));
            current_subcarriers = sh_data->SUBCARRIER;
        }

        sdr.tx_buffer.clear();
        sdr.tx_buffer.reserve(2 * sdr.tx_mtu);

        while (sdr.tx_buffer.size() < 2 * sdr.tx_mtu)
        {
            build_ofdm_symbol(bit_fifo, in, out, sh_data->modul_type_TX, sh_data->SUBCARRIER);
            append_symbol(out, sdr.tx_buffer, sh_data->SUBCARRIER, sh_data->CYCLIC_PREFEX);
        }

        void *rx_buffs[] = {sdr.rx_buffer.data()};
        void *tx_buffs[] = {sdr.tx_buffer.data()};

        int flags = 0;
        long long timeNs = 0;

        int sr = SoapySDRDevice_readStream(sdr.sdr, sdr.rxStream, rx_buffs, sdr.rx_mtu, &flags, &timeNs, TIMEOUT);
        (void)sr;


        long long tx_time = timeNs + TX_DELAY;
        flags = SOAPY_SDR_HAS_TIME;

        if (sh_data->send) {
            int st = SoapySDRDevice_writeStream(sdr.sdr, sdr.txStream, tx_buffs, sdr.tx_mtu, &flags, tx_time, TIMEOUT);
            (void)st;
        }

        if (sh_data->changed_rx_gain) {
            if (int err; (err = SoapySDRDevice_setGain(sdr.sdr, SOAPY_SDR_RX, 0, static_cast<double>(sh_data->rx_gain))) != 0) {
                std::cout << "[ERROR] SET RX GAIN | ERR CODE: " << err << "\n";
            }
            sh_data->changed_rx_gain = false;
        }

        if (sh_data->changed_tx_gain) {
            if (int err; (err = SoapySDRDevice_setGain(sdr.sdr, SOAPY_SDR_TX, 0, static_cast<double>(sh_data->tx_gain))) != 0) {
                std::cout << "[ERROR] SET TX GAIN | ERR CODE: " << err << "\n";
            }
            sh_data->changed_tx_gain = false;
        }

        if (sh_data->changed_rx_freq) {
            if (int err; (err = SoapySDRDevice_setFrequency(sdr.sdr, SOAPY_SDR_RX, 0, sh_data->rx_frequency, NULL)) != 0) {
                std::cout << "[ERROR] SET RX FREQ | ERR CODE: " << err << "\n";
            }
            sh_data->changed_rx_freq = false;
        }

        if (sh_data->changed_tx_freq) {
            if (int err; (err = SoapySDRDevice_setFrequency(sdr.sdr, SOAPY_SDR_TX, 0, sh_data->tx_frequency, NULL)) != 0) {
                std::cout << "[ERROR] SET TX FREQ | ERR CODE: " << err << "\n";
            }
            sh_data->changed_tx_freq = false;
        }

        if (sh_data->changed_sample_rate) {
            if (int err; (err = SoapySDRDevice_setSampleRate(sdr.sdr, SOAPY_SDR_RX, 0, sh_data->sample_rate)) != 0) {
                std::cout << "[ERROR] SET RX SAMPLE RATE | ERR CODE: " << err << "\n";
            }
            if (int err; (err = SoapySDRDevice_setSampleRate(sdr.sdr, SOAPY_SDR_TX, 0, sh_data->sample_rate)) != 0) {
                std::cout << "[ERROR] SET TX SAMPLE RATE | ERR CODE: " << err << "\n";
            }
            sh_data->changed_sample_rate = false;
        }

        if (sh_data->changed_rx_bandwidth) {
            if (int err; (err = SoapySDRDevice_setBandwidth(sdr.sdr, SOAPY_SDR_RX, 0, static_cast<double>(sh_data->rx_bandwidth))) != 0) {
                std::cout << "[ERROR] SET RX BANDWIDTH | ERR CODE: " << err << "\n";
            }
            sh_data->changed_rx_bandwidth = false;
        }

        if (sh_data->changed_tx_bandwidth) {
            if (int err; (err = SoapySDRDevice_setBandwidth(sdr.sdr, SOAPY_SDR_TX, 0, static_cast<double>(sh_data->tx_bandwidth))) != 0) {
                std::cout << "[ERROR] SET TX BANDWIDTH | ERR CODE: " << err << "\n";
            }
            sh_data->changed_tx_bandwidth = false;
        }


        std::transform(sdr.rx_buffer.begin(), sdr.rx_buffer.end(), rx_buffer_double.begin(), [](int16_t v) { return static_cast<double>(v); });

        for (size_t i = 0; i < sdr.rx_mtu; ++i) {
            sh_data->real_p[i] = rx_buffer_double[i * 2];
            sh_data->imag_p[i] = rx_buffer_double[i * 2 + 1];
        }

        // for (size_t i = 0; i < sdr.rx_mtu; ++i) {
        //     in1[i][0] = static_cast<double>(sdr.rx_buffer[2 * i] / 32768.0);
        //     in1[i][1] = static_cast<double>(sdr.rx_buffer[2 * i + 1] / 32768.0);
        // }

        // fft(in1, out1, sdr.rx_mtu);

        // for (size_t i = 0; i < sdr.rx_mtu; ++i) {
        //     double real = out1[i][0];
        //     double imag = out1[i][1];
        //     magnitude[i] = 20.0 * log10(sqrt(real * real + imag * imag) / sdr.rx_mtu) + 1e-12;
        //     sh_data->argument[i] = atan2(imag, real);
        // }

        // for (size_t i = 0; i < sdr.rx_mtu / 2; ++i) {
        //     sh_data->shifted_magnitude[i] = magnitude[i + sdr.rx_mtu / 2];
        //     sh_data->shifted_magnitude[i + sdr.rx_mtu / 2] = magnitude[i];
        // }
    }
    std::cout << "[TX] " << "Transmission complited\n";
    fftw_free(in);
    fftw_free(out);
    fftw_free(in1);
    fftw_free(out1);
}

void run_gui(sharedData *sh_data) {
    std::vector<float> bandwidths = {2e5f, 1e6f, 2e6f, 3e6f, 4e6f, 5e6f, 6e6f, 7e6f, 8e6f, 9e6f, 10e6f};
    int cur_rx_bandwidth = 1;
    int cur_tx_bandwidth = 1;


    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER);
    SDL_Window* window = SDL_CreateWindow(
        "Backend start", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        1920, 1080, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    SDL_GL_SetSwapInterval(0);

    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 10.f;
    style.FrameRounding = 8.f;
    style.ChildRounding = 8.f;
    style.ScrollbarRounding = 10.f;
    style.TabRounding = 8.f;
    style.WindowBorderSize = 0.0f;
    style.FrameBorderSize = 0.0f;
    style.PopupBorderSize = 0.0f;
    style.WindowPadding = ImVec2(10, 10);
    style.FramePadding = ImVec2(4, 4);
    style.ItemSpacing = ImVec2(8, 8);
    style.ItemInnerSpacing = ImVec2(4, 4);

    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init("#version 330");

    bool running = true;
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) {
                running = false;
            }
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();
        ImGui::DockSpaceOverViewport(0, nullptr, ImGuiDockNodeFlags_None);

        ImGui::Begin("Signal Workspace", nullptr, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
            ImVec2 size = ImGui::GetContentRegionAvail();
            float quoter_w = size.x * 0.25f;
            float quoter_h = size.y * 0.33f;
            float tr_quoter_w = size.x * 0.75f;

            ImGui::BeginChild("Constellation Diagram", ImVec2(quoter_w, quoter_h), true);
                if (ImPlot::BeginPlot("Constellation Diagram")) {
                    ImPlot::PlotScatter("I/Q", sh_data->real_p.data(), sh_data->imag_p.data(), sh_data->imag_p.size());
                    ImPlot::EndPlot();
                }
            ImGui::EndChild();
            ImGui::SameLine();
            ImGui::BeginChild("I/Q samples", ImVec2(tr_quoter_w, quoter_h), true);
                if (ImPlot::BeginPlot("I/Q samples")) {
                    ImPlot::PlotLine("I", sh_data->real_p.data(), sh_data->real_p.size());
                    ImPlot::PlotLine("Q", sh_data->imag_p.data(), sh_data->imag_p.size());
                    ImPlot::EndPlot();
                }
            ImGui::EndChild();
            ImGui::BeginChild("Magnitude", ImVec2(size.x, quoter_h), true);
                if (ImPlot::BeginPlot("Magnitude")) {
                    ImPlot::PlotLine("Magnitude", sh_data->frequency_axis.data(), sh_data->shifted_magnitude.data(), sh_data->shifted_magnitude.size());
                    ImPlot::EndPlot();
                }
            ImGui::EndChild();
            ImGui::BeginChild("Argument", ImVec2(size.x, quoter_h), true);
                if (ImPlot::BeginPlot("Argument")) {
                    ImPlot::PlotLine("Argument", sh_data->frequency_axis.data(), sh_data->argument.data(), sh_data->argument.size());
                    ImPlot::EndPlot();
                }
            ImGui::EndChild();
        ImGui::End();

        ImGui::Begin("Settings");
        ImGui::Text("FPS: %.1f (%.3f ms)", io.Framerate, 1000.0f / io.Framerate);
        if (ImGui::BeginTabBar("Control Panel")) {
            if (ImGui::BeginTabItem("Config")) {

                ImGui::SeparatorText("Processing Blocks");
                ImGui::Checkbox("Transmission", &sh_data->send);

                ImGui::SeparatorText("Pre Modulation");
                const char* rx_mod_type = nullptr;
                const char* tx_mod_type = nullptr;

                switch (sh_data->modul_type_RX) {
                    case ModulationType::BPSK: rx_mod_type = "RX Modulation BPSK"; break;
                    case ModulationType::QPSK: rx_mod_type = "RX Modulation QPSK"; break;
                    case ModulationType::QAM16: rx_mod_type = "RX Modulation QAM16"; break;
                }
                switch (sh_data->modul_type_TX) {
                    case ModulationType::BPSK: tx_mod_type = "TX Modulation BPSK"; break;
                    case ModulationType::QPSK: tx_mod_type = "TX Modulation QPSK"; break;
                    case ModulationType::QAM16: tx_mod_type = "TX Modulation QAM16"; break;
                }

                if (ImGui::BeginCombo("RX Modulation Type", rx_mod_type)) {
                    if (ImGui::Selectable("BPSK", sh_data->modul_type_RX == ModulationType::BPSK)) sh_data->modul_type_RX = ModulationType::BPSK;
                    if (ImGui::Selectable("QPSK", sh_data->modul_type_RX == ModulationType::QPSK)) sh_data->modul_type_RX = ModulationType::QPSK;
                    if (ImGui::Selectable("QAM16", sh_data->modul_type_RX == ModulationType::QAM16)) sh_data->modul_type_RX = ModulationType::QAM16;
                    ImGui::EndCombo();
                }

                if (ImGui::BeginCombo("TX Modulation Type", tx_mod_type)) {
                    if (ImGui::Selectable("BPSK", sh_data->modul_type_TX == ModulationType::BPSK)) sh_data->modul_type_TX = ModulationType::BPSK;
                    if (ImGui::Selectable("QPSK", sh_data->modul_type_TX == ModulationType::QPSK)) sh_data->modul_type_TX = ModulationType::QPSK;
                    if (ImGui::Selectable("QAM16", sh_data->modul_type_TX == ModulationType::QAM16)) sh_data->modul_type_TX = ModulationType::QAM16;
                    ImGui::EndCombo();
                }

                ImGui::InputInt("Upsample Coefficient", &sh_data->upsample_koef, 1, 1e1);
                ImGui::InputInt("Cycle Prefix", &sh_data->CYCLIC_PREFEX, 1);
                ImGui::InputInt("Subcarrier", &sh_data->SUBCARRIER, 1);

                ImGui::SeparatorText("SDR Configuration");

                if (ImGui::BeginCombo("SDR Device", sh_data->device.c_str())) {
                    for (const auto& dev : sh_data->devices) {
                        bool is_selected = (sh_data->device == dev);
                        if (ImGui::Selectable(dev.c_str(), is_selected)) sh_data->device = dev;
                        if (is_selected) ImGui::SetItemDefaultFocus();
                    }
                    ImGui::EndCombo();
                }

                if (ImGui::DragFloat("RX Gain", &sh_data->rx_gain, 0.25f, 0.f, 73.f)){
                    sh_data->changed_rx_gain = true;
                }
                if (ImGui::DragFloat("TX Gain", &sh_data->tx_gain, 0.25f, 0.f, 89.f)) {
                    sh_data->changed_tx_gain = true;
                }
                if (ImGui::InputDouble("RX Frequency", &sh_data->rx_frequency, 1e2, 1e3)) {
                    sh_data->changed_rx_freq = true;
                }
                if (ImGui::InputDouble("TX Frequency", &sh_data->tx_frequency, 1e2, 1e3)) {
                    sh_data->changed_tx_freq = true;
                }
                if (ImGui::SliderInt("RX Bandwidth", &cur_rx_bandwidth, 0, bandwidths.size() - 1, std::to_string(bandwidths[cur_rx_bandwidth]).c_str())) {
                    sh_data->rx_bandwidth = bandwidths[cur_rx_bandwidth];
                    sh_data->changed_rx_bandwidth = true;
                }
                if (ImGui::SliderInt("TX Bandwidth", &cur_tx_bandwidth, 0, bandwidths.size() - 1, std::to_string(bandwidths[cur_tx_bandwidth]).c_str())) {
                    sh_data->tx_bandwidth = bandwidths[cur_tx_bandwidth];
                    sh_data->changed_tx_bandwidth = true;
                }
                if (ImGui::InputDouble("Sample Rate", &sh_data->sample_rate, 1e5, 1e6)) {
                    sh_data->changed_sample_rate = true;
                }

                ImGui::SeparatorText("Time Control");
                const char* label_time = sh_data->cont_time ? "Running" : "Stopped";

                if (ImGui::Button(label_time)) {
                    sh_data->cont_time = !sh_data->cont_time;
                }
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("View")) {
                ImGui::SeparatorText("Theme Colors");
                if (ImGui::Button("Light Theme")) ImGui::StyleColorsLight();
                ImGui::SameLine();
                if (ImGui::Button("Dark Theme")) ImGui::StyleColorsDark();
                ImGui::SameLine();
                if (ImGui::Button("Classic Theme")) ImGui::StyleColorsClassic();
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }
        ImGui::End();

        // ImGui::ShowDemoWindow();

        ImGui::Render();
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);
    }
    sh_data->quit = true;
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();
}

int main() {
    sharedData sd(1920);

    std::thread gui_thread(run_gui, &sd);
    std::thread backend_thread(run_backend, &sd);

    backend_thread.join();
    gui_thread.join();

    return 0;
}
