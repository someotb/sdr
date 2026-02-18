#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <SoapySDR/Types.h>
#include <cstddef>
#include <cstdint>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <complex.h>
#include <math.h>
#include <fftw3.h>
#include <unistd.h>
#include <vector>
#include <string.h>
#include <GL/glew.h>
#include <SDL2/SDL.h>
#include <thread>
#include <cmath>
#include <iostream>
#include <iomanip>
#include "imgui.h"
#include "implot.h"
#include "backends/imgui_impl_opengl3.h"
#include "backends/imgui_impl_sdl2.h"
#include "modulation.hpp"
#include "sdr.hpp"

constexpr int UPSAMPLE = 10;
constexpr size_t N_BUFFERS = 1000000;
constexpr long long TIMEOUT = 400000;
constexpr long long TX_DELAY = 4000000;

struct sharedData
{
    vector<int16_t> real_p_aft_con;
    vector<int16_t> imag_p_aft_con;
    vector<int16_t> real_p_aft_con_offset;
    vector<int16_t> imag_p_aft_con_offset;
    vector<int16_t> real_p;
    vector<int16_t> imag_p;
    vector<int> offset;
    vector<double> shifted_magnitude;
    vector<double> argument;
    vector<double> frequency_axis;
    bool send = false;
    bool quit = false;
    bool changed_rx_gain = false;
    bool changed_tx_gain = false;
    bool changed_rx_freq = false;
    bool changed_tx_freq = false;
    bool changed_sample_rate = false;
    bool changed_rx_bandwidth = false;
    bool changed_tx_bandwidth = false;
    float rx_gain = 20.f;
    float tx_gain = 80.f;
    double rx_frequency = 826e6;
    double tx_frequency = 826e6;
    double sample_rate = 1e6;
    float rx_bandwidth = 1e6;
    float tx_bandwidth = 1e6;
    double BnTs = 0.0001f;

    sharedData(size_t rx_mtu) {
        real_p_aft_con_offset.resize(rx_mtu / 10, 0);
        imag_p_aft_con_offset.resize(rx_mtu / 10, 0);
        real_p_aft_con.resize(rx_mtu, 0);
        imag_p_aft_con.resize(rx_mtu, 0);
        offset.resize(rx_mtu / 10, 0);
        real_p.resize(rx_mtu, 0);
        imag_p.resize(rx_mtu, 0);
        shifted_magnitude.resize(rx_mtu, 0);
        argument.resize(rx_mtu, 0);
        frequency_axis.resize(rx_mtu, 0);
    }
};

void run_backend(sharedData *sh_data, char *argv[]) {
    SDRDevice sdr(argv[1]);

    ModulationType modulation = ModulationType::QPSK;
    int bits_size = bits_per_symbol(modulation);
    size_t max_symbols = sdr.tx_mtu / UPSAMPLE;

    vector<int16_t> bits(max_symbols * bits_size);
    vector<int16_t> impulse_int16_t(UPSAMPLE, 1);
    vector<int16_t> rx_buffer_after_conv(sdr.rx_buffer.size(), 0);
    vector<complex<double>> symbols(bits.size() / bits_size);
    vector<complex<double>> impulse(UPSAMPLE, 1.0);
    vector<complex<double>> symbols_ups(sdr.tx_mtu * UPSAMPLE);
    vector<double> magnitude(sdr.rx_mtu, 0);

    fftw_complex* in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * sdr.rx_mtu);
    fftw_complex* out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * sdr.rx_mtu);
    fftw_plan plan = fftw_plan_dft_1d(sdr.rx_mtu, in, out, FFTW_FORWARD, FFTW_ESTIMATE);

    for (size_t i = 0; i < sdr.rx_mtu; ++i) {
        sh_data->frequency_axis[i] = (i - sdr.rx_mtu / 2.0) * sh_data->sample_rate / sdr.rx_mtu;
    }

    for (size_t i = 0; i < bits.size(); i++) bits[i] = rand() % 2;

    modulate(bits, symbols, modulation);
    UpSampler(symbols, symbols_ups, UPSAMPLE);
    filter(symbols_ups, impulse);

    for (size_t i = 0; i < sdr.tx_mtu; i++) {
        sdr.tx_buffer[2*i] = (real(symbols_ups[i]) * 16000);
        sdr.tx_buffer[2*i+1] = (imag(symbols_ups[i]) * 16000);
    }

    cout << "[TX] " << "Transmission of '" << N_BUFFERS << "' buffers started:\n";
    for (size_t i = 0; i < N_BUFFERS; ++i) {
        if (sh_data->changed_rx_gain) {
            if (int err; (err = SoapySDRDevice_setGain(sdr.sdr, SOAPY_SDR_RX, 0, static_cast<double>(sh_data->rx_gain))) != 0) {
                cout << "[ERROR] SET RX GAIN | ERR CODE: " << err << "\n";
            }
            sh_data->changed_rx_gain = false;
        }

        if (sh_data->changed_tx_gain) {
            if (int err; (err = SoapySDRDevice_setGain(sdr.sdr, SOAPY_SDR_TX, 0, static_cast<double>(sh_data->tx_gain))) != 0) {
                cout << "[ERROR] SET TX GAIN | ERR CODE: " << err << "\n";
            }
            sh_data->changed_tx_gain = false;
        }

        if (sh_data->changed_rx_freq) {
            if (int err; (err = SoapySDRDevice_setFrequency(sdr.sdr, SOAPY_SDR_RX, 0, sh_data->rx_frequency, NULL)) != 0) {
                cout << "[ERROR] SET RX FREQ | ERR CODE: " << err << "\n";
            }
            sh_data->changed_rx_freq = false;
        }

        if (sh_data->changed_tx_freq) {
            if (int err; (err = SoapySDRDevice_setFrequency(sdr.sdr, SOAPY_SDR_TX, 0, sh_data->tx_frequency, NULL)) != 0) {
                cout << "[ERROR] SET TX FREQ | ERR CODE: " << err << "\n";
            }
            sh_data->changed_tx_freq = false;
        }

        if (sh_data->changed_sample_rate) {
            if (int err; (err = SoapySDRDevice_setSampleRate(sdr.sdr, SOAPY_SDR_RX, 0, sh_data->sample_rate)) != 0) {
                cout << "[ERROR] SET RX SAMPLE RATE | ERR CODE: " << err << "\n";
            }
            if (int err; (err = SoapySDRDevice_setSampleRate(sdr.sdr, SOAPY_SDR_TX, 0, sh_data->sample_rate)) != 0) {
                cout << "[ERROR] SET TX SAMPLE RATE | ERR CODE: " << err << "\n";
            }
            sh_data->changed_sample_rate = false;
        }

        if (sh_data->changed_rx_bandwidth) {
            if (int err; (err = SoapySDRDevice_setBandwidth(sdr.sdr, SOAPY_SDR_RX, 0, static_cast<double>(sh_data->rx_bandwidth))) != 0) {
                cout << "[ERROR] SET RX BANDWIDTH | ERR CODE: " << err << "\n";
            }
            sh_data->changed_rx_bandwidth = false;
        }

        if (sh_data->changed_tx_bandwidth) {
            if (int err; (err = SoapySDRDevice_setBandwidth(sdr.sdr, SOAPY_SDR_TX, 0, static_cast<double>(sh_data->tx_bandwidth))) != 0) {
                cout << "[ERROR] SET TX BANDWIDTH | ERR CODE: " << err << "\n";
            }
            sh_data->changed_tx_bandwidth = false;
        }

        if (sh_data->quit) break;

        void *rx_buffs[] = {sdr.rx_buffer.data()};
        const void *tx_buffs[] = {sdr.tx_buffer.data()};

        int flags = 0;
        long long timeNs = 0;

        int sr = SoapySDRDevice_readStream(sdr.sdr, sdr.rxStream, rx_buffs, sdr.rx_mtu, &flags, &timeNs, TIMEOUT);
        (void)sr;

        for (size_t i = 0; i < sdr.rx_mtu; ++i) {
            in[i][0] = static_cast<double>(sdr.rx_buffer[2 * i] / 32768.0);
            in[i][1] = static_cast<double>(sdr.rx_buffer[2 * i + 1] / 32768.0);
        }

        fftw_execute(plan);

        for (size_t i = 0; i < sdr.rx_mtu; ++i) {
            double real = out[i][0];
            double imag = out[i][1];
            magnitude[i] = 20.0 * log10(sqrt(real * real + imag * imag) / 1920.0) + 1e-12;
            sh_data->real_p[i] = sdr.rx_buffer[2 * i];
            sh_data->imag_p[i] = sdr.rx_buffer[2 * i + 1];
            sh_data->argument[i] = atan2(imag, real);
        }

        for (size_t i = 0; i < sdr.rx_mtu / 2; ++i) {
            sh_data->shifted_magnitude[i] = magnitude[i + sdr.rx_mtu / 2];
            sh_data->shifted_magnitude[i + sdr.rx_mtu / 2] = magnitude[i];
        }

        long long tx_time = timeNs + TX_DELAY;
        flags = SOAPY_SDR_HAS_TIME;

        if (sh_data->send) {
            int st = SoapySDRDevice_writeStream(sdr.sdr, sdr.txStream, tx_buffs, sdr.tx_mtu, &flags, tx_time, TIMEOUT);
            (void)st;
        }

        filter_int16_t(sdr.rx_buffer, impulse_int16_t, rx_buffer_after_conv);

        for (size_t i = 0; i < sdr.rx_mtu; ++i) {
            sh_data->real_p_aft_con[i] = rx_buffer_after_conv[i * 2];
            sh_data->imag_p_aft_con[i] = rx_buffer_after_conv[i * 2 + 1];
        }

        symbols_sync(rx_buffer_after_conv, sh_data->offset, sh_data->BnTs);

        for (size_t i = 0; i + 10 < rx_buffer_after_conv.size() / 2; i += 10) {
            size_t k = i + sh_data->offset[i / 10];

            if (k >= rx_buffer_after_conv.size() / 2) break;

            sh_data->real_p_aft_con_offset[i / 10] = rx_buffer_after_conv[2 * k];
            sh_data->imag_p_aft_con_offset[i / 10] = rx_buffer_after_conv[2 * k + 1];
        }
    }
    cout << "[TX] " << "Transmission complited\n";
    fftw_destroy_plan(plan);
    fftw_free(in);
    fftw_free(out);
}

void run_gui(sharedData *sh_data) {
    vector<float> bandwidths = {2e5f, 1e6f, 2e6f, 3e6f, 4e6f, 5e6f, 6e6f, 7e6f, 8e6f, 9e6f, 10e6f};
    int cur_rx_bandwidth = 1;
    int cur_tx_bandwidth = 1;


    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER);
    SDL_Window* window = SDL_CreateWindow(
        "Backend start", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        1920, 1080, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);

    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
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

        ImGui::Begin("Constellation Diagram");
            if (ImPlot::BeginPlot("Constellation Diagram")) {
                ImPlot::PlotScatter("I/Q", sh_data->real_p.data(), sh_data->imag_p.data(), sh_data->imag_p.size());
                ImPlot::EndPlot();
            }
        ImGui::End();

        ImGui::Begin("Constellation Diagram After Convolve");
            if (ImPlot::BeginPlot("Constellation Diagram After Convolve")) {
                ImPlot::PlotScatter("I/Q", sh_data->real_p_aft_con.data(), sh_data->imag_p_aft_con.data(), sh_data->imag_p_aft_con.size());
                ImPlot::EndPlot();
            }
        ImGui::End();

        ImGui::Begin("Constellation Diagram After Convolve and Offset");
            if (ImPlot::BeginPlot("Constellation Diagram After Convolve and Offset")) {
                ImPlot::PlotScatter("I/Q", sh_data->real_p_aft_con_offset.data(), sh_data->imag_p_aft_con_offset.data(), sh_data->imag_p_aft_con_offset.size());
                ImPlot::EndPlot();
            }
        ImGui::End();

        ImGui::Begin("I/Q samples");
            if (ImPlot::BeginPlot("I/Q samples")) {
                ImPlot::PlotLine("I", sh_data->real_p.data(), sh_data->real_p.size());
                ImPlot::PlotLine("Q", sh_data->imag_p.data(), sh_data->imag_p.size());
                ImPlot::EndPlot();
            }
        ImGui::End();

        ImGui::Begin("I/Q samples After Convolve");
            if (ImPlot::BeginPlot("I/Q samples After Convolve")) {
                ImPlot::PlotLine("I", sh_data->real_p_aft_con.data(), sh_data->real_p_aft_con.size());
                ImPlot::PlotLine("Q", sh_data->imag_p_aft_con.data(), sh_data->imag_p_aft_con.size());
                ImPlot::EndPlot();
            }
        ImGui::End();

        ImGui::Begin("Magnitude");
            if (ImPlot::BeginPlot("Magnitude")) {
                ImPlot::PlotLine("Magnitude", sh_data->frequency_axis.data(), sh_data->shifted_magnitude.data(), sh_data->shifted_magnitude.size());
                ImPlot::EndPlot();
            }
        ImGui::End();

        ImGui::Begin("Argument");
            if (ImPlot::BeginPlot("Argument")) {
                ImPlot::PlotLine("Argument", sh_data->frequency_axis.data(), sh_data->argument.data(), sh_data->argument.size());
                ImPlot::EndPlot();
            }
        ImGui::End();

        ImGui::Begin("Offset");
        if (ImPlot::BeginPlot("Offset")) {
            ImPlot::PlotLine("Offset", sh_data->offset.data(), sh_data->offset.size());
            ImPlot::EndPlot();
        }
        ImGui::End();

        ImGui::Begin("Settings");
        if (ImGui::BeginTabBar("Control Panel")) {
            if (ImGui::BeginTabItem("TX CONFIG")) {
                if (ImGui::TreeNodeEx("Transmission")) {
                    ImGui::Checkbox("Transmission(on/off)", &sh_data->send);
                    if (ImGui::DragFloat("RX GAIN", &sh_data->rx_gain, 0.25f, 0.f, 73.f)){
                        sh_data->changed_rx_gain = true;
                    }
                    if (ImGui::DragFloat("TX GAIN", &sh_data->tx_gain, 0.25f, 0.f, 89.f)) {
                        sh_data->changed_tx_gain = true;
                    }
                    if (ImGui::InputDouble("RX FREQUENCY", &sh_data->rx_frequency, 1e2)) {
                        sh_data->changed_rx_freq = true;
                    }
                    if (ImGui::InputDouble("TX FREQUENCY", &sh_data->tx_frequency, 1e2)) {
                        sh_data->changed_tx_freq = true;
                    }
                    if (ImGui::InputDouble("SAMPLE RATE", &sh_data->sample_rate, 1e5)) {
                        sh_data->changed_sample_rate = true;
                    }
                    if (ImGui::SliderInt("RX BANDWIDTH", &cur_rx_bandwidth, 0, bandwidths.size() - 1, to_string(bandwidths[cur_rx_bandwidth]).c_str())) {
                        sh_data->rx_bandwidth = bandwidths[cur_rx_bandwidth];
                        sh_data->changed_rx_bandwidth = true;
                    }
                    if (ImGui::SliderInt("TX BANDWIDTH", &cur_tx_bandwidth, 0, bandwidths.size() - 1, to_string(bandwidths[cur_tx_bandwidth]).c_str())) {
                        sh_data->tx_bandwidth = bandwidths[cur_tx_bandwidth];
                        sh_data->changed_tx_bandwidth = true;
                    }
                    ImGui::InputDouble("BnTs VALUE", &sh_data->BnTs, 0.0001);
                    ImGui::TreePop();
                }
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("VIEW")) {
                if (ImGui::TreeNodeEx("Theme Color")) {
                    if (ImGui::Button("Light")) ImGui::StyleColorsLight();
                    if (ImGui::Button("Dark")) ImGui::StyleColorsDark();
                    if (ImGui::Button("Classic")) ImGui::StyleColorsClassic();
                    ImGui::TreePop();
                }
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

int main(int argc, char *argv[]) {
    (void) argc;
    sharedData sd(1920);

    std::thread gui_thread(run_gui, &sd);
    std::thread backend_thread(run_backend, &sd, argv);

    backend_thread.join();
    gui_thread.join();

    return 0;
}
