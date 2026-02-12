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
#include "modulation.h"
#include "sdr.h"

constexpr int UPSAMPLE = 10;
constexpr size_t N_BUFFERS = 1000000;
constexpr long long TIMEOUT = 400000;
constexpr long long TX_DELAY = 4000000;

struct sharedData
{
    std::vector<int16_t> datas;
    bool send;
    bool quit;
    float rx_gain;
    float tx_gain;
    float frequency;
};

static void HelpMarker(const char* desc)
{
    ImGui::TextDisabled("(?)");
    if (ImGui::BeginItemTooltip())
    {
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
        ImGui::TextUnformatted(desc);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

void run_backend(sharedData *sh_data, char *argv[]) {
    SDRDevice sdr(argv[1]);
    SoapySDRRange r;
    r = SoapySDRDevice_getGainRange(sdr.sdr, 0, 0);
    cout << "[INFO] " << "TX GAIN: " << "MAX: " << r.maximum << " | MIN: " << r.minimum << " | STEP: " << r.step << "\n";
    r = SoapySDRDevice_getGainRange(sdr.sdr, 1, 0);
    cout << "[INFO] " << "RX GAIN: " << "MAX: " << r.maximum << " | MIN: " << r.minimum << " | STEP: " << r.step << "\n";


    ModulationType modulation = ModulationType::QPSK;
    int bits_size = bits_per_symbol(modulation);
    size_t max_symbols = sdr.tx_mtu / UPSAMPLE;

    vector<int16_t> bits(max_symbols * bits_size);
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
    int buffers_per_second = SAMPLE_RATE/sdr.tx_mtu;
    int sec = 0;
    int cnt_of_buffers = 0;
    cout << "[TX] " << "Transmission of '" << N_BUFFERS << "' buffers started:\n";
    for (size_t i = 0; i < N_BUFFERS; ++i) {
        cnt_of_buffers = i;
        if (sh_data->quit) break;

        // if (int err = sdr.set_frequency(static_cast<double>(sh_data->frequency), 1)) cout << "[ERROR] SET FREQ | ERR CODE: " << err << "\n";

        if (i % buffers_per_second == 0 && i != 0) {
            sec++;
            cout << "[TX] " << "Minutes: " << setw(2) << setfill('0') << (sec / 60) << ":" << setw(2) << setfill('0') << (sec % 60) << " | Buffers: " << i << endl;
        }

        void *rx_buffs[] = {sh_data->datas.data()};
        const void *tx_buffs[] = {sdr.tx_buffer.data()};

        int flags = 0;
        long long timeNs = 0;

        if (int err; (err = SoapySDRDevice_setGainElement(sdr.sdr, 1, 0, "LNA", static_cast<double>(sh_data->rx_gain))) != 0) {
            cout << "[ERROR] SET RX GAIN | ERR CODE: " << err << "\n";
        }
        int sr = SoapySDRDevice_readStream(sdr.sdr, sdr.rxStream, rx_buffs, sdr.rx_mtu, &flags, &timeNs, TIMEOUT);
        (void)sr;

        long long tx_time = timeNs + TX_DELAY;
        flags = SOAPY_SDR_HAS_TIME;

        if (sh_data->send) {
            if (int err; (err = SoapySDRDevice_setGainElement(sdr.sdr, SOAPY_SDR_TX, 0, "PDA", static_cast<double>(sh_data->tx_gain))) != 0) {
                cout << "[ERROR] SET TX GAIN | ERR CODE: " << err << "\n";
            }

            int st = SoapySDRDevice_writeStream(sdr.sdr, sdr.txStream, tx_buffs, sdr.tx_mtu, &flags, tx_time, TIMEOUT);
            (void)st;
        }
    }
    cout << "[TX] " << "Transmission of '" << cnt_of_buffers << "' buffers complited\n";
}

void run_gui(sharedData *sh_data) {
    std::vector<int16_t> real(1920);
    std::vector<int16_t> imag(1920);

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
    ImVec2 Window_Size = ImVec2(1920, 1080);


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
        ImGui::Begin("Graphs");
        Window_Size = ImGui::GetWindowSize();
        Window_Size.y -= 60;
        Window_Size.y /= 2;
        Window_Size.x -= 20;
        for (int i = 0; i < 1920; ++i) {
            real[i] = sh_data->datas[2 * i];
            imag[i] = sh_data->datas[2 * i + 1];
        }
        if (ImPlot::BeginPlot("Scatter", Window_Size)) {
            ImPlot::PlotScatter("real / imag", real.data(), imag.data(), 1920);
            ImPlot::EndPlot();
        }
        if (ImPlot::BeginPlot("Line", Window_Size)) {
            ImPlot::PlotLine("real", real.data(), real.size());
            ImPlot::PlotLine("imag", imag.data(), imag.size());
            ImPlot::EndPlot();
        }
        ImGui::End();

        ImGui::Begin("Settings");
        if (ImGui::BeginTabBar("Control Panel")) {
            if (ImGui::BeginTabItem("TX CONFIG")) {
                if (ImGui::TreeNodeEx("Transmission")) {
                    ImGui::Checkbox("Transmission(on/off)", &sh_data->send);
                    ImGui::DragFloat("RX GAIN", &sh_data->rx_gain, 0.25f, 1.f, 73.f);
                    ImGui::SameLine(); HelpMarker("Ctrl+Click to input value.");
                    ImGui::DragFloat("TX GAIN", &sh_data->tx_gain, 0.25f, 0.f, 89.f);
                    ImGui::SameLine(); HelpMarker("Ctrl+Click to input value.");
                    ImGui::DragFloat("FREQUENCY", &sh_data->frequency, 1000000.f, 70.f * 1000000.f, 6.f * 1000000000.f);
                    ImGui::SameLine(); HelpMarker("Ctrl+Click to input value.");
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
    sharedData sd;
    sd.datas.resize(1920 * 2);
    sd.send = false;
    sd.quit = false;
    sd.rx_gain = 25.f;
    sd.tx_gain = 60.f;
    sd.frequency = 868000000;


    std::thread gui_thread(run_gui, &sd);
    std::thread backend_thread(run_backend, &sd, argv);

    backend_thread.join();
    gui_thread.join();

    return 0;
}
