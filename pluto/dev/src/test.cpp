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
#include <iomanip>
#include <GL/glew.h>
#include <SDL2/SDL.h>
#include <chrono>
#include <thread>
#include <cmath>
#include "imgui.h"
#include "implot.h"
#include "backends/imgui_impl_opengl3.h"
#include "backends/imgui_impl_sdl2.h"
#include "modulation.h"
#include "sdr.h"

struct sharedData 
{
    vector<int16_t> bits;
};

void run_backend(struct sharedData &data) {
    for (size_t i = 0; i < 10000; ++i) {
        data.bits.push_back(rand() % 2);
    }
}

void run_gui(struct sharedData &data) {
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER);
    SDL_Window* window = SDL_CreateWindow(
        "Backend start", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        1024, 768, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);

    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Включить Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Включить Gamepad Controls
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;         // Включить Docking
    
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

        vector<int16_t> real;
        vector<int16_t> imag;

        for (size_t i = 0; i < data.bits.size(); ++i) {
            real.push_back(data.bits[i]);
            imag.push_back(data.bits[i]);
        }

        if (ImPlot::BeginPlot("Plot Line")) {
            ImPlot::PlotLine("real / imag", real.data(), imag.data(), real.size());
            ImPlot::EndPlot();
        }

        ImGui::Render();
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        SDL_GL_SwapWindow(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();
}

int main() {
    struct sharedData sd;

    std::thread gui_thread(run_gui, ref(sd));
    gui_thread.join();
    std::thread backend_thread(run_backend, ref(sd));
    backend_thread.join();
    
    return 0;
}