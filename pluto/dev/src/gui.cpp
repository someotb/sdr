#include "gui.hpp"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>
#include "imgui.h"
#include "implot.h"

#include <vector>

void run_gui(sharedData &sh_data)
{
    std::vector<float> bandwidths = {2e5f, 1e6f, 2e6f, 3e6f, 4e6f, 5e6f, 6e6f, 7e6f, 8e6f, 9e6f, 10e6f};
    int cur_rx_bandwidth = 1;
    int cur_tx_bandwidth = 1;

    if (!glfwInit()) return;

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    #ifdef __APPLE__
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    #endif

    GLFWwindow* window = glfwCreateWindow(1280, 720, "MMS", nullptr, nullptr);
    if (!window) return;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGui::StyleColorsDark();
    ImGuiIO& io = ImGui::GetIO();

    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport());

        // Start GUI

        const float *raw_data = reinterpret_cast<const float *>(sh_data.rx_complex.data());
        const float *dsp_data = reinterpret_cast<const float *>(sh_data.rx_complex_fft_gui.data());

        if (ImGui::Begin("Scatter Raw"))
        {
            if (ImPlot::BeginPlot("Raw Samples", ImVec2(ImGui::GetContentRegionAvail())))
            {
                ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 1.5f, ImPlot::GetColormapColor(0), 1.0f, ImPlot::GetColormapColor(0));
                ImPlot::PlotScatter("I/Q", raw_data, raw_data + 1, sh_data.rx_complex.size(), 0, 0, sizeof(std::complex<float>));
                ImPlot::EndPlot();
            }
        }
        ImGui::End();

        if (ImGui::Begin("Scatter FFT"))
        {
            ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 1.5f, ImPlot::GetColormapColor(0), 1.0f, ImPlot::GetColormapColor(0));
            if (ImPlot::BeginPlot("Samples After FFT", ImVec2(ImGui::GetContentRegionAvail())))
            {
                ImPlot::PlotScatter("I/Q", dsp_data, dsp_data + 1, sh_data.rx_complex_fft_gui.size(), 0, 0, sizeof(std::complex<float>));
                ImPlot::EndPlot();
            }
        }
        ImGui::End();

        if (ImGui::Begin("Plot Raw"))
        {
            if (ImPlot::BeginPlot("Raw I/Q samples", ImVec2(ImGui::GetContentRegionAvail())))
            {
                ImPlot::PlotLine("I", raw_data, sh_data.rx_complex.size(), 1.0, 0, 0, 0, sizeof(std::complex<float>));
                ImPlot::PlotLine("Q", raw_data + 1, sh_data.rx_complex.size(), 1.0, 0, 0, 0, sizeof(std::complex<float>));
                ImPlot::EndPlot();
            }
        }
        ImGui::End();

        if (ImGui::Begin("Plot FFT"))
        {
            if (ImPlot::BeginPlot("I/Q Samples After FFT", ImVec2(ImGui::GetContentRegionAvail())))
            {
                ImPlot::PlotLine("I", dsp_data, sh_data.rx_complex_fft_gui.size(), 1.0, 0, 0, 0, sizeof(std::complex<float>));
                ImPlot::PlotLine("Q", dsp_data + 1, sh_data.rx_complex_fft_gui.size(), 1.0, 0, 0, 0, sizeof(std::complex<float>));
                ImPlot::EndPlot();
            }
        }
        ImGui::End();

        if (ImGui::Begin("Argument"))
        {
            if (ImPlot::BeginPlot("Signal Argument", ImVec2(ImGui::GetContentRegionAvail())))
            {
                ImPlot::PlotLine("Argument", sh_data.frequency_axis.data(), sh_data.argument.data(), sh_data.argument.size());
                ImPlot::EndPlot();
            }
        }
        ImGui::End();

        if (ImGui::Begin("Magnitude"))
        {
            if (ImPlot::BeginPlot("Signal Magnitude", ImVec2(ImGui::GetContentRegionAvail())))
            {
                ImPlot::PlotLine("Magnitude", sh_data.frequency_axis.data(), sh_data.shifted_magnitude.data(), sh_data.shifted_magnitude.size());
                ImPlot::EndPlot();
            }
        }
        ImGui::End();

        if (sh_data.flags.debug)
        {
            if (ImGui::Begin("Latency"))
            {
                if (ImPlot::BeginPlot("Latency", ImVec2(ImGui::GetContentRegionAvail())))
                {
                    if (!sh_data.milisecs.empty())
                    {
                        ImPlot::PlotLine("Latency", sh_data.milisecs.data(), sh_data.milisecs.size());
                        ImPlot::EndPlot();
                    }
                    else
                    {
                        ImPlot::Annotation(0.5, 0.5, ImVec4(1, 0, 0, 1), ImVec2(0, 0), true, "No Data");
                        ImPlot::EndPlot();
                    }
                }
            }
            ImGui::End();

            if (ImGui::Begin("Zadoff-Chu"))
            {
                if (ImPlot::BeginPlot("Zadoff-Chu Correlation Array", ImVec2(ImGui::GetContentRegionAvail())))
                {
                    if (!sh_data.zadoff_corr_arr.empty())
                    {
                        ImPlot::PlotLine("Zadoff-Chu", sh_data.zadoff_corr_arr.data(), sh_data.zadoff_corr_arr.size());
                        ImPlot::EndPlot();
                    }
                    else
                    {
                        ImPlot::Annotation(0.5, 0.5, ImVec4(1, 0, 0, 1), ImVec2(0, 0), true, "No Data");
                        ImPlot::EndPlot();
                    }
                }
            }
            ImGui::End();

            if (ImGui::Begin("CFO"))
            {
                if (ImPlot::BeginPlot("CFO Correction Array", ImVec2(ImGui::GetContentRegionAvail())))
                {
                    if (!sh_data.cfo_offset.empty())
                    {
                        ImPlot::PlotLine("CFO", sh_data.cfo_offset.data(), sh_data.cfo_offset.size());
                        ImPlot::EndPlot();
                    }
                    else
                    {
                        ImPlot::Annotation(0.5, 0.5, ImVec4(1, 0, 0, 1), ImVec2(0, 0), true, "No Data");
                        ImPlot::EndPlot();
                    }
                }
            }
            ImGui::End();

            if (ImGui::Begin("Demapped Bits"))
            {
                if (sh_data.flags.constant_mode)
                {
                    ImGui::Text("Error Counter: %d", sh_data.err_cnt);
                    if (ImPlot::BeginPlot("Demapped Bits vs Sended", ImVec2(ImGui::GetContentRegionAvail())))
                    {
                        if (!sh_data.demaped_bits.empty())
                        {
                            ImPlot::PlotLine("RX", sh_data.demaped_bits.data(), sh_data.demaped_bits.size());
                            ImPlot::PlotLine("TX", sh_data.bits.data(), sh_data.bits.size());
                            ImPlot::EndPlot();
                        }
                        else
                        {
                            ImPlot::Annotation(0.5, 0.5, ImVec4(1, 0, 0, 1), ImVec2(0, 0), true, "No Data");
                            ImPlot::EndPlot();
                        }
                    }
                }

                if (sh_data.flags.one_time_mode)
                {
                    if (ImPlot::BeginPlot("Demapped Bits", ImVec2(ImGui::GetContentRegionAvail())))
                    {
                        if (!sh_data.demaped_bits.empty())
                        {
                            ImPlot::PlotLine("RX", sh_data.demaped_bits.data(), sh_data.demaped_bits.size());
                            ImPlot::EndPlot();
                        }
                        else
                        {
                            ImPlot::Annotation(0.5, 0.5, ImVec4(1, 0, 0, 1), ImVec2(0, 0), true, "No Data");
                            ImPlot::EndPlot();
                        }
                    }
                }
            }
            ImGui::End();

            if (sh_data.flags.one_time_mode)
            {
                if (ImGui::Begin("Decoded Text"))
                {
                    ImGui::Text("Decoded text:");
                    ImGui::TextWrapped("%s", sh_data.dec_message.c_str());
                }
                ImGui::End();
            }
        }

        if (ImGui::BeginMainMenuBar())
        {
            if (ImGui::BeginMenu("Control Panel"))
            {
                ImGui::SeparatorText("Processing Blocks");

                const char *label_time = sh_data.flags.changed_cont_time ? "Programm | Running" : "Programm | Stopped";
                const char *sdr_mode = sh_data.flags.changed_send ? "SDR Mode | Transmission" : "SDR Mode | Receiving";
                const char *pss_mode = sh_data.flags.changed_pss_symbols ? "PSS Symbol [ON]" : "PSS Symbol [OFF]";
                const char *zadoff_chu = sh_data.flags.get_zadoff_pos ? "Direct Mode [ON]" : "Direct Mode [OFF]";
                const char *cfo_correct = sh_data.flags.cfo_cor ? "CFO Correction [ON]" : "CFO Correction [OFF]";
                const char *equal_mode = sh_data.flags.equal ? "Equalization [ON]" : "Equalization [OFF]";
                const char *check_bit = sh_data.flags.check_bits ? "Bits Check [ON]" : "Bits Check [OFF]";
                const char *modulation_type = nullptr;

                if (ImGui::Button(label_time, ImVec2(ImGui::GetContentRegionAvail().x, 0.f)))
                    sh_data.flags.changed_cont_time = !sh_data.flags.changed_cont_time;

                if (ImGui::Button(sdr_mode, ImVec2(ImGui::GetContentRegionAvail().x, 0.f)))
                    sh_data.flags.changed_send = !sh_data.flags.changed_send;

                if (ImGui::Button(pss_mode, ImVec2(ImGui::GetContentRegionAvail().x, 0.f)))
                    sh_data.flags.changed_pss_symbols = !sh_data.flags.changed_pss_symbols;

                if (ImGui::Button(check_bit, ImVec2(ImGui::GetContentRegionAvail().x / 2, 0.f)))
                    sh_data.flags.check_bits = !sh_data.flags.check_bits;

                ImGui::SameLine();

                if (ImGui::Button("Reset Error", ImVec2(ImGui::GetContentRegionAvail().x, 0.f)))
                    sh_data.err_cnt = 0;

                ImGui::SeparatorText("ZadOff-Chu");
                if (ImGui::Button("Loopback Mode", ImVec2(ImGui::GetContentRegionAvail().x, 0.f)))
                    sh_data.flags.get_zadoff_pos_loopback = !sh_data.flags.get_zadoff_pos_loopback;
                if (ImGui::Button(zadoff_chu, ImVec2(ImGui::GetContentRegionAvail().x, 0.f)))
                    sh_data.flags.get_zadoff_pos = !sh_data.flags.get_zadoff_pos;
                ImGui::InputInt("Sync Pos", &sh_data.sync_pos, 1, 1e1);
                ImGui::InputInt("Sync Offset", &sh_data.sync_offset, 1, 1e1);
                ImGui::InputInt("U Value ", &sh_data.zadoff_chu_u, 1, 10);

                ImGui::SeparatorText("DSP Module");
                if (ImGui::Button(cfo_correct, ImVec2(ImGui::GetContentRegionAvail().x, 0.f)))
                    sh_data.flags.cfo_cor = !sh_data.flags.cfo_cor;

                if (ImGui::Button(equal_mode, ImVec2(ImGui::GetContentRegionAvail().x, 0.f)))
                    sh_data.flags.equal = !sh_data.flags.equal;

                ImGui::SeparatorText("Pre Modulation");
                switch (sh_data.modul_type_TX)
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
                case ModulationType::QAM64:
                    modulation_type = "Modulation: QAM64";
                    break;
                case ModulationType::QAM256:
                    modulation_type = "Modulation: QAM256";
                    break;
                }

                ImGui::SetNextItemWidth(-FLT_MIN);
                if (ImGui::BeginCombo("##Modulation Type", modulation_type))
                {
                    if (ImGui::Selectable("BPSK", sh_data.modul_type_TX == ModulationType::BPSK))
                    {
                        sh_data.modul_type_TX = ModulationType::BPSK;
                        sh_data.flags.changed_modulation_type = true;
                    }
                    if (ImGui::Selectable("QPSK", sh_data.modul_type_TX == ModulationType::QPSK))
                    {
                        sh_data.modul_type_TX = ModulationType::QPSK;
                        sh_data.flags.changed_modulation_type = true;
                    }
                    if (ImGui::Selectable("QAM16", sh_data.modul_type_TX == ModulationType::QAM16))
                    {
                        sh_data.modul_type_TX = ModulationType::QAM16;
                        sh_data.flags.changed_modulation_type = true;
                    }
                    if (ImGui::Selectable("QAM64", sh_data.modul_type_TX == ModulationType::QAM64))
                    {
                        sh_data.modul_type_TX = ModulationType::QAM64;
                        sh_data.flags.changed_modulation_type = true;
                    }
                    if (ImGui::Selectable("QAM256", sh_data.modul_type_TX == ModulationType::QAM256))
                    {
                        sh_data.modul_type_TX = ModulationType::QAM256;
                        sh_data.flags.changed_modulation_type = true;
                    }
                    ImGui::EndCombo();
                }

                ImGui::InputInt("Cycle Prefix", &sh_data.cyclic_prefex, 1);
                ImGui::InputInt("Subcarrier", &sh_data.subcarrier, 1);

                ImGui::SeparatorText("SDR Configuration");
                ImGui::SetNextItemWidth(-FLT_MIN);
                if (ImGui::BeginCombo("##SDR Device", sh_data.device.c_str()))
                {
                    for (const auto &dev : sh_data.devices)
                    {
                        bool is_selected = (sh_data.device == dev);
                        if (ImGui::Selectable(dev.c_str(), is_selected))
                            sh_data.device = dev;
                        if (is_selected)
                            ImGui::SetItemDefaultFocus();
                    }
                    ImGui::EndCombo();
                }

                if (ImGui::Button("RX Mode", ImVec2(ImGui::GetContentRegionAvail().x / 2, 0.0f)))
                {
                    sh_data.flags.get_zadoff_pos = !sh_data.flags.get_zadoff_pos;
                    sh_data.flags.rx_gain_mode = !sh_data.flags.rx_gain_mode;
                    sh_data.flags.changed_rx_gain_mode = true;
                    sh_data.tx_gain = 0.0f;
                    sh_data.flags.cfo_cor = !sh_data.flags.cfo_cor;
                    sh_data.flags.equal = !sh_data.flags.equal;
                    sh_data.flags.check_bits = !sh_data.flags.check_bits;
                    sh_data.flags.debug = !sh_data.flags.debug;
                }

                ImGui::SameLine();

                if (ImGui::Button("TX Mode", ImVec2(ImGui::GetContentRegionAvail().x, 0.0f)))
                {
                    sh_data.flags.changed_send = !sh_data.flags.changed_send;
                    sh_data.flags.changed_pss_symbols = !sh_data.flags.changed_pss_symbols;
                    sh_data.flags.get_zadoff_pos = !sh_data.flags.get_zadoff_pos;
                    sh_data.flags.rx_gain_mode = !sh_data.flags.rx_gain_mode;
                    sh_data.flags.changed_rx_gain_mode = true;
                    sh_data.tx_gain = 89.0f;
                    sh_data.flags.cfo_cor = !sh_data.flags.cfo_cor;
                    sh_data.flags.equal = !sh_data.flags.equal;
                    sh_data.flags.check_bits = !sh_data.flags.check_bits;
                    sh_data.flags.debug = !sh_data.flags.debug;
                }

                if (ImGui::DragFloat("RX Gain", &sh_data.rx_gain, 0.25f, 0.f, 73.f))
                {
                    sh_data.flags.changed_rx_gain = true;
                    sh_data.flags.rx_gain_mode = false;
                    sh_data.flags.changed_rx_gain_mode = true;
                }
                ImGui::SameLine();
                if (ImGui::Checkbox("AGC", &sh_data.flags.rx_gain_mode))
                    sh_data.flags.changed_rx_gain_mode = true;

                if (ImGui::DragFloat("TX Gain", &sh_data.tx_gain, 0.25f, 0.f, 89.f))
                    sh_data.flags.changed_tx_gain = true;

                if (ImGui::InputFloat("RX Frequency", &sh_data.rx_frequency, 1e2, 1e3))
                    sh_data.flags.changed_rx_freq = true;

                if (ImGui::InputFloat("TX Frequency", &sh_data.tx_frequency, 1e2, 1e3))
                    sh_data.flags.changed_tx_freq = true;

                if (ImGui::SliderInt("RX Bandwidth", &cur_rx_bandwidth, 0, bandwidths.size() - 1,
                                     std::to_string(bandwidths[cur_rx_bandwidth]).c_str()))
                {
                    sh_data.rx_bandwidth = bandwidths[cur_rx_bandwidth];
                    sh_data.flags.changed_rx_bandwidth = true;
                }

                if (ImGui::SliderInt("TX Bandwidth", &cur_tx_bandwidth, 0, bandwidths.size() - 1,
                                     std::to_string(bandwidths[cur_tx_bandwidth]).c_str()))
                {
                    sh_data.tx_bandwidth = bandwidths[cur_tx_bandwidth];
                    sh_data.flags.changed_tx_bandwidth = true;
                }

                if (ImGui::InputFloat("Sample Rate", &sh_data.sample_rate, 1e5, 1e6))
                    sh_data.flags.changed_sample_rate = true;

                if (ImGui::Button("Open Message Editor", ImVec2(ImGui::GetContentRegionAvail().x, 0.0f)))
                    sh_data.flags.show_input_window = true;

                if (sh_data.flags.show_input_window)
                {
                    ImGui::Begin("Input Message", &sh_data.flags.show_input_window);

                    if (ImGui::InputText("Text", sh_data.input_buffer, 256))
                        sh_data.message = sh_data.input_buffer;

                    if (ImGui::Button("Close"))
                        sh_data.flags.show_input_window = false;

                    ImGui::End();
                }
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Debug"))
            {
                if (ImGui::MenuItem("Debug Mode", nullptr, sh_data.flags.debug))
                    sh_data.flags.debug = !sh_data.flags.debug;
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Send Mode"))
            {
                if (ImGui::MenuItem("Constant Mode", nullptr, sh_data.flags.constant_mode))
                {
                    sh_data.flags.constant_mode = true;
                    sh_data.flags.one_time_mode = false;
                }
                if (ImGui::MenuItem("One-Time Mode", nullptr, sh_data.flags.one_time_mode))
                {
                    sh_data.flags.one_time_mode = true;
                    sh_data.flags.constant_mode = false;
                }
                ImGui::EndMenu();
            }

            float window_width = ImGui::GetWindowWidth();
            ImGui::SameLine(window_width - ImGui::CalcTextSize("FPS: 0000 (0000.000 ms)").x);
            ImGui::Text("FPS: %.f (%.3f ms)", io.Framerate, 1000.0f / io.Framerate);

            ImGui::EndMainMenuBar();
        }

        // End GUI

        ImGui::Render();

        int w, h;
        glfwGetFramebufferSize(window, &w, &h);
        glViewport(0, 0, w, h);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);

        if (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
        {
            GLFWwindow* backup = glfwGetCurrentContext();
            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();
            glfwMakeContextCurrent(backup);
        }
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return;
}
