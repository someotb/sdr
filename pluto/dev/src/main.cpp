#include "gui.hpp"
#include "sdr.hpp"
#include "dsp.hpp"
#include "common.hpp"

#include <thread>

int main()
{
    sharedData sd(1920);

    std::jthread sdr_thread(run_sdr, std::ref(sd));
    std::jthread dsp_thread(run_dsp, std::ref(sd));
    run_gui(sd);
    return 0;
}
