#include "common.hpp"

FFT_Context::FFT_Context(int n) : N(n)
{
    in = fftwf_alloc_complex(N);
    out = fftwf_alloc_complex(N);

    plan_forward = fftwf_plan_dft_1d(N, in, out, FFTW_FORWARD, FFTW_MEASURE);
    plan_backward = fftwf_plan_dft_1d(N, in, out, FFTW_BACKWARD, FFTW_MEASURE);
}

FFT_Context::~FFT_Context()
{
    fftwf_destroy_plan(plan_forward);
    fftwf_destroy_plan(plan_backward);
    fftwf_free(in);
    fftwf_free(out);
}

void FFT_Context::fft()
{
    fftwf_execute(plan_forward);
}

void FFT_Context::ifft()
{
    fftwf_execute(plan_backward);

    float scale = 1.0f / N;
    for (int i = 0; i < N; ++i)
    {
        out[i][0] *= scale;
        out[i][1] *= scale;
    }
}
