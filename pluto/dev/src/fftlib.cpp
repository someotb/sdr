#include "fftlib.hpp"

void fft(FFT_Context &context)
{
    fftwf_execute(context.plan_forward);
}

void ifft(FFT_Context &context)
{
    fftwf_execute(context.plan_backward);

    float scale = 1.0f / context.N;
    #pragma omp simd
    for (int i = 0; i < context.N; ++i)
    {
        context.out[i][0] *= scale;
        context.out[i][1] *= scale;
    }
}