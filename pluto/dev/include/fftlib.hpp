#pragma once

#include <fftw3.h>

struct FFT_Context
{
    int N;
    fftwf_complex *in;
    fftwf_complex *out;
    fftwf_plan plan_forward;
    fftwf_plan plan_backward;

    FFT_Context(int n) : N(n)
    {
        in = fftwf_alloc_complex(N);
        out = fftwf_alloc_complex(N);

        plan_forward = fftwf_plan_dft_1d(N, in, out, FFTW_FORWARD, FFTW_MEASURE);
        plan_backward = fftwf_plan_dft_1d(N, in, out, FFTW_BACKWARD, FFTW_MEASURE);
    }

    ~FFT_Context()
    {
        fftwf_destroy_plan(plan_forward);
        fftwf_destroy_plan(plan_backward);
        fftwf_free(in);
        fftwf_free(out);
    }
};

void fft(FFT_Context &context);

void ifft(FFT_Context &context);