#include "modulation.h"

void modulate(const vector<int16_t>& bits, vector<complex<double>>& symbols, ModulationType modulation_type) {
    switch (modulation_type) {
        case ModulationType::QPSK:
            {
                // According to 3GPP TS 38.211 section 5.1.3:
                // d(i) = 1/sqrt(2) * (1 - 2*b(2i) + j*(1 - 2*b(2i+1)))
                symbols.resize(bits.size() / 2);

                for (size_t i = 0; i < bits.size(); i += 2) {
                    double real = (1.0 - 2.0 * bits[i]) / sqrt(2.0);
                    double imag = (1.0 - 2.0 * bits[i+1]) / sqrt(2.0);
                    symbols[i/2] = complex<double>(real, imag);
                }
            }
            break;

        case ModulationType::BPSK:
            // Will be implemented later
            throw runtime_error("BPSK modulation is not implemented yet");

        case ModulationType::QAM16:
            // Will be implemented later
            throw runtime_error("16QAM modulation is not implemented yet");

        default:
            throw invalid_argument("Unsupported modulation type");
    }
}

void UpSampler(const vector<complex<double>>& symbols, vector<complex<double>>& symbols_ups, int L) {
    const complex<double> i0 (0, 0);
    size_t len_symbols = symbols.size();
    symbols_ups.assign(len_symbols * L, i0);
    for (size_t i = 0; i < len_symbols; i++) {
        symbols_ups[i * L] = symbols[i];
    }
}

void filter(vector<complex<double>>& symbols_ups, const vector<complex<double>>& impulse) {
    size_t n = symbols_ups.size();
    size_t L = impulse.size();
    vector<complex<double>> output(n, 0.0);

    for (size_t i = 0; i < n; i++) {
        size_t max_j = min(L, i + 1);
        for (size_t j = 0; j < max_j; j++) {
            output[i] += impulse[j] * symbols_ups[i - j];
        }
    }

    symbols_ups.swap(output);
}
