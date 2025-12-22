#pragma once

#include <vector>
#include <complex>
#include <cstdint>
#include <cmath>

using namespace std;
enum class ModulationType {
    BPSK,
    QPSK,
    QAM16
};

/**
 * @brief Modulation mapper according to 3GPP TS 38.211 specification
 *
 * @param bits Input bit sequence
 * @param symbols Output complex symbols
 * @param modulation_type Type of modulation to use
 */

 void modulate(const vector<int16_t>& bits, vector<complex<double>>& symbols, ModulationType modulation_type);

 /**
  * @brief Upsample function
  *
  * @param symbols Input complex symbols
  * @param symbols Output complex symbols_ups
  * @param L Upsample factor
  */

 void UpSampler(const vector<complex<double>>& symbols, vector<complex<double>>& symbols_ups, int L);

 /**
  * @brief Upsample function
  *
  * @param symbols_ups Input complex symbols
  * @param impulse Impulse response
  */

 void filter(vector<complex<double>>& symbols_ups, const vector<complex<double>>& impulse);
