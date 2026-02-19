#pragma once

#include <vector>
#include <complex>
#include <cstdint>
#include <cmath>

enum class ModulationType {
    BPSK,
    QPSK,
    QAM16
};

/**
 * @brief Modulation mapper according to 3GPP TS 38.211 specification
 *
 * @param bits Input bit sequence
 * @param symbols Output std::complex symbols
 * @param modulation_type Type of modulation to use
 */

void modulate(const std::vector<int16_t>& bits, std::vector<std::complex<double>>& symbols, ModulationType modulation_type);

 /**
  * @brief Upsample function
  *
  * @param symbols Input std::complex symbols
  * @param symbols Output std::complex symbols_ups
  * @param L Upsample factor
  */

void UpSampler(const std::vector<std::complex<double>>& symbols, std::vector<std::complex<double>>& symbols_ups, int L);

 /**
  * @brief Filter function
  *
  * @param symbols_ups Input std::complex symbols
  * @param impulse Impulse response
  */

void filter(std::vector<std::complex<double>>& symbols_ups, const std::vector<std::complex<double>>& impulse);

 /**
  * @brief Bits function
  *
  * @param type Type of modulation
  */

int bits_per_symbol(ModulationType type);

void filter_int16_t(std::vector<int16_t>& symbols_ups, const std::vector<int16_t>& impulse, std::vector<int16_t>& output);

void norm(std::vector<double>& rx_buffer_after_convolve);

void symbols_sync(const std::vector<double>& rx_buffer_after_convolve, std::vector<int>& offset, double& BnTs, double& Kp, int& Nsp);
