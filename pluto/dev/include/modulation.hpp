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

void filter_double(std::vector<double>& symbols_ups, const std::vector<double>& impulse, std::vector<double>& output);

void norm(std::vector<double>& rx_buff);

void norm_after_conv(std::vector<double>& rx, int sps);


class GardnerState {
public:
    double mu = 0.0;
    double omega;
    double integrator = 0.0;

    void gardner_step(std::vector<double>& real_pa, std::vector<double>& imag_pa, std::vector<int>& offset, double& BnTs, int& Nsp);
};

struct CostasState {
    double phase = 0.0;
    double freq = 0.0;

    void costas_step(double& I_orig, double& Q_orig, double& I_new, double& Q_new, double& Kp, double& Ki, ModulationType& mod_type);
    double get_phase();
    double get_freq();
    double QAM16slicer(double x);
    void reset_costas_state();
};
