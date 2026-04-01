#pragma once

#include <cstdint>

struct PRBS15 {
    uint16_t state = 0xACE1;

    int get_bit() {
        int new_bit = ((state >> 14) ^ (state >> 13)) & 1;
        state = (state << 1) | new_bit;
        return new_bit & 1;
    }
};
