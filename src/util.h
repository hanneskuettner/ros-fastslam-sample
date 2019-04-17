#pragma once

#include <cmath>

static inline float clamp_angle_pi_pi(float angle) {
    return static_cast<float>(angle - 2 * M_PI * floor((angle + M_PI) / (2 * M_PI)));
}