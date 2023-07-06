#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include "utils.hpp"
#include <gcem.hpp>

namespace loudness {
    constexpr double relative_gate = -10.0;
    constexpr double minus_twenty_decibels = gcem::pow(10.0, -20.0 / 10.0);
    constexpr double relative_gate_factor = gcem::pow(10.0, relative_gate / 10.0);
    constexpr double abs_threshold = -70.0;
    constexpr double absolute_gate = loudnessToEnergy(abs_threshold);

    constexpr int momentary_block_ms = 400;
    constexpr int shortterm_block_ms = 3000;
    static_assert(shortterm_block_ms >= momentary_block_ms);

    constexpr int m_subblocks = momentary_block_ms / 100;
    constexpr int st_subblocks = shortterm_block_ms / 100;

    constexpr int m_block_overlap_ms = momentary_block_ms / 4;
    constexpr int st_block_overlap_ms = shortterm_block_ms / 3;
} // namespace loudness

#endif // CONSTANTS_HPP
