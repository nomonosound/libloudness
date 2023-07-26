/* See LICENSE file for copyright and license details. */

#ifndef LIBLOUDNESS_CONSTANTS_HPP
#define LIBLOUDNESS_CONSTANTS_HPP

#include <gcem.hpp>

#include "loudness/utils.hpp"

namespace loudness {
    constexpr double relative_gate_LU = -10.0;
    constexpr double absolute_gate_LUFS = -70.0;

    constexpr double absolute_gate = loudnessToEnergy(absolute_gate_LUFS);
    constexpr double relative_gate_factor = gcem::pow(10.0, relative_gate_LU / 10.0);
    constexpr double minus_twenty_decibels = gcem::pow(10.0, -20.0 / 10.0);

    constexpr int milliseconds_in_second = 1000;

    constexpr int subblocks_in_s = 10;
    constexpr int subblock_ms = milliseconds_in_second / subblocks_in_s;
    static_assert(milliseconds_in_second % subblocks_in_s == 0);

    constexpr int m_subblocks = 4;
    constexpr int st_subblocks = 30;
    /* Overlap between blocks is 1 subblock, size needs to be larger than that */
    static_assert(m_subblocks > 1);
    static_assert(st_subblocks >= m_subblocks);

    constexpr int momentary_block_ms = m_subblocks * subblock_ms;
    constexpr int shortterm_block_ms = st_subblocks * subblock_ms;

    constexpr int block_overlap_ms = subblock_ms;
}  // namespace loudness

#endif  // LIBLOUDNESS_CONSTANTS_HPP
