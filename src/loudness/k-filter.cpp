/* See LICENSE file for copyright and license details. */

#include "k-filter.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <gcem.hpp>
#include <numbers>
#include <numeric>

namespace loudness {
    KFilter::KFilter(double samplerate, unsigned int channels) : v_(channels, filter_state{})
    {
        assert(samplerate > 0);
        constexpr double f0 = 1681.974450955533;
        constexpr double G = 3.999843853973347;
        constexpr double Vh = gcem::pow(10.0, G / 20.0);
        constexpr double Vb = gcem::pow(Vh, 0.4996667741545416);
        constexpr double Q = 0.7071752369554196;

        double K = std::tan(std::numbers::pi * f0 / samplerate);

        const double a0 = 1.0 + K / Q + K * K;

        const std::array<double, 3> pb{(Vh + Vb * K / Q + K * K) / a0, 2.0 * (K * K - Vh) / a0,
                                       (Vh - Vb * K / Q + K * K) / a0};

        const std::array<double, 3> pa = {1.0, 2.0 * (K * K - 1.0) / a0, (1.0 - K / Q + K * K) / a0};

        constexpr std::array<double, 3> rb = {1.0, -2.0, 1.0};

        b_[0] = pb[0] * rb[0];
        b_[1] = pb[0] * rb[1] + pb[1] * rb[0];
        b_[2] = pb[0] * rb[2] + pb[1] * rb[1] + pb[2] * rb[0];
        b_[3] = pb[1] * rb[2] + pb[2] * rb[1];
        b_[4] = pb[2] * rb[2];

        constexpr double f0_2 = 38.13547087602444;
        constexpr double Q_2 = 0.5003270373238773;
        K = std::tan(std::numbers::pi * f0_2 / samplerate);

        const std::array<double, 3> ra = {1.0, 2.0 * (K * K - 1.0) / (1.0 + K / Q_2 + K * K),
                                          (1.0 - K / Q_2 + K * K) / (1.0 + K / Q_2 + K * K)};

        a_[0] = pa[0] * ra[0];
        a_[1] = pa[0] * ra[1] + pa[1] * ra[0];
        a_[2] = pa[0] * ra[2] + pa[1] * ra[1] + pa[2] * ra[0];
        a_[3] = pa[1] * ra[2] + pa[2] * ra[1];
        a_[4] = pa[2] * ra[2];
    }

    double KFilter::apply(double src, std::size_t channel)
    {
        // NB! Critical inner loop, benchmark when modifying anything
        v_[channel][4] = v_[channel][3];
        v_[channel][3] = v_[channel][2];
        v_[channel][2] = v_[channel][1];
        v_[channel][1] = v_[channel][0];
        v_[channel][0] = src -
            a_[1] * v_[channel][1] -
            a_[2] * v_[channel][2] -
            a_[3] * v_[channel][3] -
            a_[4] * v_[channel][4];
        return
            b_[0] * v_[channel][0] +
            b_[1] * v_[channel][1] +
            b_[2] * v_[channel][2] +
            b_[3] * v_[channel][3] +
            b_[4] * v_[channel][4];
    }

    void KFilter::manuallyFTZ(std::size_t channel)
    {
        v_[channel][4] = std::abs(v_[channel][4]) < std::numeric_limits<double>::min() ? 0.0 : v_[channel][4];
        v_[channel][3] = std::abs(v_[channel][3]) < std::numeric_limits<double>::min() ? 0.0 : v_[channel][3];
        v_[channel][2] = std::abs(v_[channel][2]) < std::numeric_limits<double>::min() ? 0.0 : v_[channel][2];
        v_[channel][1] = std::abs(v_[channel][1]) < std::numeric_limits<double>::min() ? 0.0 : v_[channel][1];
    }
}  // namespace loudness
