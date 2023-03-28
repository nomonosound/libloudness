#include "k-filter.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <numeric>

KFilter::KFilter(double samplerate, unsigned int channels) : v_(channels, filter_state{})
{
    // TODO: assert samplerate > 0
    constexpr double f0 = 1681.974450955533;
    constexpr double G = 3.999843853973347;
    constexpr double Q = 0.7071752369554196;

    double K = std::tan(std::numbers::pi * f0 / samplerate);
    const double Vh = std::pow(10.0, G / 20.0);
    const double Vb = std::pow(Vh, 0.4996667741545416);

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

double KFilter::apply(double src, unsigned int channel)
{
    std::shift_right(v_[channel].begin(), v_[channel].end(), 1);
    v_[channel][0] =
        src - std::transform_reduce(std::next(a_.cbegin()), a_.cend(), std::next(v_[channel].cbegin()), 0.0);
    return std::transform_reduce(b_.cbegin(), b_.cend(), v_[channel].cbegin(), 0.0);
}
