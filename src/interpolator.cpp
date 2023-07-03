#include "interpolator.hpp"

#include <cmath>
#include <numbers>

#include "utils.hpp"
#include <cassert>

static constexpr double ALMOST_ZERO = 0.000001;

Interpolator::Interpolator(unsigned int taps, unsigned int factor, unsigned int channels)
    : factor_(factor),
      channels_(channels),
      delay_((taps + factor - 1) / factor),
      z_(std::vector<std::vector<float>>(channels, std::vector<float>(delay_))),
      peaks_(channels)
{
    assert(taps % 2 == 1); // Assert odd number of taps

    /* Initialize the filter memory
     * One subfilter per interpolation factor. */
    filter_ = std::vector<InterpFilter>(
                factor, {.indicies = {std::vector<unsigned int>()}, .coeff = {}});

    /* Calculate the filter coefficients */
    for (unsigned int j = 0; j < taps; j++) {
        /* Calculate sinc */
        const double m = static_cast<double>(j) - static_cast<double>(taps - 1) / 2.0;
        double c = std::abs(m) > ALMOST_ZERO ? sinc(m * std::numbers::pi / factor) : 1.0;

        /* Apply Hanning window */
        c *= 0.5 * (1 - std::cos(2 * std::numbers::pi * j / (taps - 1)));

        if (std::abs(c) > ALMOST_ZERO) { /* Ignore any zero coeffs. */
            /* Put the coefficient into the correct subfilter */
            const unsigned int f = j % factor;
            filter_[f].coeff.push_back(c);
            filter_[f].indicies[0].push_back(j / factor);
        }
    }
    // Precalculate buffer indicies for each tap for each buffer index
    for (auto& filter : filter_){
        std::vector<std::vector<unsigned int>> indicies(delay_);
        for (size_t i = 0; i < delay_; ++i){
            for (size_t t = 0; t < filter.coeff.size(); ++t){
                if (filter.indicies[0][t] > i){
                    indicies[i].push_back(i + delay_ - filter.indicies[0][t]);
                } else {
                    indicies[i].push_back(i - filter.indicies[0][t]);
                }
            }
        }
        filter.indicies = indicies;
    }
}

double Interpolator::peak(unsigned int channel) const {
    assert(channel < channels_);
    return peaks_[channel];
}
