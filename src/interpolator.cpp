#include "interpolator.hpp"

#include <cmath>
#include <numbers>

#include "utils.hpp"
#include <cassert>

static constexpr double ALMOST_ZERO = 0.000001;

namespace loudness {
    Interpolator::Interpolator(unsigned int taps, unsigned int factor, unsigned int channels)
        : factor_(factor),
          channels_(channels),
          delay_((taps - 1 + factor) / factor),
          z_(std::vector<std::vector<float>>(channels, std::vector<float>(delay_))),
          zi_(channels),
          peaks_(channels)
    {
        assert(taps % 2 == 1); // Assert odd number of taps

        /* Initialize the filter memory
         * One subfilter per interpolation factor. */
        filter_ = std::vector<InterpFilter>(
            factor, {.indicies = std::vector<std::vector<unsigned int>>(delay_), .coeff = {}});

        /* Calculate the filter coefficients */
        for (unsigned int j = 0; j < taps; j++) {
            /* Calculate sinc */
            const double m = static_cast<double>(j) - static_cast<double>(taps - 1) / 2.0;
            double c = std::abs(m) > ALMOST_ZERO ? sinc(m * std::numbers::pi / factor) : 1.0;

            /* Apply Hann window */
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
            for (size_t i = delay_; i-- > 0;){
                filter.indicies[i].resize(filter.coeff.size());
                for (size_t t = 0; t < filter.coeff.size(); ++t){
                    filter.indicies[i][t] = filter.indicies[0][t] > i ? i + delay_ - filter.indicies[0][t] : i - filter.indicies[0][t];
                }
            }
        }
    }

    double Interpolator::peak(unsigned int channel) const {
        assert(channel < channels_);
        return peaks_[channel];
    }
} // namespace loudness
