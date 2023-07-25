/* See LICENSE file for copyright and license details. */

#include "interpolator.hpp"

#include <cassert>
#include <cmath>
#include <numbers>
#include <ranges>

#include "utils.hpp"

namespace loudness {
    namespace {
        auto createFilter(unsigned int taps, unsigned int factor)
        {
            static constexpr double almost_zero = 0.000001;
            std::vector<std::vector<float>> filter(factor - 1);
            for (unsigned int j = taps; j-- > 0;) {
                const unsigned int f = j % factor;
                /* Skip first factor, as it just equals an input sample */
                if (f == 0) continue;

                /* Calculate sinc */
                const double m = static_cast<double>(j) - static_cast<double>(taps - 1) / 2.0;
                double c = std::abs(m) > almost_zero ? sinc(m * std::numbers::pi / factor) : 1.0;

                /* Apply Hann window */
                c *= 0.5 * (1 - std::cos(2 * std::numbers::pi * j / (taps - 1)));

                /* Put the coefficient into the correct subfilter.
                 * First factor is skipped, none of the others have zeros.
                 */
                filter[f - 1].push_back(static_cast<float>(c));
            }
            return filter;
        }
    }  // namespace

    Interpolator::Interpolator(unsigned int taps, unsigned int factor, unsigned int channels)
        : filter_(createFilter(taps, factor)),
          chan_data_(channels, {.buffer = std::vector<float>((taps - 1) / factor), .index = 0, .peak = 0.0})
    {
        assert(taps % 2 == 1);  // Some optimizations assume odd number of taps
    }

    double Interpolator::peak(unsigned int channel) const
    {
        assert(channel < chan_data_.size());
        return chan_data_[channel].peak;
    }

    double Interpolator::peak() const
    {
        return std::ranges::max_element(chan_data_, [](const auto& lhs, const auto& rhs){return lhs.peak < rhs.peak;})->peak;
    }
}  // namespace loudness
