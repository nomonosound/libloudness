#include "interpolator.hpp"

#include <cmath>
#include <numbers>

#include "utils.hpp"
#include <cassert>

namespace loudness {
    namespace {
        static constexpr double almost_zero = 0.000001;

        auto createFilter(unsigned int taps, unsigned int factor) {
            std::vector<std::vector<double>> filter(factor - 1);
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
                filter[f-1].push_back(c);
            }
            return filter;
        }
    }
    Interpolator::Interpolator(unsigned int taps, unsigned int factor, unsigned int channels)
        : filter_(createFilter(taps, factor)),
          chan_data_(channels, {.buffer = std::vector<float>((taps - 1) / factor), .index = 0, .peak = 0.0})
    {
        assert(taps % 2 == 1); // Some optimizations assume odd number of taps
    }

    double Interpolator::peak(unsigned int channel) const {
        assert(channel < chan_data_.size());
        return chan_data_[channel].peak;
    }
} // namespace loudness
