#include "interpolator.hpp"

#include <cmath>
#include <numbers>

#include "utils.hpp"

static constexpr double ALMOST_ZERO = 0.000001;

Interpolator::Interpolator(unsigned int taps, unsigned int factor, unsigned int channels)
    : factor_(factor),
      channels_(channels),
      delay_((taps + factor - 1) / factor),
      out_stride_(channels * factor),
      zi_(0),
      z_(std::vector<std::vector<float>>(channels, std::vector<float>(delay_)))
{
    /* Initialize the filter memory
     * One subfilter per interpolation factor. */
    filter_ = std::vector<InterpFilter>(
        factor, {.index = std::vector<unsigned int>(delay_), .coeff = std::vector<double>(delay_)});

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
            const unsigned int t = filter_[f].count++;
            filter_[f].coeff[t] = c;
            filter_[f].index[t] = j / factor;
        }
    }
}

size_t Interpolator::process(size_t frames, const float* in, float* out)
{
    float* outp = nullptr;
    double acc = 0;
    double c = 0;

    for (size_t frame = 0; frame < frames; frame++) {
        for (unsigned int chan = 0; chan < channels_; chan++) {
            /* Add sample to delay buffer */
            z_[chan][zi_] = *in++;
            /* Apply coefficients */
            outp = out + chan;
            for (unsigned int f = 0; f < factor_; f++) {
                acc = 0.0;
                for (unsigned int t = 0; t < filter_[f].count; t++) {
                    int i = static_cast<int>(zi_) - static_cast<int>(filter_[f].index[t]);
                    if (i < 0) {
                        i += static_cast<int>(delay_);
                    }
                    c = filter_[f].coeff[t];
                    acc += static_cast<double>(z_[chan][i]) * c;
                }
                *outp = static_cast<float>(acc);
                outp += channels_;
            }
        }
        out += out_stride_;
        zi_++;
        if (zi_ == delay_) {
            zi_ = 0;
        }
    }

    return frames * factor_;
}
