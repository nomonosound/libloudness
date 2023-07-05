#ifndef INTERPOLATOR_HPP
#define INTERPOLATOR_HPP

#include <vector>
#include <cmath>

#include "utils.hpp"
#include "defines.hpp"
#include <numeric>

class Interpolator {
    /* Polyphase FIR interpolator */
public:
    Interpolator(unsigned int taps, unsigned int factor, unsigned int channels);
    template <ConstData T>
    void process(T* in, std::size_t frames);
    double peak(unsigned int channel) const;
    void clearPeaks() {peaks_.assign(channels_, 0.0);}
    unsigned int factor() const { return factor_; };

private:
    struct InterpFilter {
        std::vector<std::vector<unsigned int>> indicies;
        std::vector<double> coeff;
    };

    const unsigned int factor_;   /* Interpolation factor of the interpolator */
                                  //    unsigned int taps_;     /* Taps (prefer odd to increase zero coeffs) */
    const unsigned int channels_; /* Number of channels */
    const unsigned int delay_;    /* Size of delay buffer */
    std::vector<InterpFilter> filter_;
    std::vector<std::vector<float>> z_; /* List of delay buffers (one for each channel) */
    unsigned int zi_{0};                /* Current delay buffer index */

    std::vector<double> peaks_;
};

template <ConstData T>
void Interpolator::process(T* in, std::size_t frames)
{
    for (std::size_t frame = 0; frame < frames; frame++) {
        for (unsigned int chan = 0; chan < channels_; chan++) {
            /* Add sample to delay buffer */
            if constexpr (std::is_pointer_v<T>){
                using U = std::remove_pointer_t<T>;
                if constexpr (std::is_floating_point_v<U>){
                    z_[chan][zi_] = static_cast<float>(in[chan][frame]);
                } else {
                    z_[chan][zi_] = static_cast<float>(static_cast<double>(in[chan][frame]) / getScalingFactor<U>());
                }
            } else {
                if constexpr (std::is_floating_point_v<T>){
                    z_[chan][zi_] = static_cast<float>(*in++);
                } else {
                    z_[chan][zi_] = static_cast<float>(static_cast<double>(*in++) / getScalingFactor<T>());
                }
            }

            /* Apply coefficients */
            // Skip first factor, as it is the same as the original sample, so caught by sample peak
            for (unsigned int f = 1; f < factor_; f++) {
                double acc = std::transform_reduce(
                            filter_[f].coeff.cbegin(),
                            filter_[f].coeff.cend(),
                            filter_[f].indicies[zi_].cbegin(),
                            0.0,
                            std::plus<>(),
                            [this, chan](auto coeff, auto index){ return static_cast<double>(z_[chan][index]) * coeff;}
                            );
                acc = std::abs(acc);
                if (acc > peaks_[chan]){
                    peaks_[chan] = acc;
                }
            }
        }
        ++zi_;
        if (zi_ == delay_) {
            zi_ = 0;
        }
    }
}
#endif  // INTERPOLATOR_HPP
