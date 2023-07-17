#ifndef INTERPOLATOR_HPP
#define INTERPOLATOR_HPP

#include <cmath>
#include <numeric>
#include <vector>

#include "utils.hpp"

namespace loudness {
    /**
     * @brief Finds interpolated peaks with given factor
     *
     * @attention Does not find peaks in original samples, only the interpolated samples
     */
    class Interpolator {
        /* Polyphase FIR interpolator */
    public:
        Interpolator(unsigned int taps, unsigned int factor, unsigned int channels);

        template <ConstData T>
        void process(T* in, std::size_t frames, std::size_t chan);
        [[nodiscard]] double peak(unsigned int channel) const;

        [[nodiscard]] unsigned int factor() const { return factor_; }
        [[nodiscard]] unsigned int channels() const { return channels_; }

    private:
        struct InterpFilter {
            std::vector<std::vector<unsigned int>> indicies;
            std::vector<double> coeff;
        };

        struct ChannelData {
            std::vector<float> buffer;
            size_t index;
            double peak;
        };

        const unsigned int factor_;   /* Interpolation factor of the interpolator */
        const unsigned int channels_; /* Number of channels */
        const unsigned int delay_;    /* Size of delay buffer */
        std::vector<InterpFilter> filter_;
        std::vector<ChannelData> chan_data_;
    };

    template <ConstData T>
    void Interpolator::process(T* in, std::size_t frames, std::size_t chan)
    {
        auto& [buffer, buf_i, peak] = chan_data_[chan];
        peak = 0.0;
        if constexpr (not std::is_pointer_v<T>){
            in += chan;
        }
        for (std::size_t frame = 0; frame < frames; frame++) {
                /* Add sample to delay buffer */
                if constexpr (std::is_pointer_v<T>){
                    using U = std::remove_pointer_t<T>;
                    if constexpr (std::is_floating_point_v<U>){
                        buffer[buf_i] = static_cast<float>(in[chan][frame]);
                    } else {
                        buffer[buf_i] = static_cast<float>(static_cast<double>(in[chan][frame]) / getScalingFactor<U>());
                    }
                } else {
                    if constexpr (std::is_floating_point_v<T>){
                        buffer[buf_i] = static_cast<float>(*in);
                    } else {
                        buffer[buf_i] = static_cast<float>(static_cast<double>(*in) / getScalingFactor<T>());
                    }
                    in += channels_;
                }

                /* Apply coefficients */
                // Skip first factor, as it is the same as the original sample, so caught by sample peak
                for (unsigned int f = 1; f < factor_; f++) {
                    double acc = std::transform_reduce(
                                filter_[f].coeff.cbegin(),
                                filter_[f].coeff.cend(),
                                filter_[f].indicies[buf_i].cbegin(),
                                0.0,
                                std::plus<>(),
                                [&buffer](auto coeff, auto index){ return static_cast<double>(buffer[index]) * coeff;}
                                );
                    acc = std::abs(acc);
                    if (acc > peak){
                        peak = acc;
                    }
                }
            ++buf_i;
            if (buf_i == delay_) {
                buf_i = 0;
            }
        }
    }
} // namespace loudness
#endif  // INTERPOLATOR_HPP
