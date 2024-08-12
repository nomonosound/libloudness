/* See LICENSE file for copyright and license details. */

#ifndef LIBLOUDNESS_INTERPOLATOR_HPP
#define LIBLOUDNESS_INTERPOLATOR_HPP

#include <cmath>
#include <numeric>
#include <span>
#include <vector>

#include "loudness/utils.hpp"

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
        void process(T* in_data, std::size_t frames, std::size_t chan);
        [[nodiscard]] double peak(unsigned int channel) const;
        [[nodiscard]] double peak() const;

        [[nodiscard]] unsigned int channels() const { return static_cast<unsigned int>(chan_data_.size()); }

    private:
        struct ChannelData {
            std::vector<float> buffer;
            std::size_t index;
            float peak;
        };

        const std::vector<std::vector<float>> filter_;
        std::vector<ChannelData> chan_data_;
    };

    template <ConstData T>
    void Interpolator::process(T* in_data, std::size_t frames, std::size_t chan)
    {
        const auto buffer = std::span(chan_data_[chan].buffer);
        /* Have these variables on the local thread's stack for better optimization */
        float peak = 0.f;
        auto buf_i = chan_data_[chan].index;

        if constexpr (not std::is_pointer_v<T>) {
            in_data += chan;
        }
        for (std::size_t frame = 0; frame < frames; ++frame) {
            /* Add sample to delay buffer */
            if constexpr (std::is_pointer_v<T>) {
                using U = std::remove_pointer_t<T>;
                if constexpr (std::floating_point<U>) {
                    buffer[buf_i] = static_cast<float>(in_data[chan][frame]);
                }
                else {
                    buffer[buf_i] =
                        static_cast<float>(static_cast<double>(in_data[chan][frame]) / getScalingFactor<U>());
                }
            }
            else {
                if constexpr (std::floating_point<T>) {
                    buffer[buf_i] = static_cast<float>(*in_data);
                }
                else {
                    buffer[buf_i] = static_cast<float>(static_cast<double>(*in_data) / getScalingFactor<T>());
                }
                in_data += chan_data_.size();
            }

            /* Apply coefficients */
            for (const auto& coeffs : filter_) {
                /* The accumulation is split over the seam in the circular buffer */
                float acc = std::transform_reduce(buffer.begin() + buf_i + 1, buffer.end(), coeffs.cbegin(),
                                                  std::transform_reduce(coeffs.cbegin() + coeffs.size() - 1 - buf_i,
                                                                        coeffs.cend(), buffer.begin(), 0.f));
                acc = std::abs(acc);
                if (acc > peak) {
                    peak = acc;
                }
            }
            ++buf_i;
            if (buf_i == buffer.size()) {
                buf_i = 0;
            }
        }
        chan_data_[chan].index = buf_i;
        chan_data_[chan].peak = peak;
    }
}  // namespace loudness
#endif  // LIBLOUDNESS_INTERPOLATOR_HPP
