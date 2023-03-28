#ifndef TRUE_PEAK_CALCULATOR_HPP
#define TRUE_PEAK_CALCULATOR_HPP

#include <memory>
#include <span>
#include <vector>

#include "interpolator.hpp"
#include "utils.hpp"

class TruePeakCalculator {
public:
    TruePeakCalculator(unsigned long samplerate, unsigned int channels);

    void configure(unsigned long samplerate, unsigned int channels);

    // Note, other types than float are less efficient, especially non-floating point
    template <typename T>
    std::size_t process(const T* src, std::size_t frames);

    float prevTruePeak(unsigned int channel) const
    {
        return channel < prev_true_peak_.size() - 1 ? prev_true_peak_[channel] : 0.0f;
    }

    void clearTruePeaks() { prev_true_peak_.assign(channels_, 0.0); }

private:
    std::vector<float> resampler_buffer_input_;
    std::vector<float> resampler_buffer_output_;
    std::unique_ptr<Interpolator> interp_;
    std::vector<float> prev_true_peak_;
    unsigned int channels_;
};

template <typename T>
std::size_t TruePeakCalculator::process(const T* src, std::size_t frames)
{
    static constexpr double scaling_factor = getScalingFactor<T>();

    for (size_t i = 0; i < frames; ++i) {
        for (unsigned int c = 0; c < channels_; ++c) {
            resampler_buffer_input_[i * channels_ + c] =
                static_cast<float>(static_cast<double>(src[i * channels_ + c]) / scaling_factor);
        }
    }

    size_t frames_out;
    std::span<float> resampled_data;
    if (interp_) {
        frames_out = interp_->process(frames, resampler_buffer_input_.data(), resampler_buffer_output_.data());
        ;
        resampled_data = std::span(resampler_buffer_output_.data(), frames_out * channels_);
    }
    else {
        frames_out = frames;
        resampled_data = std::span(resampler_buffer_input_.data(), frames_out * channels_);
    }

    for (size_t i = 0; i < frames_out; ++i) {
        for (size_t c = 0; c < channels_; ++c) {
            auto val = std::abs(resampled_data[i * channels_ + c]);
            if (val > prev_true_peak_[c]) {
                prev_true_peak_[c] = val;
            }
        }
    }
    // TODO: Breaks with samplerate >= 96000
    return frames_out;
}

#endif  // TRUE_PEAK_CALCULATOR_HPP
