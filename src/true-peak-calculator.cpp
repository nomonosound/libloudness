#include "true-peak-calculator.hpp"

#include <cmath>

TruePeakCalculator::TruePeakCalculator(unsigned long samplerate, unsigned int channels)
{
    configure(samplerate, channels);
}

void TruePeakCalculator::configure(unsigned long samplerate, unsigned int channels)
{
    channels_ = channels;
    if (samplerate < 96000) {
        interp_ = std::make_unique<Interpolator>(49, 4, channels);
    }
    else if (samplerate < 192000) {
        interp_ = std::make_unique<Interpolator>(49, 2, channels);
    }
    else {
        interp_ = nullptr;
        return;
    }

    // input frames over 400ms
    const size_t resampler_buffer_input_frames = std::lround(static_cast<double>(samplerate) * 0.4);
    resampler_buffer_input_.resize(resampler_buffer_input_frames * channels);

    const size_t resampler_buffer_output_frames = resampler_buffer_input_frames * interp_->factor();
    resampler_buffer_output_.resize(resampler_buffer_output_frames * channels);

    prev_true_peak_.assign(channels, 0.0);
}

template <>
std::size_t TruePeakCalculator::process(const float* src, std::size_t frames)
{
    size_t frames_out;
    std::span<const float> resampled_data;
    if (interp_) {
        frames_out = interp_->process(frames, src, resampler_buffer_output_.data());
        ;
        resampled_data = std::span(resampler_buffer_output_.data(), frames_out * channels_);
    }
    else {
        frames_out = frames;
        resampled_data = std::span(src, frames_out * channels_);
    }

    for (size_t i = 0; i < frames_out; ++i) {
        for (size_t c = 0; c < channels_; ++c) {
            auto val = std::abs(resampled_data[i * channels_ + c]);
            if (val > prev_true_peak_[c]) {
                prev_true_peak_[c] = val;
            }
        }
    }
    return frames_out;
}
