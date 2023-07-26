/* See LICENSE file for copyright and license details. */

#ifndef LOUDNESS_TEST_UTILITIES
#define LOUDNESS_TEST_UTILITIES

#include <algorithm>
#include <cassert>
#include <catch2/matchers/catch_matchers_templated.hpp>
#include <cmath>
#include <vector>

#include "loudness/utils.hpp"

constexpr unsigned long prime_samplerate = 69313;
using DataTypes = std::tuple<float, double, std::int16_t, std::int32_t>;

inline double dbFSToLinear(double dbFS) { return std::pow(10, dbFS / 20.0); }

template <typename T>
auto sineWaveTP(double frequency, double samplerate, unsigned int channels, double phase, double scale)
    -> std::vector<T>
{
    scale *= loudness::getScalingFactor<T>();
    const long num_samples = std::lround(2 * samplerate);
    const double factor = 2 * std::numbers::pi * frequency / samplerate;
    const double offset = phase * std::numbers::pi / 180.0;
    std::vector<T> output;
    output.reserve(num_samples * channels);
    for (long i = 0; i < num_samples; ++i) {
        for (unsigned int c = 0; c < channels; ++c) {
            output.push_back(scale * std::sin(i * factor + offset));
        }
    }
    const long samples_10_ms = std::lround(samplerate / 100);
    for (long i = 0; i < samples_10_ms; ++i) {
        auto k = static_cast<double>(i) / static_cast<double>(samples_10_ms);
        for (unsigned int c = 0; c < channels; ++c) {
            output[i * channels + c] *= k;
            output[output.size() - 1 - i * channels - c] *= k;
        }
    }
    return output;
}

template <typename T>
auto sineWaveChannels(double frequency, double samplerate, long num_samples, unsigned int channels,
                      const std::vector<double>& dbFS) -> std::vector<std::vector<T>>
{
    assert(dbFS.size() == channels);
    std::vector<double> scale(channels);
    std::transform(dbFS.begin(), dbFS.end(), scale.begin(), [](auto val) {
        return dbFSToLinear(val) * loudness::getScalingFactor<T>();
        ;
    });
    const double factor = 2 * std::numbers::pi * frequency / samplerate;
    std::vector<std::vector<T>> output(channels, std::vector<T>(num_samples));
    for (long i = 0; i < num_samples; ++i) {
        auto val = std::sin(i * factor);
        for (unsigned int c = 0; c < channels; ++c) {
            output[c][i] = scale[c] * val;
        }
    }
    return output;
}

template <typename T>
auto sineWaveChannels(double frequency, double samplerate, long num_samples, unsigned int channels, double dbFS)
    -> std::vector<std::vector<T>>
{
    std::vector dbFSs(channels, dbFS);
    return sineWaveChannels<T>(frequency, samplerate, num_samples, channels, dbFSs);
}

template <typename T>
auto sineWave(double frequency, double samplerate, long num_samples, unsigned int channels,
              const std::vector<double>& dbFS) -> std::vector<T>
{
    assert(dbFS.size() == channels);
    std::vector<double> scale(channels);
    std::transform(dbFS.begin(), dbFS.end(), scale.begin(),
                   [](auto val) { return dbFSToLinear(val) * loudness::getScalingFactor<T>(); });
    const double factor = 2 * std::numbers::pi * frequency / samplerate;
    std::vector<T> output;
    output.reserve(num_samples * channels);
    for (long i = 0; i < num_samples; ++i) {
        for (unsigned int c = 0; c < channels; ++c) {
            output.push_back(scale[c] * std::sin(i * factor));
        }
    }
    return output;
}

template <typename T>
auto sineWave(double frequency, double samplerate, long num_samples, unsigned int channels, double dbFS)
    -> std::vector<T>
{
    std::vector dbFSs(channels, dbFS);
    return sineWave<T>(frequency, samplerate, num_samples, channels, dbFSs);
}

template <typename T>
auto deinterleave(const std::vector<T> data, int num_channels) -> std::vector<std::vector<T>>
{
    const long num_samples = data.size() / num_channels;
    std::vector<std::vector<T>> channels(num_channels, std::vector<T>(num_samples));
    for (long i = 0; i < num_samples; ++i) {
        for (int c = 0; c < num_channels; ++c) {
            channels[c][i] = data[i * num_channels + c];
        }
    }
    return channels;
}

class AsymetricMarginMatcher final : public Catch::Matchers::MatcherGenericBase {
public:
    AsymetricMarginMatcher(double target, double lower_margin, double upper_margin)
        : target_(target), lower_margin_(lower_margin), upper_margin_(upper_margin)
    {
    }
    bool match(double matchee) const
    {
        return (matchee + lower_margin_ >= target_) && (target_ + upper_margin_ >= matchee);
    }
    std::string describe() const override
    {
        return std::string("Check value is within ") + std::to_string(target_) + " +" + std::to_string(upper_margin_) +
               " -" + std::to_string(lower_margin_);
    }

private:
    double target_;
    double lower_margin_;
    double upper_margin_;
};
#endif  // LOUDNESS_TEST_UTILITIES
