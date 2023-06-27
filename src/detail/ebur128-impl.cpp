/* See COPYING file for copyright and license details. */

#include "detail/ebur128-impl.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath> /* You may have to define _USE_MATH_DEFINES if you use MSVC */
#include <cstdio>
#include <deque>
#include <numeric>
#include <type_traits>
#include <vector>

#include <gcem.hpp>

#include "bs1770-calculator.hpp"
#include "k-filter.hpp"
#include "true-peak-calculator.hpp"
#include "utils.hpp"
#include "constants.hpp"

namespace detail {
    struct Ebur128Impl {
        Ebur128Impl(unsigned int channels, unsigned long samplerate, unsigned int mode);

        void initChannelMap(size_t num_channels);
        void initFilter(double samplerate, unsigned int channels);
        double energyShortterm();
        double calcGatingBlock(size_t frames_per_block, bool add_block);
        double energyInInterval(size_t interval_frames);
        /** Filtered audio data (used as ring buffer). */
        std::vector<std::vector<double>> audio_data;
        /** Size of audio_data array. */
        size_t audio_data_frames;
        /** Current index for audio_data. */
        size_t audio_data_index = 0;

        size_t calculated_audio_index = 0;
        std::vector<std::deque<double>> calculated_subblocks;
        /** The channel map. Has as many elements as there are channels. */
        std::vector<Channel> channel_map;
        /** How many samples fit in 100ms (rounded). */
        unsigned long samples_in_100ms;
        /** How many frames are needed for a gating block. Will correspond to 400ms
         *  of audio at initialization, and 100ms after the first block (75% overlap
         *  as specified in the 2011 revision of BS1770). */
        unsigned long needed_frames;
        KFilter filter;

        /** Keeps track of when a new short term block is needed. */
        size_t short_term_frame_counter = 0;
        /** Maximum sample peak, one per channel */
        std::vector<double> sample_peak;
        std::vector<double> prev_sample_peak;
        /** Maximum true peak, one per channel */
        std::vector<double> true_peak;
        /** The maximum window duration in ms. */
        unsigned long window;

        TruePeakCalculator resampler;
        std::unique_ptr<BS1770Calculator> bs1770_calculator;
    };

    void Ebur128Impl::initChannelMap(size_t num_channels)
    {
        channel_map.resize(num_channels);
        if (num_channels == 4) {
            channel_map[0] = Channel::LEFT;
            channel_map[1] = Channel::RIGHT;
            channel_map[2] = Channel::LEFT_SURROUND;
            channel_map[3] = Channel::RIGHT_SURROUND;
        }
        else if (num_channels == 5) {
            channel_map[0] = Channel::LEFT;
            channel_map[1] = Channel::RIGHT;
            channel_map[2] = Channel::CENTER;
            channel_map[3] = Channel::LEFT_SURROUND;
            channel_map[4] = Channel::RIGHT_SURROUND;
        }
        else {
            for (size_t i = 0; i < num_channels; ++i) {
                switch (i) {
                case 0:
                    channel_map[i] = Channel::LEFT;
                    break;
                case 1:
                    channel_map[i] = Channel::RIGHT;
                    break;
                case 2:
                    channel_map[i] = Channel::CENTER;
                    break;
                case 3:
                    channel_map[i] = Channel::UNUSED;
                    break;
                case 4:
                    channel_map[i] = Channel::LEFT_SURROUND;
                    break;
                case 5:
                    channel_map[i] = Channel::RIGHT_SURROUND;
                    break;
                default:
                    channel_map[i] = Channel::UNUSED;
                    break;
                }
            }
        }
    }

    Ebur128::Ebur128(unsigned int channels, unsigned long samplerate, unsigned int mode)
        : mode(mode), channels(channels), samplerate(samplerate)
    {
        assert(channels > 0 && channels <= MAX_CHANNELS);
        assert(samplerate >= MIN_SAMPLERATE && samplerate <= MAX_SAMPLERATE);

        pimpl = std::make_unique<Ebur128Impl>(channels, samplerate, mode);
    }

    Ebur128::~Ebur128() = default;

    Ebur128Impl::Ebur128Impl(unsigned int channels, unsigned long samplerate, unsigned int mode)
        : calculated_subblocks(channels, std::deque<double>()),
          samples_in_100ms((samplerate + 5) / 10),
          /* the first block needs 400ms of audio data */
          needed_frames(samples_in_100ms * 4),
          filter(static_cast<double>(samplerate), channels),
          sample_peak(channels),
          prev_sample_peak(channels),
          true_peak(channels),
          resampler(samplerate, channels)
    {
        initChannelMap(channels);

        if ((mode & EBUR128_MODE_S) == EBUR128_MODE_S) {
            window = 3000;
        }
        else if ((mode & EBUR128_MODE_M) == EBUR128_MODE_M) {
            window = 400;
        }
        else {
            // No valid mode?
            return;
        }
        audio_data_frames = samplerate * window / 1000;
        if (audio_data_frames % samples_in_100ms) {
            /* round up to multiple of samples_in_100ms */
            audio_data_frames = (audio_data_frames + samples_in_100ms) - (audio_data_frames % samples_in_100ms);
        }
        audio_data.assign(channels, std::vector<double>(audio_data_frames));

        if (mode & EBUR128_MODE_HISTOGRAM) {
            bs1770_calculator = std::make_unique<HistogramCalculator>();
        }
        else {
            bs1770_calculator = std::make_unique<BlockListCalculator>();
        }
    }

    template <typename T>
    void Ebur128::filter(const T* src, size_t frames)
    {
        static constexpr double scaling_factor = getScalingFactor<T>();

        ScopedFTZ guard;

        // Find new sample peak
        if ((mode & EBUR128_MODE_SAMPLE_PEAK) == EBUR128_MODE_SAMPLE_PEAK) {
            for (size_t c = 0; c < channels; ++c) {
                double max = 0.0;
                for (size_t i = 0; i < frames; ++i) {
                    const double cur = std::abs(static_cast<double>(src[i * channels + c]));
                    if (cur > max) {
                        max = cur;
                    }
                }
                max /= scaling_factor;
                if (max > pimpl->prev_sample_peak[c]) {
                    pimpl->prev_sample_peak[c] = max;
                }
            }
        }

        // Find new true peak
        if ((mode & EBUR128_MODE_TRUE_PEAK) == EBUR128_MODE_TRUE_PEAK) {
            pimpl->resampler.process(src, frames);
        }

        // Apply filter
        for (size_t c = 0; c < channels; ++c) {
            if (pimpl->channel_map[c] == Channel::UNUSED) {
                continue;
            }
            double* audio_data = pimpl->audio_data[c].data() + pimpl->audio_data_index;
            for (size_t i = 0; i < frames; ++i) {
                audio_data[i] = pimpl->filter.apply(static_cast<double>(src[i * channels + c]) / scaling_factor, c);
            }
            #ifdef MANUALLY_FTZ
            pimpl->filter.manuallyFTZ(c);
            #endif
        }
        pimpl->audio_data_index += frames;
    }

    double Ebur128Impl::calcGatingBlock(size_t frames_per_block, bool add_block)
    {
        double sum = 0.0;
        const size_t channels = channel_map.size();
        for (size_t c = 0; c < channels; ++c) {
            if (channel_map[c] == Channel::UNUSED) {
                continue;
            }
            double channel_sum = 0.0;
            if (audio_data_index < frames_per_block) {
                // Read in the two parts of the circular buffer
                channel_sum += std::transform_reduce(audio_data[c].cbegin(), audio_data[c].cbegin() + audio_data_index, 0.0,
                                                     std::plus<>(), [](auto val){return val*val;});
                channel_sum += std::transform_reduce(audio_data[c].crbegin(), audio_data[c].crbegin() + frames_per_block - audio_data_index, 0.0,
                                                     std::plus<>(), [](auto val){return val*val;});
            }
            else {
                channel_sum += std::transform_reduce(audio_data[c].cbegin() + audio_data_index - frames_per_block, audio_data[c].cbegin() + audio_data_index, 0.0,
                                                     std::plus<>(), [](auto val){return val*val;});
            }
            switch (channel_map[c]) {
                case Channel::Mp110: case Channel::Mm110: case Channel::Mp060: case Channel::Mm060: case Channel::Mp090: case  Channel::Mm090:
                channel_sum *= 1.41;
                break;
            case Channel::DUAL_MONO:
                channel_sum *= 2.0;
                break;
            default:
                break;
            }

            sum += channel_sum;
        }

        sum /= static_cast<double>(frames_per_block);

        if (add_block && sum >= absolute_gate) {
            bs1770_calculator->addBlock(sum);
        }

        return sum;
    }

    void Ebur128::setChannel(unsigned int channel_number, Channel value)
    {
        if (channel_number > MAX_CHANNELS) {
            throw std::invalid_argument("Requested channels larger than maximum");
        }
        if (value == Channel::DUAL_MONO && (channels != 1 || channel_number != 0)) {
            throw std::invalid_argument("Channel::DUAL_MONO only works with mono files!");
        }
        pimpl->channel_map[channel_number] = value;
    }

    bool Ebur128::changeParameters(unsigned int channels, unsigned long samplerate)
    {
        if (channels == 0 || channels > MAX_CHANNELS) {
            throw std::invalid_argument("Requested channels larger than maximum");
        }

        if (samplerate < MIN_SAMPLERATE || samplerate > MAX_SAMPLERATE) {
            throw std::invalid_argument("Requested samplerate larger whan maximum");
        }

        if (this->channels == channels && this->samplerate == samplerate) {
            return false;
        }

        if (channels != this->channels) {
            pimpl->calculated_subblocks.assign(channels, std::deque<double>());
            pimpl->sample_peak.assign(channels, 0.0);
            pimpl->prev_sample_peak.assign(channels, 0.0);
            pimpl->true_peak.assign(channels, 0.0);

            pimpl->initChannelMap(channels);
        }
        if (samplerate != this->samplerate) {
            this->samplerate = samplerate;
            pimpl->samples_in_100ms = (samplerate + 5) / 10;
        }

        /* If we're here, either samplerate or channels
         * have changed. Re-init filter. */
        pimpl->filter = KFilter(static_cast<double>(samplerate), channels);

        pimpl->audio_data_frames = samplerate * pimpl->window / 1000;
        if (pimpl->audio_data_frames % pimpl->samples_in_100ms) {
            /* round up to multiple of samples_in_100ms */
            pimpl->audio_data_frames =
                (pimpl->audio_data_frames + pimpl->samples_in_100ms) - (pimpl->audio_data_frames % pimpl->samples_in_100ms);
        }
        pimpl->audio_data.assign(channels, std::vector<double>(pimpl->audio_data_frames));

        pimpl->resampler.configure(samplerate, channels);
        /* the first block needs 400ms of audio data */
        pimpl->needed_frames = pimpl->samples_in_100ms * 4;
        /* start at the beginning of the buffer */
        pimpl->audio_data_index = 0;
        /* reset short term frame counter */
        pimpl->short_term_frame_counter = 0;

        return true;
    }

    bool Ebur128::setMaxWindow(unsigned long window)
    {
        if ((mode & EBUR128_MODE_S) == EBUR128_MODE_S && window < 3000) {
            window = 3000;
        }
        else if ((mode & EBUR128_MODE_M) == EBUR128_MODE_M && window < 400) {
            window = 400;
        }

        if (window == pimpl->window) {
            return false;
        }

        size_t new_audio_data_frames;
        if (not safe_size_mul(samplerate, window, &new_audio_data_frames) ||
            new_audio_data_frames > std::numeric_limits<size_t>::max() - pimpl->samples_in_100ms) {
            throw std::invalid_argument("Requested window too large");
        }
        if (new_audio_data_frames % pimpl->samples_in_100ms) {
            /* round up to multiple of samples_in_100ms */
            new_audio_data_frames =
                (new_audio_data_frames + pimpl->samples_in_100ms) - (new_audio_data_frames % pimpl->samples_in_100ms);
        }

        size_t new_audio_data_size;
        if (not safe_size_mul(new_audio_data_frames, channels * sizeof(double), &new_audio_data_size)) {
            throw std::invalid_argument("Requested window too large");
        }

        pimpl->window = window;
        pimpl->audio_data.assign(channels, std::vector<double>(pimpl->audio_data_frames));

        /* the first block needs 400ms of audio data */
        pimpl->needed_frames = pimpl->samples_in_100ms * 4;
        /* start at the beginning of the buffer */
        pimpl->audio_data_index = 0;
        /* reset short term frame counter */
        pimpl->short_term_frame_counter = 0;

        return true;
    }

    bool Ebur128::setMaxHistory(unsigned long history)
    {
        if ((mode & EBUR128_MODE_LRA) == EBUR128_MODE_LRA && history < 3000) {
            history = 3000;
        }
        else if ((mode & EBUR128_MODE_M) == EBUR128_MODE_M && history < 400) {
            history = 400;
        }

        return pimpl->bs1770_calculator->setMaxHistory(history);
    }

    template <typename T>
    void Ebur128::addFrames(const T* src, size_t frames)
    {
        size_t src_index = 0;
        pimpl->prev_sample_peak.assign(channels, 0.0);
        pimpl->resampler.clearTruePeaks();
        while (frames > 0) {
            if (frames >= pimpl->needed_frames) {
                filter(src + src_index, pimpl->needed_frames);
                src_index += pimpl->needed_frames * channels;
                frames -= pimpl->needed_frames;
                /* calculate the new gating block */
                if ((mode & EBUR128_MODE_I) == EBUR128_MODE_I) {
                    pimpl->calcGatingBlock(pimpl->samples_in_100ms * 4, true);
                }
                if ((mode & EBUR128_MODE_LRA) == EBUR128_MODE_LRA) {
                    pimpl->short_term_frame_counter += pimpl->needed_frames;
                    if (pimpl->short_term_frame_counter == pimpl->samples_in_100ms * 30) {
                        const double st_energy = pimpl->energyShortterm();
                        if (st_energy >= absolute_gate) {
                            pimpl->bs1770_calculator->addShortTermBlock(st_energy);
                        }
                        pimpl->short_term_frame_counter = pimpl->samples_in_100ms * 20;
                    }
                }
                /* 100ms are needed for all blocks besides the first one */
                pimpl->needed_frames = pimpl->samples_in_100ms;
                /* reset audio_data_index when buffer full */
                if (pimpl->audio_data_index == pimpl->audio_data_frames) {
                    pimpl->audio_data_index = 0;
                }
            }
            else {
                filter(src + src_index, frames);
                if ((mode & EBUR128_MODE_LRA) == EBUR128_MODE_LRA) {
                    pimpl->short_term_frame_counter += frames;
                }
                pimpl->needed_frames -= static_cast<unsigned long>(frames);
                frames = 0;
            }
        }
        for (unsigned int c = 0; c < channels; c++) {
            if (pimpl->prev_sample_peak[c] > pimpl->sample_peak[c]) {
                pimpl->sample_peak[c] = pimpl->prev_sample_peak[c];
            }
            if (pimpl->resampler.prevTruePeak(c) > pimpl->true_peak[c]) {
                pimpl->true_peak[c] = pimpl->resampler.prevTruePeak(c);
            }
        }
    }

    template void Ebur128::addFrames<short>(const short* src, size_t frames);
    template void Ebur128::addFrames<int>(const int* src, size_t frames);
    template void Ebur128::addFrames<float>(const float* src, size_t frames);
    template void Ebur128::addFrames<double>(const double* src, size_t frames);

    double Ebur128::relativeThreshold()
    {
        auto [above_thresh_counter, relative_threshold] = pimpl->bs1770_calculator->relativeThreshold();

        if (above_thresh_counter == 0) {
            return -70.0; // absolute threshold
        }

        relative_threshold /= static_cast<double>(above_thresh_counter);
        relative_threshold *= relative_gate_factor;

        return energyToLoudness(relative_threshold);
    }

    double Ebur128::loudnessGlobal() {
        auto [above_thresh_counter, relative_threshold] = pimpl->bs1770_calculator->relativeThreshold();
        if (above_thresh_counter == 0) {
            return -HUGE_VAL;
        }

        relative_threshold /= static_cast<double>(above_thresh_counter);
        relative_threshold *= relative_gate_factor;

        auto [above_rel_counter, gated_loudness] = pimpl->bs1770_calculator->gatedLoudness(relative_threshold);

        return above_rel_counter == 0 ? -HUGE_VAL : energyToLoudness(gated_loudness / static_cast<double>(above_rel_counter));
    }

    double Ebur128Impl::energyInInterval(size_t interval_frames)
    {
        return calcGatingBlock(interval_frames, false);
    }

    double Ebur128Impl::energyShortterm()
    {
        return energyInInterval(samples_in_100ms * 30);
    }

    double Ebur128::loudnessMomentary()
    {
        const double energy = pimpl->energyInInterval(pimpl->samples_in_100ms * 4);
        return energy <= 0.0 ? -HUGE_VAL : energyToLoudness(energy);
    }

    double Ebur128::loudnessShortterm()
    {
        const double energy = pimpl->energyShortterm();
        return energy <= 0.0 ? -HUGE_VAL :  energyToLoudness(energy);
    }

    double Ebur128::loudnessWindow(unsigned long window)
    {
        const size_t interval_frames = samplerate * window / 1000;;

        if (window > pimpl->window || interval_frames > pimpl->audio_data_frames) [[unlikely]]{
            throw std::invalid_argument("Given window too large for current mode");
        }

        const double energy = pimpl->calcGatingBlock(interval_frames, false);

        return energy <= 0.0 ? -HUGE_VAL : energyToLoudness(energy);
    }

    double Ebur128::loudnessRange() {
        // TODO: Find better solution here, avoiding dynamic cast
        if ((mode & EBUR128_MODE_HISTOGRAM) == EBUR128_MODE_HISTOGRAM) {
            return HistogramCalculator::loudnessRangeMultiple({dynamic_cast<HistogramCalculator*>(pimpl->bs1770_calculator.get())});
        } else {
            return  BlockListCalculator::loudnessRangeMultiple({dynamic_cast<BlockListCalculator*>(pimpl->bs1770_calculator.get())});
        }
    }

    double Ebur128::loudnessGlobalMedian()
    {
        auto [above_thresh_counter, relative_threshold] = pimpl->bs1770_calculator->relativeThreshold();
        if (above_thresh_counter == 0) {
            return -HUGE_VAL;
        }

        relative_threshold /= static_cast<double>(above_thresh_counter);
        relative_threshold *= relative_gate_factor;

        return energyToLoudness(pimpl->bs1770_calculator->gatedMedianLoudness(relative_threshold));
    }

    double Ebur128::samplePeak(unsigned int channel_index) const
    {
        if (channel_index >= channels) {
            throw std::invalid_argument("Invalid channel index");
        }

        return pimpl->sample_peak[channel_index];
    }

    double Ebur128::prevSamplePeak(unsigned int channel_number) const
    {
        if (channel_number >= channels) {
            throw std::invalid_argument("Invalid channel index");
        }

        return pimpl->prev_sample_peak[channel_number];
    }

    double Ebur128::truePeak(unsigned int channel_number) const
    {
        if (channel_number >= channels) {
            throw std::invalid_argument("Invalid channel index");
        }

        return std::max(pimpl->true_peak[channel_number], pimpl->sample_peak[channel_number]);
    }

    double Ebur128::prevTruePeak(unsigned int channel_number) const
    {
        if (channel_number >= channels) {
            throw std::invalid_argument("Invalid channel index");
        }

        return std::max(static_cast<double>(pimpl->resampler.prevTruePeak(channel_number)),
                        pimpl->prev_sample_peak[channel_number]);
    }

    double loudnessRangeMultipleHist(const std::vector<const Ebur128Impl*>& meters)
    {
        std::vector<const HistogramCalculator*> hists;
        for (const auto& meter : meters){
            hists.push_back(dynamic_cast<const HistogramCalculator*>(meter->bs1770_calculator.get()));
        }
        return HistogramCalculator::loudnessRangeMultiple(hists);
    }

    double loudnessRangeMultipleBlocks(const std::vector<const Ebur128Impl*>& meters)
    {
        std::vector<const BlockListCalculator*> lists;
        for (const auto& meter : meters){
            lists.push_back(dynamic_cast<BlockListCalculator*>(meter->bs1770_calculator.get()));
        }
        return BlockListCalculator::loudnessRangeMultiple(lists);
    }

    double loudnessGlobalMultiple(const std::vector<const Ebur128Impl*>& meters)
    {
        double gated_loudness = 0.0;
        double relative_threshold = 0.0;
        size_t above_thresh_counter = 0;

        for (const auto& meter : meters) {
            auto value = meter->bs1770_calculator->relativeThreshold();
            relative_threshold += value.sum;
            above_thresh_counter += value.counter;
        }
        if (above_thresh_counter == 0) {
            return -HUGE_VAL;
        }

        relative_threshold /= static_cast<double>(above_thresh_counter);
        relative_threshold *= relative_gate_factor;

        above_thresh_counter = 0;
        for (const auto& meter : meters) {
            auto value = meter->bs1770_calculator->gatedLoudness(relative_threshold);
            above_thresh_counter += value.counter;
            gated_loudness += value.sum;
        }
        if (above_thresh_counter == 0) {
            return -HUGE_VAL;
        }
        gated_loudness /= static_cast<double>(above_thresh_counter);
        return energyToLoudness(gated_loudness);
    }

} // namespace detail
