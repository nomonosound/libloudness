/* See LICENSE file for copyright and license details. */

#include "meter-impl.hpp"

#include <BS_thread_pool_light.hpp>
#include <algorithm>
#include <array>
#include <cassert>
#include <deque>
#include <gcem.hpp>
#include <numeric>
#include <optional>
#include <type_traits>
#include <vector>

#include "loudness/bs1770-calculator.hpp"
#include "loudness/constants.hpp"
#include "loudness/interpolator.hpp"
#include "loudness/k-filter.hpp"
#include "loudness/utils.hpp"

namespace loudness::detail {
    struct Impl {
        Impl(unsigned int channels, unsigned long samplerate, Mode mode);

        /** Number of subblocks to store. Each subblock is 100ms*/
        static constexpr int num_subblocks = st_subblocks;

        void initChannelMap(std::size_t num_channels);
        void initAudioBuffers(unsigned num_channels, unsigned long samplerate);
        void initInterpolator(unsigned num_channels, unsigned long samplerate);
        void recalcChannelWeights();
        void calcSubBlocks();
        void addIntegrationBlock();
        void addShorttermBlock();
        [[nodiscard]] double energyInInterval(std::size_t interval_frames) const;

        void addFramesLoudness(DataType src, std::size_t frames);

        template <ConstData T>
        void filter(T* src, std::size_t offset, std::size_t frames);

        template <ConstData T>
        void findSamplePeaks(T* src, std::size_t frames);

        /** Filtered audio data (used as ring buffer). */
        std::vector<std::vector<double>> audio_data_;
        /** Size of audio_data array. */
        std::size_t audio_data_frames_;
        /** Current index for audio_data. */
        std::size_t audio_data_index_ = 0;

        /** Data for subblocks for gating blocks */
        std::size_t calc_audio_index_ = 0;
        std::deque<double> calculated_subblocks_;

        /** The channel map. Has as many elements as there are channels. */
        std::vector<Channel> channel_map_;
        std::vector<double> channel_weight_;

        /** How many samples fit in 100ms (rounded). */
        unsigned long samples_in_100ms_;
        /** How many frames are needed for a gating block. Will correspond to 400ms
         *  of audio at initialization, and 100ms after the first block (75% overlap
         *  as specified in ITU-BS.1770). */
        unsigned long needed_frames_;
        KFilter k_filter_;

        /** Maximum sample peak, one per channel */
        std::vector<double> sample_peak_;
        std::vector<double> last_sample_peak_;
        /** Maximum true peak, one per channel */
        std::vector<double> true_peak_;
        /** The maximum window duration in ms. */
        unsigned long window_ms_;

        std::optional<Interpolator> interpolator_;
        std::unique_ptr<BS1770Calculator> bs1770_calculator_;

        std::optional<BS::thread_pool_light> pool_;
        Mode mode_;
    };

    Impl::Impl(unsigned int channels, unsigned long samplerate, Mode mode)
        : samples_in_100ms_(roundedDivide<subblocks_in_s>(samplerate)),
          /* the first block needs 400ms of audio data */
          needed_frames_(samples_in_100ms_ * m_subblocks),
          k_filter_(static_cast<double>(samplerate), channels),
          sample_peak_(channels),
          last_sample_peak_(channels),
          true_peak_(channels),
          mode_(mode)
    {
        initChannelMap(channels);

        if ((mode & Mode::Histogram) == Mode::Histogram) {
            bs1770_calculator_ = std::make_unique<HistogramCalculator>();
        }
        else {
            bs1770_calculator_ = std::make_unique<BlockListCalculator>();
        }

        if ((mode & Mode::EBU_S) == Mode::EBU_S) {
            window_ms_ = shortterm_block_ms;
        }
        else if ((mode & Mode::EBU_M) == Mode::EBU_M) {
            window_ms_ = momentary_block_ms;
        }
        else {
            // Disable window
            window_ms_ = 0;
        }

        initAudioBuffers(channels, samplerate);

        if ((mode & Mode::TruePeak) == Mode::TruePeak) {
            initInterpolator(channels, samplerate);
        }
    }

    void Impl::initChannelMap(std::size_t num_channels)
    {
        channel_map_.resize(num_channels);
        if (num_channels == 4) {
            channel_map_[0] = Channel::Left;
            channel_map_[1] = Channel::Right;
            channel_map_[2] = Channel::LeftSurround;
            channel_map_[3] = Channel::RightSurround;
        }
        else if (num_channels == 5) {
            channel_map_[0] = Channel::Left;
            channel_map_[1] = Channel::Right;
            channel_map_[2] = Channel::Center;
            channel_map_[3] = Channel::LeftSurround;
            channel_map_[4] = Channel::RightSurround;
        }
        else {
            for (std::size_t i = 0; i < num_channels; ++i) {
                switch (i) {
                case 0:
                    channel_map_[i] = Channel::Left;
                    break;
                case 1:
                    channel_map_[i] = Channel::Right;
                    break;
                case 2:
                    channel_map_[i] = Channel::Center;
                    break;
                case 3:
                    channel_map_[i] = Channel::Unused;
                    break;
                case 4:
                    channel_map_[i] = Channel::LeftSurround;
                    break;
                case 5:
                    channel_map_[i] = Channel::RightSurround;
                    break;
                default:
                    channel_map_[i] = Channel::Unused;
                    break;
                }
            }
        }
        recalcChannelWeights();
    }

    void Impl::recalcChannelWeights()
    {
        channel_weight_.clear();
        for (const auto& channel : channel_map_) {
            switch (channel) {
            case Channel::Mp110: case Channel::Mm110: case Channel::Mp060: case Channel::Mm060: case Channel::Mp090: case Channel::Mm090:
                channel_weight_.push_back(1.41);
                break;
            case Channel::DualMono:
                channel_weight_.push_back(2.0);
                break;
            case Channel::Unused:
                channel_weight_.push_back(0.0);
                break;
            [[likely]] default:
                channel_weight_.push_back(1.0);
            }
        }
    }

    void Impl::initAudioBuffers(unsigned num_channels, unsigned long samplerate)
    {
        audio_data_frames_ = (samplerate * window_ms_) / milliseconds_in_second;
        if (audio_data_frames_ % samples_in_100ms_ != 0) {
            /* round up to multiple of samples_in_100ms */
            audio_data_frames_ = (audio_data_frames_ + samples_in_100ms_) - (audio_data_frames_ % samples_in_100ms_);
        }
        audio_data_.assign(num_channels, std::vector<double>(audio_data_frames_));

        /* the first block needs 400ms of audio data */
        needed_frames_ = samples_in_100ms_ * m_subblocks;
        /* start at the beginning of the buffer */
        audio_data_index_ = 0;
        calc_audio_index_ = 0;
    }

    void Impl::initInterpolator(unsigned int num_channels, unsigned long samplerate)
    {
        if (samplerate < 96000) {
            interpolator_.emplace(49, 4, num_channels);
        }
        else if (samplerate < 192000) {
            interpolator_.emplace(49, 2, num_channels);
        }
        else {
            interpolator_ = std::nullopt;
        }
    }

    template <ConstData T>
    void Impl::filter(T* src, std::size_t offset, std::size_t frames)
    {
        // Apply filter
        const std::size_t channels = audio_data_.size();
        for (std::size_t c = 0; c < channels; ++c) {
            if (channel_map_[c] == Channel::Unused) {
                continue;
            }
            double* audio = audio_data_[c].data() + audio_data_index_;
            if constexpr (std::is_pointer_v<T>) {
                std::transform(src[c] + offset, src[c] + offset + frames, audio, [this, c](auto val) {
                    double filtered =
                        k_filter_.apply(static_cast<double>(val) / getScalingFactor<std::remove_pointer_t<T>>(), c);
                    return filtered * filtered;
                });
            }
            else {
                for (std::size_t i = 0; i < frames; ++i) {
                    audio[i] = k_filter_.apply(
                        static_cast<double>(src[(i + offset) * channels + c]) / getScalingFactor<T>(), c);
                    audio[i] *= audio[i];
                }
            }
#ifdef MANUALLY_FTZ
            k_filter_.manuallyFTZ(c);
#endif
        }
        audio_data_index_ += frames;
    }

    template <ConstData T>
    void Impl::findSamplePeaks(T* src, std::size_t frames)
    {
        const std::size_t channels = sample_peak_.size();
        for (std::size_t chan = 0; chan < channels; ++chan) {
            if constexpr (std::is_pointer_v<T>) {
                using U = std::remove_pointer_t<T>;
                auto minmax = std::minmax_element(src[chan], src[chan] + frames);
                last_sample_peak_[chan] =
                    static_cast<double>(std::max<U>(-*minmax.first, *minmax.second)) / getScalingFactor<U>();
            }
            else {
                const std::size_t channels = sample_peak_.size();
                double max = 0.0;
                for (std::size_t i = 0; i < frames; ++i) {
                    const double cur = std::abs(static_cast<double>(src[i * channels + chan]));
                    if (cur > max) {
                        max = cur;
                    }
                }
                last_sample_peak_[chan] = max / getScalingFactor<T>();
            }
            if (last_sample_peak_[chan] > sample_peak_[chan]) {
                sample_peak_[chan] = last_sample_peak_[chan];
            }
        }
    }

    double Impl::energyInInterval(std::size_t interval_frames) const
    {
        double sum = 0.0;
        for (std::size_t c = 0; c < channel_map_.size(); ++c) {
            if (channel_map_[c] == Channel::Unused) {
                continue;
            }
            double channel_sum = 0.0;
            if (audio_data_index_ < interval_frames) {
                // Read over the seam in the circular buffer
                channel_sum += std::reduce(audio_data_[c].cbegin(), audio_data_[c].cbegin() + audio_data_index_, 0.0) +
                               std::reduce(audio_data_[c].crbegin(),
                                           audio_data_[c].crbegin() + interval_frames - audio_data_index_, 0.0);
            }
            else {
                channel_sum += std::reduce(audio_data_[c].cbegin() + audio_data_index_ - interval_frames,
                                           audio_data_[c].cbegin() + audio_data_index_, 0.0);
            }
            channel_sum *= channel_weight_[c];

            sum += channel_sum;
        }

        return sum / static_cast<double>(interval_frames);
    }

    void Impl::addFramesLoudness(DataType src, std::size_t frames)
    {
        const ScopedFTZ guard;
        std::size_t src_index = 0;
        while (frames > 0) {
            if (frames >= needed_frames_) {
                std::visit([this, src_index](auto&& src) { filter(src, src_index, needed_frames_); }, src);
                src_index += needed_frames_;
                frames -= needed_frames_;
                /* calculate the new gating block */
                if ((mode_ & Mode::EBU_I) == Mode::EBU_I) {
                    calcSubBlocks();
                    addIntegrationBlock();
                }
                if ((mode_ & Mode::EBU_LRA) == Mode::EBU_LRA) {
                    calcSubBlocks();
                    if (calculated_subblocks_.size() >= st_subblocks) {
                        addShorttermBlock();
                    }
                }
                /* 100ms are needed for all blocks besides the first one */
                needed_frames_ = samples_in_100ms_;
                /* reset audio_data_index when buffer full */
                if (audio_data_index_ == audio_data_frames_) {
                    audio_data_index_ = 0;
                }
                if (calc_audio_index_ == audio_data_frames_) {
                    calc_audio_index_ = 0;
                }
            }
            else {
                std::visit([this, src_index, frames](auto&& src) { filter(src, src_index, frames); }, src);
                needed_frames_ -= static_cast<unsigned long>(frames);
                frames = 0;
            }
        }
    }

    void Impl::calcSubBlocks()
    {
        for (; calc_audio_index_ + samples_in_100ms_ <= audio_data_index_; calc_audio_index_ += samples_in_100ms_) {
            double sum = 0.0;
            for (std::size_t c = 0; c < channel_map_.size(); ++c) {
                if (channel_map_[c] == Channel::Unused) {
                    continue;
                }
                sum += channel_weight_[c] * std::reduce(audio_data_[c].cbegin() + calc_audio_index_,
                                                        audio_data_[c].cbegin() + calc_audio_index_ + samples_in_100ms_,
                                                        0.0);
            }
            calculated_subblocks_.push_back(sum);
            if (calculated_subblocks_.size() > num_subblocks) [[likely]] {
                calculated_subblocks_.pop_front();
            }
        }
    }

    void Impl::addIntegrationBlock()
    {
        double sum = std::reduce(calculated_subblocks_.crbegin(), calculated_subblocks_.crbegin() + m_subblocks, 0.0);
        sum /= static_cast<double>(m_subblocks * samples_in_100ms_);
        if (sum >= absolute_gate) [[likely]] {
            bs1770_calculator_->addBlock(sum);
        }
    }

    void Impl::addShorttermBlock()
    {
        double sum = std::reduce(calculated_subblocks_.crbegin(), calculated_subblocks_.crbegin() + st_subblocks, 0.0);
        sum /= static_cast<double>(st_subblocks * samples_in_100ms_);
        if (sum >= absolute_gate) [[likely]] {
            bs1770_calculator_->addShortTermBlock(sum);
        }
    }

    /* Meter */

    Meter::Meter(NumChannels channels, Samplerate samplerate, Mode mode)
        : mode_(mode),
          channels_(channels.get()),
          samplerate_(samplerate.get()),
          pimpl_(std::make_unique<Impl>(channels_, samplerate_, mode))
    {
    }

    Meter::~Meter() noexcept = default;
    Meter::Meter(Meter&& other) noexcept = default;
    Meter& Meter::operator=(Meter&& other) noexcept = default;

    void Meter::reset() { pimpl_ = std::make_unique<Impl>(channels_, samplerate_, mode_); }

    void Meter::setChannel(unsigned int channel_index, Channel value)
    {
        if (channel_index >= channels_) {
            throw std::out_of_range("Invalid channel index");
        }
        if (pimpl_->channel_map_[channel_index] != value) {
            pimpl_->channel_map_[channel_index] = value;
            pimpl_->recalcChannelWeights();
        }
    }

    bool Meter::changeParameters(NumChannels channels, Samplerate samplerate)
    {
        if (channels_ == channels.get() && samplerate_ == samplerate.get()) {
            return false;
        }

        if (channels.get() != channels_) {
            channels_ = channels.get();
            pimpl_->sample_peak_.assign(channels_, 0.0);
            pimpl_->last_sample_peak_.assign(channels_, 0.0);
            pimpl_->true_peak_.assign(channels_, 0.0);

            pimpl_->initChannelMap(channels_);
        }
        if (samplerate.get() != samplerate_) {
            samplerate_ = samplerate.get();
            pimpl_->samples_in_100ms_ = roundedDivide<subblocks_in_s>(samplerate_);
        }

        /* If we're here, either samplerate or channels
         * have changed. Re-init filter. */
        pimpl_->k_filter_ = KFilter(static_cast<double>(samplerate_), channels_);

        pimpl_->initAudioBuffers(channels_, samplerate_);

        if ((mode_ & Mode::TruePeak) == Mode::TruePeak) {
            pimpl_->initInterpolator(channels_, samplerate_);
        }

        return true;
    }

    bool Meter::setMaxWindow(unsigned long window_ms)
    {
        if (window_ms > max_window_ms) {
            throw std::domain_error("Requested window too large");
        }
        if ((mode_ & Mode::EBU_S) == Mode::EBU_S && window_ms < shortterm_block_ms) {
            window_ms = shortterm_block_ms;
        }
        else if ((mode_ & Mode::EBU_M) == Mode::EBU_M && window_ms < momentary_block_ms) {
            window_ms = momentary_block_ms;
        }

        if (window_ms == pimpl_->window_ms_) {
            return false;
        }

        pimpl_->window_ms_ = window_ms;
        pimpl_->initAudioBuffers(channels_, samplerate_);
        return true;
    }

    bool Meter::setMaxHistory(unsigned long history_ms)
    {
        if ((mode_ & Mode::EBU_LRA) == Mode::EBU_LRA && history_ms < shortterm_block_ms) {
            history_ms = shortterm_block_ms;
        }
        else if ((mode_ & Mode::EBU_M) == Mode::EBU_M && history_ms < momentary_block_ms) {
            history_ms = momentary_block_ms;
        }

        return pimpl_->bs1770_calculator_->setMaxHistory(history_ms);
    }

    void Meter::addFramesMT(DataType src, std::size_t frames)
    {
        /* Lazely spin up threads when needed. */
        if (not pimpl_->pool_.has_value()) {
            pimpl_->pool_.emplace();
        }

        if (pimpl_->interpolator_.has_value()) {
            for (unsigned int c = 0; c < channels_; ++c) {
                pimpl_->pool_->push_task([this, c, src, frames] {
                    const ScopedFTZ guard;
                    std::visit([this, c, frames](auto&& src) { pimpl_->interpolator_->process(src, frames, c); }, src);
                    if (pimpl_->interpolator_->peak(c) > pimpl_->true_peak_[c]) {
                        pimpl_->true_peak_[c] = pimpl_->interpolator_->peak(c);
                    }
                });
            }
        }
        if ((mode_ & Mode::EBU_M) == Mode::EBU_M) {
            pimpl_->pool_->push_task(&Impl::addFramesLoudness, pimpl_.get(), src, frames);
        }
        // Sample peak is so lightweight it is never worth parallelizing channels
        if ((mode_ & Mode::SamplePeak) == Mode::SamplePeak) {
            pimpl_->pool_->push_task([this, src, frames] {
                std::visit([this, frames](auto&& src) { pimpl_->findSamplePeaks(src, frames); }, src);
            });
        }

        pimpl_->pool_->wait_for_tasks();
    }

    void Meter::addFrames(DataType src, std::size_t frames)
    {
        const ScopedFTZ guard;
        // Find new sample peak
        if ((mode_ & Mode::SamplePeak) == Mode::SamplePeak) {
            std::visit([this, frames](auto&& src) { pimpl_->findSamplePeaks(src, frames); }, src);
        }
        // Find new true peak
        if (pimpl_->interpolator_.has_value()) {
            for (unsigned int c = 0; c < channels_; ++c) {
                std::visit([this, frames, c](auto&& src) { pimpl_->interpolator_->process(src, frames, c); }, src);
                if (pimpl_->interpolator_->peak(c) > pimpl_->true_peak_[c]) {
                    pimpl_->true_peak_[c] = pimpl_->interpolator_->peak(c);
                }
            }
        }

        if ((mode_ & Mode::EBU_M) == Mode::EBU_M) {
            pimpl_->addFramesLoudness(src, frames);
        }
    }

    double Meter::relativeThreshold() const
    {
        auto [above_thresh_counter, relative_threshold] = pimpl_->bs1770_calculator_->blockCountAndSum();

        if (above_thresh_counter == 0) {
            return absolute_gate_LUFS;
        }

        relative_threshold /= static_cast<double>(above_thresh_counter);
        relative_threshold *= relative_gate_factor;

        return energyToLoudness(relative_threshold);
    }

    double Meter::loudnessGlobal() const
    {
        auto [above_thresh_count, block_sum] = pimpl_->bs1770_calculator_->blockCountAndSum();
        if (above_thresh_count == 0) {
            return -HUGE_VAL;
        }

        block_sum /= static_cast<double>(above_thresh_count);

        auto [above_rel_counter, gated_loudness] =
            pimpl_->bs1770_calculator_->loudness(block_sum * relative_gate_factor);

        return above_rel_counter == 0 ? -HUGE_VAL
                                      : energyToLoudness(gated_loudness / static_cast<double>(above_rel_counter));
    }

    double Meter::loudnessMomentary() const
    {
        const double energy = pimpl_->energyInInterval(pimpl_->samples_in_100ms_ * m_subblocks);
        return energy <= 0.0 ? -HUGE_VAL : energyToLoudness(energy);
    }

    double Meter::loudnessShortterm() const
    {
        const double energy = pimpl_->energyInInterval(pimpl_->samples_in_100ms_ * st_subblocks);
        return energy <= 0.0 ? -HUGE_VAL : energyToLoudness(energy);
    }

    double Meter::loudnessRange() const
    {
        // TODO: Find better solution here, avoiding dynamic cast
        if ((mode_ & Mode::Histogram) == Mode::Histogram) {
            return HistogramCalculator::loudnessRangeMultiple(
                {dynamic_cast<HistogramCalculator*>(pimpl_->bs1770_calculator_.get())});
        }
        else {
            return BlockListCalculator::loudnessRangeMultiple(
                {dynamic_cast<BlockListCalculator*>(pimpl_->bs1770_calculator_.get())});
        }
    }

    double Meter::loudnessWindow(unsigned long window_ms) const
    {
        const std::size_t interval_frames = (samplerate_ * window_ms) / milliseconds_in_second;

        if (interval_frames > pimpl_->audio_data_frames_) {
            throw std::domain_error("Requested window larger than currently configured max window");
        }

        const double energy = pimpl_->energyInInterval(interval_frames);
        return energy <= 0.0 ? -HUGE_VAL : energyToLoudness(energy);
    }

    double Meter::loudnessGlobalUngated() const
    {
        auto [above_thresh_count, block_sum] = pimpl_->bs1770_calculator_->blockCountAndSum();
        if (above_thresh_count == 0) {
            return -HUGE_VAL;
        }

        return energyToLoudness(block_sum / static_cast<double>(above_thresh_count));
    }

    double Meter::loudnessGlobalMedian() const
    {
        auto relative_threshold = pimpl_->bs1770_calculator_->ungatedMedianLoudness();
        if (relative_threshold == -HUGE_VAL) {
            return -HUGE_VAL;
        }
        relative_threshold *= relative_gate_factor;

        return energyToLoudness(pimpl_->bs1770_calculator_->medianLoudness(relative_threshold));
    }

    double Meter::loudnessGlobalMedianAfterGate() const
    {
        auto [above_thresh_count, relative_threshold] = pimpl_->bs1770_calculator_->blockCountAndSum();
        if (above_thresh_count == 0) {
            return -HUGE_VAL;
        }
        relative_threshold /= static_cast<double>(above_thresh_count);
        relative_threshold *= relative_gate_factor;

        return energyToLoudness(pimpl_->bs1770_calculator_->medianLoudness(relative_threshold));
    }

    double Meter::loudnessGlobalMedianUngated() const
    {
        return energyToLoudness(pimpl_->bs1770_calculator_->ungatedMedianLoudness());
    }

    double Meter::samplePeak(unsigned int channel_index) const
    {
        if (channel_index >= channels_) {
            throw std::out_of_range("Invalid channel index");
        }

        return pimpl_->sample_peak_[channel_index];
    }

    double Meter::lastSamplePeak(unsigned int channel_index) const
    {
        if (channel_index >= channels_) {
            throw std::out_of_range("Invalid channel index");
        }

        return pimpl_->last_sample_peak_[channel_index];
    }

    double Meter::truePeak(unsigned int channel_index) const
    {
        if (channel_index >= channels_) {
            throw std::out_of_range("Invalid channel index");
        }

        return std::max(pimpl_->true_peak_[channel_index], pimpl_->sample_peak_[channel_index]);
    }

    double Meter::lastTruePeak(unsigned int channel_index) const
    {
        if (channel_index >= channels_) {
            throw std::out_of_range("Invalid channel index");
        }
        if (pimpl_->interpolator_.has_value()) {
            return std::max(static_cast<double>(pimpl_->interpolator_->peak(channel_index)),
                            pimpl_->last_sample_peak_[channel_index]);
        }
        return pimpl_->last_sample_peak_[channel_index];
    }

    double Meter::samplePeak() const
    {
        return *std::max_element(pimpl_->sample_peak_.cbegin(), pimpl_->sample_peak_.cend());
    }

    double Meter::lastSamplePeak() const
    {
        return *std::max_element(pimpl_->last_sample_peak_.cbegin(), pimpl_->last_sample_peak_.cend());
    }

    double Meter::truePeak() const
    {
        return std::max(*std::max_element(pimpl_->true_peak_.cbegin(), pimpl_->true_peak_.cend()), samplePeak());
    }

    double Meter::lastTruePeak() const
    {
        if (pimpl_->interpolator_.has_value()) {
            return std::max(pimpl_->interpolator_->peak(), lastSamplePeak());
        }
        return lastSamplePeak();
    }

    double Meter::loudnessRangeMultipleHist(const std::vector<const Meter*>& meters)
    {
        std::vector<const HistogramCalculator*> hists;
        hists.reserve(meters.size());
        for (const auto& meter : meters) {
            hists.push_back(dynamic_cast<const HistogramCalculator*>(meter->pimpl_->bs1770_calculator_.get()));
        }
        return HistogramCalculator::loudnessRangeMultiple(hists);
    }

    double Meter::loudnessRangeMultipleBlocks(const std::vector<const Meter*>& meters)
    {
        std::vector<const BlockListCalculator*> lists;
        lists.reserve(meters.size());
        for (const auto& meter : meters) {
            lists.push_back(dynamic_cast<BlockListCalculator*>(meter->pimpl_->bs1770_calculator_.get()));
        }
        return BlockListCalculator::loudnessRangeMultiple(lists);
    }

    double Meter::loudnessGlobalMultiple(const std::vector<const Meter*>& meters)
    {
        double gated_loudness = 0.0;
        double relative_threshold = 0.0;
        std::size_t above_thresh_counter = 0;

        for (const auto& meter : meters) {
            auto value = meter->pimpl_->bs1770_calculator_->blockCountAndSum();
            relative_threshold += value.sum;
            above_thresh_counter += value.count;
        }
        if (above_thresh_counter == 0) {
            return -HUGE_VAL;
        }

        relative_threshold /= static_cast<double>(above_thresh_counter);
        relative_threshold *= relative_gate_factor;

        above_thresh_counter = 0;
        for (const auto& meter : meters) {
            auto value = meter->pimpl_->bs1770_calculator_->loudness(relative_threshold);
            above_thresh_counter += value.count;
            gated_loudness += value.sum;
        }
        if (above_thresh_counter == 0) {
            return -HUGE_VAL;
        }
        gated_loudness /= static_cast<double>(above_thresh_counter);
        return energyToLoudness(gated_loudness);
    }
}  // namespace loudness::detail
