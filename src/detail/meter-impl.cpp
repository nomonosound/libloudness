/* See COPYING file for copyright and license details. */

#include "detail/meter-impl.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <deque>
#include <numeric>
#include <type_traits>
#include <vector>

#include <gcem.hpp>

#include "bs1770-calculator.hpp"
#include "constants.hpp"
#include "interpolator.hpp"
#include "k-filter.hpp"
#include "utils.hpp"

#include "BS_thread_pool.hpp"

namespace loudness::detail {
    struct DataMessage {
        loudness::DataType data;
        size_t frames;
    };

    struct Impl {
        Impl(unsigned int channels, unsigned long samplerate, Mode mode);

        /** Number of subblocks to store. Each subblock is 100ms*/
        static constexpr int num_subblocks = st_subblocks;

        void initChannelMap(size_t num_channels);
        void recalcChannelWeights();
        void calcSubBlocks();
        void addIntegrationBlock();
        void addShorttermBlock();
        [[nodiscard]] double energyInInterval(size_t interval_frames) const;

        void addFramesLoudness(DataType src, size_t frames);

        template <ConstData T>
        void filter(T* src, size_t offset, size_t frames);

        template <ConstData T>
        void findSamplePeaks(T* src, size_t frames, size_t channels);

        /** Filtered audio data (used as ring buffer). */
        std::vector<std::vector<double>> audio_data;
        /** Size of audio_data array. */
        size_t audio_data_frames;
        /** Current index for audio_data. */
        size_t audio_data_index = 0;

        /** Data for subblocks for gating blocks */
        size_t calculated_audio_index = 0;
        std::vector<std::deque<double>> calculated_subblocks;

        /** The channel map. Has as many elements as there are channels. */
        std::vector<Channel> channel_map;

        std::vector<double> channel_weight;

        /** How many samples fit in 100ms (rounded). */
        unsigned long samples_in_100ms;
        /** How many frames are needed for a gating block. Will correspond to 400ms
         *  of audio at initialization, and 100ms after the first block (75% overlap
         *  as specified in the 2011 revision of BS1770). */
        unsigned long needed_frames;
        KFilter k_filter;

        /** Keeps track of when a new short term block is needed. */
        size_t short_term_frame_counter = 0;
        /** Maximum sample peak, one per channel */
        std::vector<double> sample_peak;
        std::vector<double> last_sample_peak;
        /** Maximum true peak, one per channel */
        std::vector<double> true_peak;
        /** The maximum window duration in ms. */
        unsigned long window_ms;

        std::unique_ptr<Interpolator> resampler;
        std::unique_ptr<BS1770Calculator> bs1770_calculator;

        BS::thread_pool pool;
        Mode mode;
    };

    void Impl::initChannelMap(size_t num_channels)
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
        recalcChannelWeights();
    }

    void Impl::recalcChannelWeights()
    {
        channel_weight.clear();
        for (const auto& channel : channel_map){
            switch (channel) {
            case Channel::Mp110: case Channel::Mm110: case Channel::Mp060: case Channel::Mm060: case Channel::Mp090: case  Channel::Mm090:
                channel_weight.push_back(1.41);
                break;
            case Channel::DUAL_MONO:
                channel_weight.push_back(2.0);
                break;
            case Channel::UNUSED:
                channel_weight.push_back(0.0);
                break;
            [[likely]] default:
                channel_weight.push_back(1.0);
            }
        }
    }

    Meter::Meter(unsigned int channels, unsigned long samplerate, Mode mode)
        : mode(mode), channels(channels), samplerate(samplerate)
    {
        assert(channels > 0 && channels <= MAX_CHANNELS);
        assert(samplerate >= MIN_SAMPLERATE && samplerate <= MAX_SAMPLERATE);

        pimpl = std::make_unique<Impl>(channels, samplerate, mode);
    }

    Meter::~Meter() noexcept = default;
    Meter::Meter(Meter&& other) noexcept = default;

    Impl::Impl(unsigned int channels, unsigned long samplerate, Mode mode)
        : calculated_subblocks(channels, std::deque<double>()),
          samples_in_100ms((samplerate + 5) / 10),
          /* the first block needs 400ms of audio data */
          needed_frames(samples_in_100ms * m_subblocks),
          k_filter(static_cast<double>(samplerate), channels),
          sample_peak(channels),
          last_sample_peak(channels),
          true_peak(channels),
          mode(mode)
    {
        initChannelMap(channels);

        if ((mode & Mode::Histogram) == Mode::Histogram) {
            bs1770_calculator = std::make_unique<HistogramCalculator>();
        }
        else {
            bs1770_calculator = std::make_unique<BlockListCalculator>();
        }

        if ((mode & Mode::EBU_S) == Mode::EBU_S) {
            window_ms = shortterm_block_ms;
        }
        else if ((mode & Mode::EBU_M) == Mode::EBU_M) {
            window_ms = momentary_block_ms;
        } else {
            // Disable window
            window_ms = 0;
        }

        audio_data_frames = samplerate * window_ms / 1000;
        if (audio_data_frames % samples_in_100ms != 0) {
            /* round up to multiple of samples_in_100ms */
            audio_data_frames = (audio_data_frames + samples_in_100ms) - (audio_data_frames % samples_in_100ms);
        }
        audio_data.assign(channels, std::vector<double>(audio_data_frames));

        if ((mode & Mode::TruePeak) == Mode::TruePeak) {
            if (samplerate < 96000) {
                resampler = std::make_unique<Interpolator>(49, 4, channels);
            }
            else if (samplerate < 192000) {
                resampler = std::make_unique<Interpolator>(49, 2, channels);
            }
            else {
                resampler = nullptr;
            }
        }
    }

    template <ConstData T>
    void Impl::filter(T* src, size_t offset, size_t frames)
    {
        // Apply filter
        const size_t channels = audio_data.size();
        for (size_t c = 0; c < channels; ++c) {
            if (channel_map[c] == Channel::UNUSED) {
                continue;
            }
            double* audio = audio_data[c].data() + audio_data_index;
            if constexpr (std::is_pointer_v<T>){
                std::transform(src[c] + offset, src[c] + offset + frames, audio,[this, c](auto val){
                    val = k_filter.apply(static_cast<double>(val) / getScalingFactor<std::remove_pointer_t<T>>(), c);
                    return val * val;
                });
            } else {
                for (size_t i = 0; i < frames; ++i) {
                    audio[i] = k_filter.apply(static_cast<double>(src[(i + offset) * channels + c]) / getScalingFactor<T>(), c);
                    audio[i] *= audio[i];
                }
            }
            #ifdef MANUALLY_FTZ
            pimpl->filter.manuallyFTZ(c);
            #endif
        }
        audio_data_index += frames;
    }


    template<ConstData T>
    void Impl::findSamplePeaks(T* src, size_t frames, size_t chan)
    {
        if constexpr (std::is_pointer_v<T>){
            using U = std::remove_pointer_t<T>;
            auto minmax = std::minmax_element(src[chan], src[chan] + frames);
            last_sample_peak[chan] = static_cast<double>(std::max<U>(-*minmax.first, *minmax.second)) / getScalingFactor<U>();
        } else {
            const size_t channels = sample_peak.size();
            double max = 0.0;
            for (size_t i = 0; i < frames; ++i) {
                const double cur = std::abs(static_cast<double>(src[i * channels + chan]));
                if (cur > max) {
                    max = cur;
                }
            }
            last_sample_peak[chan] = max / getScalingFactor<T>();
        }
        if (last_sample_peak[chan] > sample_peak[chan]) {
            sample_peak[chan] = last_sample_peak[chan];
        }
    }

    double Impl::energyInInterval(size_t interval_frames) const
    {
        double sum = 0.0;
        for (size_t c = 0; c < channel_map.size(); ++c) {
            if (channel_map[c] == Channel::UNUSED) {
                continue;
            }
            double channel_sum = 0.0;
            if (audio_data_index < interval_frames) {
                // Read over the seam in the circular buffer
                channel_sum += std::reduce(audio_data[c].cbegin(), audio_data[c].cbegin() + audio_data_index, 0.0) +
                               std::reduce(audio_data[c].crbegin(), audio_data[c].crbegin() + interval_frames - audio_data_index, 0.0);
            }
            else {
                channel_sum += std::reduce(audio_data[c].cbegin() + audio_data_index - interval_frames, audio_data[c].cbegin() + audio_data_index, 0.0);
            }
            channel_sum *= channel_weight[c];

            sum += channel_sum;
        }

        return sum / static_cast<double>(interval_frames);
    }

    void Impl::addFramesLoudness(DataType src, size_t frames)
    {
        const ScopedFTZ guard;
        size_t src_index = 0;
        while (frames > 0) {
            if (frames >= needed_frames) {
                std::visit([this, src_index](auto&& src){
                    filter(src, src_index, needed_frames);
                }, src);
                src_index += needed_frames;
                frames -= needed_frames;
                /* calculate the new gating block */
                if ((mode & Mode::EBU_I) == Mode::EBU_I) {
                    calcSubBlocks();
                    addIntegrationBlock();
                }
                if ((mode & Mode::EBU_LRA) == Mode::EBU_LRA) {
                    calcSubBlocks();
                    short_term_frame_counter += needed_frames;
                    if (short_term_frame_counter == samples_in_100ms * st_subblocks) {
                        addShorttermBlock();
                        short_term_frame_counter = samples_in_100ms * 20;
                    }
                }
                /* 100ms are needed for all blocks besides the first one */
                needed_frames = samples_in_100ms;
                /* reset audio_data_index when buffer full */
                if (audio_data_index == audio_data_frames) {
                    audio_data_index = 0;
                }
                if (calculated_audio_index == audio_data_frames) {
                    calculated_audio_index = 0;
                }
            }
            else {
                std::visit([this, src_index, frames](auto&& src){
                    filter(src, src_index, frames);
                }, src);
                if ((mode & Mode::EBU_LRA) == Mode::EBU_LRA) {
                    short_term_frame_counter += frames;
                }
                needed_frames -= static_cast<unsigned long>(frames);
                frames = 0;
            }
        }
    }


    void Impl::calcSubBlocks()
    {
        for (;calculated_audio_index + samples_in_100ms <= audio_data_index; calculated_audio_index += samples_in_100ms){
            for (size_t c = 0; c < channel_map.size(); ++c) {
                if (channel_map[c] == Channel::UNUSED){
                    continue;
                }
                calculated_subblocks[c].push_back(
                    std::reduce(audio_data[c].cbegin() + calculated_audio_index, audio_data[c].cbegin() + calculated_audio_index + samples_in_100ms, 0.0)
                );
                if (calculated_subblocks[c].size() > num_subblocks) [[likely]] {
                    calculated_subblocks[c].pop_front();
                }
            }
        }
    }

    void Impl::addIntegrationBlock()
    {
        double sum = 0.0;
        for (size_t c = 0; c < channel_weight.size(); ++c) {
            if (channel_map[c] != Channel::UNUSED){
                sum += channel_weight[c] * std::reduce(calculated_subblocks[c].crbegin(), calculated_subblocks[c].crbegin() + m_subblocks, 0.0);
            }
        }
        sum /= static_cast<double>(m_subblocks*samples_in_100ms);
        if (sum >= absolute_gate) [[likely]] {
            bs1770_calculator->addBlock(sum);
        }
    }

    void Impl::addShorttermBlock()
    {
        double sum = 0.0;
        for (size_t c = 0; c < channel_weight.size(); ++c) {
            if (channel_map[c] != Channel::UNUSED){
                sum += channel_weight[c] * std::reduce(calculated_subblocks[c].crbegin(), calculated_subblocks[c].crbegin() + st_subblocks, 0.0);
            }
        }
        sum /= static_cast<double>(st_subblocks*samples_in_100ms);
        if (sum >= absolute_gate) [[likely]] {
            bs1770_calculator->addShortTermBlock(sum);
        }
    }

    void Meter::setChannel(unsigned int channel_index, Channel value)
    {
        if (channel_index > channels) {
            throw std::invalid_argument("Requested channels larger than maximum");
        }
        if (value == Channel::DUAL_MONO && (channels != 1 || channel_index != 0)) {
            throw std::invalid_argument("Channel::DUAL_MONO only works with mono files!");
        }
        if (pimpl->channel_map[channel_index] != value){
            pimpl->channel_map[channel_index] = value;
            pimpl->recalcChannelWeights();
        }
    }

    bool Meter::changeParameters(unsigned int channels, unsigned long samplerate)
    {
        if (channels == 0 || channels > max_channels) {
            throw std::invalid_argument("Requested channels larger than maximum");
        }

        if (samplerate < min_samplerate || samplerate > max_samplerate) {
            throw std::invalid_argument("Requested samplerate larger whan maximum");
        }

        if (this->channels == channels && this->samplerate == samplerate) {
            return false;
        }

        if (channels != this->channels) {
            pimpl->calculated_subblocks.assign(channels, std::deque<double>());
            pimpl->sample_peak.assign(channels, 0.0);
            pimpl->last_sample_peak.assign(channels, 0.0);
            pimpl->true_peak.assign(channels, 0.0);

            pimpl->initChannelMap(channels);
        }
        if (samplerate != this->samplerate) {
            this->samplerate = samplerate;
            pimpl->samples_in_100ms = (samplerate + 5) / 10;
        }

        /* If we're here, either samplerate or channels
         * have changed. Re-init filter. */
        pimpl->k_filter = KFilter(static_cast<double>(samplerate), channels);

        pimpl->audio_data_frames = samplerate * pimpl->window_ms / 1000;
        if (pimpl->audio_data_frames % pimpl->samples_in_100ms != 0) {
            /* round up to multiple of samples_in_100ms */
            pimpl->audio_data_frames =
                (pimpl->audio_data_frames + pimpl->samples_in_100ms) - (pimpl->audio_data_frames % pimpl->samples_in_100ms);
        }
        pimpl->audio_data.assign(channels, std::vector<double>(pimpl->audio_data_frames));

        if ((mode & Mode::TruePeak) == Mode::TruePeak) {
            if (samplerate < 96000) {
                pimpl->resampler = std::make_unique<Interpolator>(49, 4, channels);
            }
            else if (samplerate < 192000) {
                pimpl->resampler = std::make_unique<Interpolator>(49, 2, channels);
            }
            else {
                pimpl->resampler = nullptr;
            }
        }
        /* the first block needs 400ms of audio data */
        pimpl->needed_frames = pimpl->samples_in_100ms * m_subblocks;
        /* start at the beginning of the buffer */
        pimpl->audio_data_index = 0;
        /* reset short term frame counter */
        pimpl->short_term_frame_counter = 0;

        return true;
    }

    bool Meter::setMaxWindow(unsigned long window_ms)
    {
        if ((mode & Mode::EBU_S) == Mode::EBU_S && window_ms < shortterm_block_ms) {
            window_ms = shortterm_block_ms;
        }
        else if ((mode & Mode::EBU_M) == Mode::EBU_M && window_ms < momentary_block_ms) {
            window_ms = momentary_block_ms;
        }

        if (window_ms == pimpl->window_ms) {
            return false;
        }

        size_t new_audio_data_frames;
        if (not safeSizeMul(samplerate, window_ms, &new_audio_data_frames) ||
            new_audio_data_frames > std::numeric_limits<size_t>::max() - pimpl->samples_in_100ms) {
            throw std::invalid_argument("Requested window too large");
        }
        if (new_audio_data_frames % pimpl->samples_in_100ms != 0) {
            /* round up to multiple of samples_in_100ms */
            new_audio_data_frames =
                (new_audio_data_frames + pimpl->samples_in_100ms) - (new_audio_data_frames % pimpl->samples_in_100ms);
        }

        pimpl->window_ms = window_ms;
        pimpl->audio_data_frames = new_audio_data_frames;
        pimpl->audio_data.assign(channels, std::vector<double>(pimpl->audio_data_frames));

        /* the first block needs 400ms of audio data */
        pimpl->needed_frames = pimpl->samples_in_100ms * m_subblocks;
        /* start at the beginning of the buffer */
        pimpl->audio_data_index = 0;
        /* reset short term frame counter */
        pimpl->short_term_frame_counter = 0;

        return true;
    }

    bool Meter::setMaxHistory(unsigned long history_ms)
    {
        if ((mode & Mode::EBU_LRA) == Mode::EBU_LRA && history_ms < shortterm_block_ms) {
            history_ms = shortterm_block_ms;
        }
        else if ((mode & Mode::EBU_M) == Mode::EBU_M && history_ms < momentary_block_ms) {
            history_ms = momentary_block_ms;
        }

        return pimpl->bs1770_calculator->setMaxHistory(history_ms);
    }


    void Meter::addFrames(DataType src, size_t frames)
    {
//        addFramesSeq(src, frames);

        if ((mode & Mode::TruePeak) == Mode::TruePeak) {
            for (size_t c = 0; c < channels; ++c){
                pimpl->pool.push_task([this, c, src, frames] {
                    const ScopedFTZ guard;
                    std::visit([this, c, frames](auto&& src){
                        pimpl->resampler->process(src, frames, c);
                    }, src);
                    if (pimpl->resampler->peak(c) > pimpl->true_peak[c]) {
                        pimpl->true_peak[c] = pimpl->resampler->peak(c);
                    }
                });
            }
        }
        if ((mode & Mode::EBU_M) == Mode::EBU_M){
            pimpl->pool.push_task(&Impl::addFramesLoudness, pimpl.get(), src, frames);
        }
        // Sample peak so lightweight it is never worth parallelizing channels
        if ((mode & Mode::SamplePeak) == Mode::SamplePeak) {
            pimpl->pool.push_task([this, src, frames]{
                std::visit([this, frames](auto&& src){
                    for (size_t c = 0; c < channels; ++c){
                        pimpl->findSamplePeaks(src, frames, c);
                    }
                }, src);
            });
        }

        pimpl->pool.wait_for_tasks();
    }


    void Meter::addFramesSeq(DataType src, size_t frames)
    {
        ScopedFTZ guard;
        // Find new sample peak
        if ((mode & Mode::SamplePeak) == Mode::SamplePeak) {
            for (size_t c = 0; c < channels; ++c){
                std::visit([this, frames, c](auto&& src){
                    pimpl->findSamplePeaks(src, frames, c);
                }, src);
            }
        }
        // Find new true peak
        if (pimpl->resampler) {
            for (size_t c = 0; c < channels; ++c){
                std::visit([this, frames, c](auto&& src){
                    pimpl->resampler->process(src, frames, c);
                }, src);
                if (pimpl->resampler->peak(c) > pimpl->true_peak[c]) {
                    pimpl->true_peak[c] = pimpl->resampler->peak(c);
                }
            }
        }

        if ((mode & Mode::EBU_M) == Mode::EBU_M){
            pimpl->addFramesLoudness(src, frames);
        }
    }

    double Meter::relativeThreshold() const
    {
        auto [above_thresh_counter, relative_threshold] = pimpl->bs1770_calculator->relativeThreshold();

        if (above_thresh_counter == 0) {
            return abs_threshold;
        }

        relative_threshold /= static_cast<double>(above_thresh_counter);
        relative_threshold *= relative_gate_factor;

        return energyToLoudness(relative_threshold);
    }

    double Meter::loudnessGlobal() const {
        auto [above_thresh_counter, relative_threshold] = pimpl->bs1770_calculator->relativeThreshold();
        if (above_thresh_counter == 0) {
            return -HUGE_VAL;
        }

        relative_threshold /= static_cast<double>(above_thresh_counter);
        relative_threshold *= relative_gate_factor;

        auto [above_rel_counter, gated_loudness] = pimpl->bs1770_calculator->gatedLoudness(relative_threshold);

        return above_rel_counter == 0 ? -HUGE_VAL : energyToLoudness(gated_loudness / static_cast<double>(above_rel_counter));
    }

    double Meter::loudnessMomentary() const
    {
        const double energy = pimpl->energyInInterval(pimpl->samples_in_100ms * m_subblocks);
        return energy <= 0.0 ? -HUGE_VAL : energyToLoudness(energy);
    }

    double Meter::loudnessShortterm() const
    {
        const double energy = pimpl->energyInInterval(pimpl->samples_in_100ms * st_subblocks);
        return energy <= 0.0 ? -HUGE_VAL :  energyToLoudness(energy);
    }

    double Meter::loudnessWindow(unsigned long window_ms) const
    {
        const size_t interval_frames = (samplerate * window_ms) / 1000;

        if (window_ms > pimpl->window_ms || interval_frames > pimpl->audio_data_frames) {
            throw std::invalid_argument("Given window too large for current mode");
        }

        const double energy = pimpl->energyInInterval(interval_frames);
        return energy <= 0.0 ? -HUGE_VAL : energyToLoudness(energy);
    }

    double Meter::loudnessRange() const {
        // TODO: Find better solution here, avoiding dynamic cast
        if ((mode & Mode::Histogram) == Mode::Histogram) {
            return HistogramCalculator::loudnessRangeMultiple({dynamic_cast<HistogramCalculator*>(pimpl->bs1770_calculator.get())});
        } else {
            return  BlockListCalculator::loudnessRangeMultiple({dynamic_cast<BlockListCalculator*>(pimpl->bs1770_calculator.get())});
        }
    }

    double Meter::loudnessGlobalMedian() const
    {
        auto [above_thresh_counter, relative_threshold] = pimpl->bs1770_calculator->relativeThreshold();
        if (above_thresh_counter == 0) {
            return -HUGE_VAL;
        }

        relative_threshold /= static_cast<double>(above_thresh_counter);
        relative_threshold *= relative_gate_factor;

        return energyToLoudness(pimpl->bs1770_calculator->gatedMedianLoudness(relative_threshold));
    }

    double Meter::samplePeak(unsigned int channel_index) const
    {
        if (channel_index >= channels) {
            throw std::invalid_argument("Invalid channel index");
        }

        return pimpl->sample_peak[channel_index];
    }

    double Meter::lastSamplePeak(unsigned int channel_index) const
    {
        if (channel_index >= channels) {
            throw std::invalid_argument("Invalid channel index");
        }

        return pimpl->last_sample_peak[channel_index];
    }

    double Meter::truePeak(unsigned int channel_index) const
    {
        if (channel_index >= channels) {
            throw std::invalid_argument("Invalid channel index");
        }

        return std::max(pimpl->true_peak[channel_index], pimpl->sample_peak[channel_index]);
    }

    double Meter::lastTruePeak(unsigned int channel_index) const
    {
        if (channel_index >= channels) {
            throw std::invalid_argument("Invalid channel index");
        }
        if (pimpl->resampler){
            return std::max(static_cast<double>(pimpl->resampler->peak(channel_index)),
                            pimpl->last_sample_peak[channel_index]);
        }
        return pimpl->last_sample_peak[channel_index];

    }

    double Meter::loudnessRangeMultipleHist(const std::vector<const Meter*>& meters)
    {
        std::vector<const HistogramCalculator*> hists;
        hists.reserve(meters.size());
        for (const auto& meter : meters){
            hists.push_back(dynamic_cast<const HistogramCalculator*>(meter->pimpl->bs1770_calculator.get()));
        }
        return HistogramCalculator::loudnessRangeMultiple(hists);
    }

    double Meter::loudnessRangeMultipleBlocks(const std::vector<const Meter*>& meters)
    {
        std::vector<const BlockListCalculator*> lists;
        lists.reserve(meters.size());
        for (const auto& meter : meters){
            lists.push_back(dynamic_cast<BlockListCalculator*>(meter->pimpl->bs1770_calculator.get()));
        }
        return BlockListCalculator::loudnessRangeMultiple(lists);
    }

    double Meter::loudnessGlobalMultiple(const std::vector<const Meter*>& meters)
    {
        double gated_loudness = 0.0;
        double relative_threshold = 0.0;
        size_t above_thresh_counter = 0;

        for (const auto& meter : meters) {
            auto value = meter->pimpl->bs1770_calculator->relativeThreshold();
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
            auto value = meter->pimpl->bs1770_calculator->gatedLoudness(relative_threshold);
            above_thresh_counter += value.counter;
            gated_loudness += value.sum;
        }
        if (above_thresh_counter == 0) {
            return -HUGE_VAL;
        }
        gated_loudness /= static_cast<double>(above_thresh_counter);
        return energyToLoudness(gated_loudness);
    }
} // namespace loudness::detail
