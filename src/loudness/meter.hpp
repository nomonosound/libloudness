/* See LICENSE file for copyright and license details. */

#ifndef LIBLOUDNESS_METER_HPP
#define LIBLOUDNESS_METER_HPP

/** @file meter.hpp
 *  @brief libloudness - a library for loudness measurements according to
 *         ITU-BS.1770 and EBU-R128
 */

#include <ranges>
#include <vector>

#include "loudness/defines.hpp"
#include "loudness/detail/meter-impl.hpp"

namespace loudness {
    template <Mode mode>
    class Meter {
    public:
        /** @brief Initialize library state.
         *
         *  @note NumChannels and Samplerate constructor throw is values out of range
         *
         *  @tparam mode       A mode bitmap from the Mode enum, decides capabilities of the meter
         *  @param  channels   the number of channels.
         *  @param  samplerate the sample rate.
         */
        Meter(NumChannels channels, Samplerate samplerate) : meter{channels, samplerate, mode} {}

        /** @brief Reset meter to initial state, clearing all recorded data
         *
         */
        void reset() { meter.reset(); }

        /** @brief Set channel type.
         *
         *  The default is:
         *  - 0 -> Channel::Left
         *  - 1 -> Channel::Right
         *  - 2 -> Channel::Center
         *  - 3 -> Channel::Unused
         *  - 4 -> Channel::LeftSurround
         *  - 5 -> Channel::RightSurround
         *
         *  @param channel_number zero based channel index.
         *  @param value channel type from the Channel enum.
         *  @throws std::out_of_range if channel_number is out of range
         */
        void setChannel(unsigned int channel_number, Channel value)
            requires((mode & Mode::EBU_M) == Mode::EBU_M)
        {
            meter.setChannel(channel_number, value);
        }

        /** @brief Change library parameters.
         *
         *  @warning The channel map will be reset when setting a different number of
         *  channels. The current unfinished audio block will be lost.
         *
         *  @note NumChannels and Samplerate are responsible for input bounds, and will
         *  throw in their constructors.
         *
         *  @param  channels   new number of channels.
         *  @param  samplerate new sample rate.
         *  @return bool if parameters were changed or not
         */
        bool changeParameters(NumChannels channels, Samplerate samplerate)
        {
            return meter.changeParameters(channels, samplerate);
        }

        /** @brief Set the maximum window duration.
         *
         *  Set the maximum duration that will be used for loudnessWindow().
         *  Clamped to minimum of 400 ms for EBU_M and 3000 ms for
         *  EBU_S.
         *
         *  @warning This destroys the current content of the audio buffer.
         *
         *  @param  window duration of the window in ms.
         *  @return bool if window duration was changed or not
         *  @throws std::domain_error if window is larger than theoretically supported
         */
        bool setMaxWindow(unsigned long window)
            requires((mode & Mode::EBU_M) == Mode::EBU_M)
        {
            return meter.setMaxWindow(window);
        }

        /** @brief Set the maximum history.
         *
         *  Set the maximum history that will be stored for loudness integration.
         *  More history provides more accurate results, but requires more resources.
         *
         *  Applies to loudnessRange() and loudnessGlobal() when
         *  Mode::Histogram is not set.
         *
         *  Default is ULONG_MAX (at least ~50 days).
         *  Clamped to a minimum of 3000ms for EBU_LRA and 400ms for EBU_M.
         *
         *  @param  history duration of history in ms.
         *  @return bool if history duration was changed or not
         */
        bool setMaxHistory(unsigned long history_ms) noexcept
            requires((mode & Mode::Histogram) != Mode::Histogram)
        {
            return meter.setMaxHistory(history_ms);
        }

        /** @brief Add frames to be processed.
         *
         *  @param src    Pointer to array of source frames.
         *                Either 1d interleaved, or 2d ordered channel, samples
         *  @param frames Number of frames. Not number of samples!
         */
        void addFrames(DataType src, std::size_t frames) { meter.addFrames(src, frames); }

        /** @brief Multithreaded version of addFrames
         *
         *  @param src    Pointer to array of source frames.
         *                Either 1d interleaved, or 2d ordered channel, samples
         *  @param frames Number of frames. Not number of samples!
         */
        void addFramesMT(DataType src, std::size_t frames) { meter.addFramesMT(src, frames); }

        /** @brief Add frames to be processed as ranges of channels
         *
         *  @param src    Range of individual channels to be processed.
         *                Channels need to be continous
         *  @param frames Number of frames. Not number of samples!
         */
        template <std::ranges::range Range>
        void addFrames(const Range& src, std::size_t frames)
        {
            meter.addFrames(src, frames);
        }

        /** @brief Multithreaded version of addFrames
         *
         *  @param src    Range of individual channels to be processed.
         *                Channels need to be continous
         *  @param frames Number of frames. Not number of samples!
         */
        template <std::ranges::range Range>
        void addFramesMT(const Range& src, std::size_t frames)
        {
            meter.addFramesMT(src, frames);
        }

        /** @brief Add frames to be processed as ranges of channels
         *
         *  @param src    Range of individual channels to be processed.
         *                Channels need to be continous
         */
        template <std::ranges::range Range>
        void addFrames(const Range& src)
        {
            meter.addFrames(src);
        }

        /** @brief Multithreaded version of addFrames
         *
         *  @param src    Range of individual channels to be processed.
         *                Channels need to be continous
         */
        template <std::ranges::range Range>
        void addFramesMT(const Range& src)
        {
            meter.addFramesMT(src);
        }

        /** @brief Get global integrated loudness in LUFS.
         *
         *  @return integrated loudness in LUFS. -HUGE_VAL if result is negative
         *             infinity.
         */
        [[nodiscard]] double loudnessGlobal() const
            requires((mode & Mode::EBU_I) == Mode::EBU_I)
        {
            return meter.loudnessGlobal();
        }

        /** @brief Get global integrated loudness in LUFS without relative gating
         *
         *  @return integrated loudness in LUFS. -HUGE_VAL if result is negative
         *             infinity.
         */
        [[nodiscard]] double loudnessGlobalUngated() const
            requires((mode & Mode::EBU_I) == Mode::EBU_I)
        {
            return meter.loudnessGlobalUngated();
        }

        /** @brief Get global median loudness.
         *
         *  @return integrated loudness in LUFS. -HUGE_VAL if result is negative
         *             infinity.
         */
        [[nodiscard]] double loudnessGlobalMedian() const
            requires((mode & Mode::EBU_I) == Mode::EBU_I)
        {
            return meter.loudnessGlobalMedian();
        }

        /** @brief Get global median loudness measured after gating.
         *
         *  This first measures the integrated loudness to find the relative gate
         *  like in loudnessGlobal, but then takes the median of the gated loudness.
         *  This differs from loudnessGlobalMedian in that the median is not used for
         *  the gate calculation itself.
         *  @return integrated loudness in LUFS. -HUGE_VAL if result is negative
         *             infinity.
         */
        [[nodiscard]] double loudnessGlobalMedianAfterGate() const
            requires((mode & Mode::EBU_I) == Mode::EBU_I)
        {
            return meter.loudnessGlobalMedianAfterGate();
        }

        /** @brief Get global median loudness without relative gating.
         *
         *  @return integrated loudness in LUFS. -HUGE_VAL if result is negative
         *             infinity.
         */
        [[nodiscard]] double loudnessGlobalMedianUngated() const
            requires((mode & Mode::EBU_I) == Mode::EBU_I)
        {
            return meter.loudnessGlobalMedianUngated();
        }

        /** @brief Get momentary loudness (last 400ms) in LUFS.
         *
         *  @return momentary loudness in LUFS. -HUGE_VAL if result is negative
         *             infinity.
         */
        [[nodiscard]] double loudnessMomentary() const
            requires((mode & Mode::EBU_M) == Mode::EBU_M)
        {
            return meter.loudnessMomentary();
        }

        /** @brief Get short-term loudness (last 3s) in LUFS.
         *
         *  @return short-term loudness in LUFS. -HUGE_VAL if result is negative
         *             infinity.
         */
        [[nodiscard]] double loudnessShortterm() const
            requires((mode & Mode::EBU_S) == Mode::EBU_S)
        {
            return meter.loudnessShortterm();
        }

        /** @brief Get loudness of the specified window in LUFS.
         *
         *  window must not be larger than the current window set
         *  The current window can be changed by calling setMaxWindow().
         *
         *  @param  window_ms window in ms to calculate loudness.
         *  @param  out loudness in LUFS. -HUGE_VAL if result is negative infinity.
         *  @throws std::domain_error if window is larger than currently configured.
         */
        [[nodiscard]] double loudnessWindow(unsigned long window_ms) const
            requires((mode & Mode::EBU_M) == Mode::EBU_M)
        {
            return meter.loudnessWindow(window_ms);
        }

        /** @brief Get loudness range (LRA) of programme in LU.
         *
         *  Calculates loudness range according to EBU Tech 3342.
         *
         *  @return loudness range (LRA) in LU.
         */
        [[nodiscard]] double loudnessRange() const
            requires((mode & Mode::EBU_LRA) == Mode::EBU_LRA)
        {
            return meter.loudnessRange();
        }

        /** @brief Get maximum sample peak from all frames that have been processed.
         *
         *  The equation to convert to dBFS is: 20 * log10(out)
         *
         *  @param  channel_number channel to analyse
         *  @return maximum sample peak in linear form (1.0 is 0 dBFS)
         *  @throws std::out_of_range if channel_number out of range
         */
        [[nodiscard]] double samplePeak(unsigned int channel_number) const
            requires((mode & Mode::SamplePeak) == Mode::SamplePeak)
        {
            return meter.samplePeak(channel_number);
        }
        [[nodiscard]] double samplePeak() const
            requires((mode & Mode::SamplePeak) == Mode::SamplePeak)
        {
            return meter.samplePeak();
        }

        /** @brief Get maximum sample peak from the last call to addFrames().
         *
         *  The equation to convert to dBFS is: 20 * log10(out)
         *
         *  @param  channel_number channel to analyse
         *  @return maximum sample peak in in linear form (1.0 is 0 dBFS)
         *  @throws std::out_of_range if channel_number out of range
         */
        [[nodiscard]] double lastSamplePeak(unsigned int channel_number) const
            requires((mode & Mode::SamplePeak) == Mode::SamplePeak)
        {
            return meter.lastSamplePeak(channel_number);
        }
        [[nodiscard]] double lastSamplePeak() const
            requires((mode & Mode::SamplePeak) == Mode::SamplePeak)
        {
            return meter.lastSamplePeak();
        }

        /** @brief Get maximum true peak from all frames that have been processed.
         *
         *  Uses an implementation defined algorithm to calculate the true peak. Do not
         *  try to compare resulting values across different versions of the library,
         *  as the algorithm may change.
         *
         *  The current implementation uses a custom polyphase FIR interpolator to
         *  calculate true peak. Will oversample 4x for sample rates < 96000 Hz, 2x for
         *  sample rates < 192000 Hz and leave the signal unchanged for >= 192000 Hz.
         *
         *  The equation to convert to dBTP is: 20 * log10(out)
         *
         *  @param  channel_number channel to analyse
         *  @return maximum true peak in linear form (1.0 is 0 dBTP)
         *  @throws std::out_of_range if channel_number out of range
         */
        [[nodiscard]] double truePeak(unsigned int channel_number) const
            requires((mode & Mode::TruePeak) == Mode::TruePeak)
        {
            return meter.truePeak(channel_number);
        }
        [[nodiscard]] double truePeak() const
            requires((mode & Mode::TruePeak) == Mode::TruePeak)
        {
            return meter.truePeak();
        }

        /** @brief Same as truePeak, but only peak from the last call to addFrames().
         *
         *  @param  channel_number channel to analyse
         *  @return maximum true peak in linear form (1.0 is 0 dBTP)
         *  @throws std::out_of_range if channel_number out of range
         */
        [[nodiscard]] double lastTruePeak(unsigned int channel_number) const
            requires((mode & Mode::TruePeak) == Mode::TruePeak)
        {
            return meter.lastTruePeak(channel_number);
        }
        [[nodiscard]] double lastTruePeak() const
            requires((mode & Mode::TruePeak) == Mode::TruePeak)
        {
            return meter.lastTruePeak();
        }

        /** @brief Get relative threshold in LUFS.
         *
         *  @return relative threshold in LUFS.
         */
        [[nodiscard]] double relativeThreshold() const
            requires((mode & Mode::EBU_I) == Mode::EBU_I)
        {
            return meter.relativeThreshold();
        }

        template <template <class> class Range, Mode mode_>
            requires std::ranges::range<Range<Meter<mode_>>> && ((mode_ & Mode::EBU_I) == Mode::EBU_I)
        friend double loudnessGlobalMultiple(const Range<Meter<mode_>>& meters);

        template <template <class> class Range, Mode mode_>
            requires std::ranges::range<Range<Meter<mode_>>> && ((mode_ & Mode::EBU_LRA) == Mode::EBU_LRA)
        friend double loudnessRangeMultiple(const Range<Meter<mode_>>& meters);

    private:
        detail::Meter meter;
    };

    /** @brief Get global integrated loudness in LUFS across multiple instances.
     *
     *  @param  meters range of loudness meters
     *  @return integrated loudness in LUFS. -HUGE_VAL if result is negative infinity.
     */
    template <template <class> class Range, Mode mode>
        requires std::ranges::range<Range<Meter<mode>>> && ((mode & Mode::EBU_I) == Mode::EBU_I)
    [[nodiscard]] double loudnessGlobalMultiple(const Range<Meter<mode>>& meters)
    {
        std::vector<const detail::Meter*> pimpls;
        pimpls.reserve(meters.size());
        for (const auto& meter : meters) {
            pimpls.push_back(&meter.meter);
        }
        return detail::Meter::loudnessGlobalMultiple(pimpls);
    }

    /** @brief Get loudness range (LRA) in LU across multiple instances.
     *
     *  Calculates loudness range according to EBU Tech 3342.
     *
     *  @param  meters range of loudness meters
     *  @return loudness range (LRA) in LU.
     */
    template <template <class> class Range, Mode mode>
        requires std::ranges::range<Range<Meter<mode>>> && ((mode & Mode::EBU_LRA) == Mode::EBU_LRA)
    [[nodiscard]] double loudnessRangeMultiple(const Range<Meter<mode>>& meters)
    {
        std::vector<const detail::Meter*> pimpls;
        pimpls.reserve(meters.size());
        for (const auto& meter : meters) {
            pimpls.push_back(&meter.meter);
        }
        if constexpr ((mode & Mode::Histogram) == Mode::Histogram) {
            return detail::Meter::loudnessRangeMultipleHist(pimpls);
        }
        else {
            return detail::Meter::loudnessRangeMultipleBlocks(pimpls);
        }
    }
}  // namespace loudness
#endif  // LIBLOUDNESS_METER_HPP
