/* See COPYING file for copyright and license details. */

#ifndef METER_HPP
#define METER_HPP

/** @file meter.hpp
 *  @brief libloudness - a library for loudness measurements according to
 *         ITU-BS.1770
 */

#include "defines.hpp"
#include "detail/meter-impl.hpp"

#include <ranges>
#include <vector>

namespace loudness {
    template <typename T>
    concept HasDataPointer = requires (T t) {
       *t.data();
    };

    template <unsigned Mode> requires (Mode != 0U)
    class Meter {
    public:
        static constexpr unsigned mode = Mode;
        /** @brief Initialize library state.
         *
         *  @tparam mode       A mode bitmap from the Mode enum, decides capabilities of the meter
         *  @param  channels   the number of channels.
         *  @param  samplerate the sample rate.
         */
        Meter(unsigned int channels, unsigned long samplerate) : meter{channels, samplerate, mode}{
        }

        /** @brief Set channel type.
         *
         *  The default is:
         *  - 0 -> Channel::LEFT
         *  - 1 -> Channel::RIGHT
         *  - 2 -> Channel::CENTER
         *  - 3 -> Channel::UNUSED
         *  - 4 -> Channel::LEFT_SURROUND
         *  - 5 -> Channel::RIGHT_SURROUND
         *
         *  @param channel_number zero based channel index.
         *  @param value channel type from the "channel" enum.
         *  @throws std::invalid_argument if channel_number out of range or value invalid
         */
        void setChannel(unsigned int channel_number, Channel value) {
            meter.setChannel(channel_number, value);
        }

        /** @brief Change library parameters.
         *
         *  Note that the channel map will be reset when setting a different number of
         *  channels. The current unfinished block will be lost.
         *
         *  @param  channels   new number of channels.
         *  @param  samplerate new sample rate.
         *  @return bool if parameters were changed or not
         *  @throws std::invalid_argument if channels or samplerate are larger than supported
         */
        bool changeParameters(unsigned int channels, unsigned long samplerate) {
            return meter.changeParameters(channels, samplerate);
        }

        /** @brief Set the maximum window duration.
         *
         *  Set the maximum duration that will be used for loudnessWindow().
         *  Minimum is enforced to 400 ms for MODE_M and 3000 ms for
         *  MODE_S.
         *
         *  @warning This destroys the current content of the audio buffer.
         *
         *  @param  window duration of the window in ms.
         *  @return bool if window duration was changed or not
         *  @throws std::invalid_argument if window is too large
         */
        bool setMaxWindow(unsigned long window) requires ((mode & MODE_M) == MODE_M){
            return meter.setMaxWindow(window);
        }

        /** @brief Set the maximum history.
         *
         *  Set the maximum history that will be stored for loudness integration.
         *  More history provides more accurate results, but requires more resources.
         *
         *  Applies to loudnessRange() and loudnessGlobal() when
         *  MODE_HISTOGRAM is not set.
         *
         *  Default is ULONG_MAX (at least ~50 days).
         *  Minimum is 3000ms for MODE_LRA and 400ms for MODE_M.
         *
         *  @param  history duration of history in ms.
         *  @return bool if history duration was changed or not
         */
        bool setMaxHistory(unsigned long history_ms) requires ((mode & MODE_HISTOGRAM) != MODE_HISTOGRAM) {
            return meter.setMaxHistory(history_ms);
        }

        /** @brief Add frames to be processed.
         *
         *  @param src    Pointer to array of source frames.
         *                Either 1d interleaved, or 2d ordered channel, samples
         *  @param frames Number of frames. Not number of samples!
         */
        void addFrames(DataType src, size_t frames){
            meter.addFrames(src, frames);
        }

        void addFramesSeq(DataType src, size_t frames){
            meter.addFramesSeq(src, frames);
        }

        /** @brief Add frames to be processed.
         *
         *  @param src    Range of individual channels to be processed.
         *                Channels need to be continous
         *  @param frames Number of frames. Not number of samples!
         */
        template <std::ranges::range Range>
        void addFrames(Range src, size_t frames) {
            using T = std::ranges::range_value_t<Range>;
            if constexpr (std::is_pointer_v<T>){
                using U = std::add_pointer_t<std::add_const_t<std::remove_pointer_t<T>>>;
                if constexpr (HasDataPointer<Range> && std::is_const_v<std::remove_pointer_t<T>>){
                    meter.addFrames(src.data(), frames);
                } else {
                    std::vector<U> data;
                    for (auto ptr : src){
                        data.push_back(ptr);
                    }
                    meter.addFrames(data.data(), frames);
                }
            } else if constexpr (HasDataPointer<T>){
                using U = std::add_pointer_t<std::add_const_t<std::remove_pointer_t<decltype(src.begin()->data())>>>;
                std::vector<U> data;
                for (auto container : src){
                    data.push_back(container.data());
                }
                meter.addFrames(data.data(), frames);
            } else {
                // Add it as interleaved/single channel if possible
                meter.addFrames(src.data(), frames);
            }
        }

        /** @brief Get global integrated loudness in LUFS.
         *
         *  @return integrated loudness in LUFS. -HUGE_VAL if result is negative
         *             infinity.
         */
        [[nodiscard]] double loudnessGlobal() const requires ((mode & MODE_I) == MODE_I){
            return meter.loudnessGlobal();
        }

        /** @brief Get global median loudness.
         *
         *  @return integrated loudness in LUFS. -HUGE_VAL if result is negative
         *             infinity.
         */
        [[nodiscard]] double loudnessGlobalMedian() const requires ((mode & MODE_I) == MODE_I){
            return meter.loudnessGlobalMedian();
        }

        /** @brief Get momentary loudness (last 400ms) in LUFS.
         *
         *  @return momentary loudness in LUFS. -HUGE_VAL if result is negative
         *             infinity.
         */
        [[nodiscard]] double loudnessMomentary() const requires ((mode & MODE_M) == MODE_M) {
            return meter.loudnessMomentary();
        }

        /** @brief Get short-term loudness (last 3s) in LUFS.
         *
         *  @return short-term loudness in LUFS. -HUGE_VAL if result is negative
         *             infinity.
         */
        [[nodiscard]] double loudnessShortterm() const requires ((mode & MODE_S) == MODE_S) {
            return meter.loudnessShortterm();
        }

        /** @brief Get loudness of the specified window in LUFS.
         *
         *  window must not be larger than the current window set
         *  The current window can be changed by calling setMaxWindow().
         *
         *  @param  window window in ms to calculate loudness.
         *  @param  out loudness in LUFS. -HUGE_VAL if result is negative infinity.
         *  @throws std::invalid_argument if window is larger than supported
         */
        [[nodiscard]] double loudnessWindow(unsigned long window) const requires ((mode & MODE_M) == MODE_M) {
            return meter.loudnessWindow(window);
        }

        /** @brief Get loudness range (LRA) of programme in LU.
         *
         *  Calculates loudness range according to EBU Tech 3342.
         *
         *  @return loudness range (LRA) in LU.
         */
        [[nodiscard]] double loudnessRange() const requires ((mode & MODE_LRA) == MODE_LRA){
            return meter.loudnessRange();
        }

        /** @brief Get maximum sample peak from all frames that have been processed.
         *
         *  The equation to convert to dBFS is: 20 * log10(out)
         *
         *  @param  channel_number channel to analyse
         *  @return maximum sample peak in linear form (1.0 is 0 dBFS)
         *  @throws std::invalid_argument if channel_number out of range
         */
        [[nodiscard]] double samplePeak(unsigned int channel_number) const requires ((mode & MODE_SAMPLE_PEAK) == MODE_SAMPLE_PEAK){
            return meter.samplePeak(channel_number);
        }

        /** @brief Get maximum sample peak from the last call to addFrames().
         *
         *  The equation to convert to dBFS is: 20 * log10(out)
         *
         *  @param  channel_number channel to analyse
         *  @return maximum sample peak in in linear form (1.0 is 0 dBFS)
         *  @throws std::invalid_argument if channel_number out of range
         */
        [[nodiscard]] double lastSamplePeak(unsigned int channel_number) const requires ((mode & MODE_SAMPLE_PEAK) == MODE_SAMPLE_PEAK) {
            return meter.lastSamplePeak(channel_number);
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
         *  @throws std::invalid_argument if channel_number out of range
         */
        [[nodiscard]] double truePeak(unsigned int channel_number) const requires ((mode & MODE_TRUE_PEAK) == MODE_TRUE_PEAK) {
            return meter.truePeak(channel_number);
        }

        /** @brief Get maximum true peak from the last call to addFrames().
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
         *  @throws std::invalid_argument if channel_number out of range
         */
        [[nodiscard]] double lastTruePeak(unsigned int channel_number) const requires ((mode & MODE_TRUE_PEAK) == MODE_TRUE_PEAK) {
            return meter.lastTruePeak(channel_number);
        }

        /** @brief Get relative threshold in LUFS.
         *
         *  @return relative threshold in LUFS.
         */
        [[nodiscard]] double relativeThreshold() const requires ((mode & MODE_I) == MODE_I){
            return meter.relativeThreshold();
        }

        template<std::ranges::range Range>
        friend double loudnessGlobalMultiple(const Range& meters);

        template<std::ranges::range Range>
        friend double loudnessRangeMultiple(const Range& meters);

    private:
        detail::Meter meter;
    };

    /** @brief Get global integrated loudness in LUFS across multiple instances.
     *
     *  @param  meters range of loudness meters
     *  @return integrated loudness in LUFS. -HUGE_VAL if result is negative infinity.
     */
    template<std::ranges::range Range>
    [[nodiscard]] double loudnessGlobalMultiple(const Range& meters)
    {
        using MeterType = std::ranges::range_value_t<Range>;
        static_assert(std::same_as<MeterType, Meter<MeterType::mode>>);
        static_assert((MeterType::mode & MODE_I) == MODE_I);

        std::vector<const detail::Meter*> pimpls;
        pimpls.reserve(meters.size());
        for (const auto& meter : meters){
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
    template<std::ranges::range Range>
    [[nodiscard]] double loudnessRangeMultiple(const Range& meters)
    {
        using MeterType = std::ranges::range_value_t<Range>;
        static_assert(std::same_as<MeterType, Meter<MeterType::mode>>);
        static_assert((MeterType::mode & MODE_LRA) == MODE_LRA);

        std::vector<const detail::Meter*> pimpls;
        pimpls.reserve(meters.size());
        for (const auto& meter : meters){
            pimpls.push_back(&meter.meter);
        }
        if constexpr ((MeterType::mode & MODE_HISTOGRAM) == MODE_HISTOGRAM) {
            return detail::Meter::loudnessRangeMultipleHist(pimpls);
        } else {
            return detail::Meter::loudnessRangeMultipleBlocks(pimpls);
        }
    }
} // namespace loudness
#endif /* METER_HPP */
