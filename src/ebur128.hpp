/* See COPYING file for copyright and license details. */

#ifndef EBUR128_HPP
#define EBUR128_HPP

/** @file ebur128.hpp
 *  @brief libloudness - a library for loudness measurement according to
 *         EBU-R128 among others.
 */

#define EBUR128_VERSION_MAJOR 1
#define EBUR128_VERSION_MINOR 2
#define EBUR128_VERSION_PATCH 6

#include "detail/ebur128-impl.hpp"

#include <vector>
#include <ranges>


template <unsigned mode> requires (mode != 0U)
class Ebur128 {
public:
    /** @brief Initialize library state.
     *
     *  @tparam mode       A mode bitmap from the Mode enum, decides capabilities of the meter
     *  @param  channels   the number of channels.
     *  @param  samplerate the sample rate.
     */
    Ebur128(unsigned int channels, unsigned long samplerate) : meter{channels, samplerate, mode}{
    }

    /** @brief Set channel type.
     *
     *  The default is:
     *  - 0 -> EBUR128_LEFT
     *  - 1 -> EBUR128_RIGHT
     *  - 2 -> EBUR128_CENTER
     *  - 3 -> EBUR128_UNUSED
     *  - 4 -> EBUR128_LEFT_SURROUND
     *  - 5 -> EBUR128_RIGHT_SURROUND
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
     *  Minimum is enforced to 400 ms for EBUR128_MODE_M and 3000 ms for
     *  EBUR128_MODE_S.
     *
     *  @warning This destroys the current content of the audio buffer.
     *
     *  @param  window duration of the window in ms.
     *  @return bool if window duration was changed or not
     *  @throws std::invalid_argument if window is too large
     */
    bool setMaxWindow(unsigned long window) {
        return meter.setMaxWindow(window);
    }

    /** @brief Set the maximum history.
     *
     *  Set the maximum history that will be stored for loudness integration.
     *  More history provides more accurate results, but requires more resources.
     *
     *  Applies to loudnessRange() and loudnessGlobal() when
     *  EBUR128_MODE_HISTOGRAM is not set.
     *
     *  Default is ULONG_MAX (at least ~50 days).
     *  Minimum is 3000ms for EBUR128_MODE_LRA and 400ms for EBUR128_MODE_M.
     *
     *  @param  history duration of history in ms.
     *  @return bool if history duration was changed or not
     */
    bool setMaxHistory(unsigned long history) requires ((mode & EBUR128_MODE_HISTOGRAM) != EBUR128_MODE_HISTOGRAM) {
        return meter.setMaxHistory(history);
    }

    /** @brief Add frames to be processed.
     *
     *  @param src    array of source frames. Channels must be interleaved.
     *  @param frames number of frames. Not number of samples!
     */
    template <typename T>
    void addFrames(const T* src, size_t frames) {
        meter.addFrames(src, frames);
    }

    /** @brief Get global integrated loudness in LUFS.
     *
     *  @return integrated loudness in LUFS. -HUGE_VAL if result is negative
     *             infinity.
     */
    double loudnessGlobal() requires ((mode & EBUR128_MODE_I) == EBUR128_MODE_I){
        return meter.loudnessGlobal();
    }

    /** @brief Get global median loudness.
     *
     *  @return integrated loudness in LUFS. -HUGE_VAL if result is negative
     *             infinity.
     */
    double loudnessGlobalMedian() requires ((mode & EBUR128_MODE_I) == EBUR128_MODE_I){
        return meter.loudnessGlobalMedian();
    }

    /** @brief Get momentary loudness (last 400ms) in LUFS.
     *
     *  @return momentary loudness in LUFS. -HUGE_VAL if result is negative
     *             infinity.
     */
    double loudnessMomentary() requires ((mode & EBUR128_MODE_M) == EBUR128_MODE_M) {
        return meter.loudnessMomentary();
    }

    /** @brief Get short-term loudness (last 3s) in LUFS.
     *
     *  @return short-term loudness in LUFS. -HUGE_VAL if result is negative
     *             infinity.
     */
    double loudnessShortterm() requires ((mode & EBUR128_MODE_S) == EBUR128_MODE_S) {
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
    double loudnessWindow(unsigned long window) {
        return meter.loudnessWindow(window);
    }

    /** @brief Get loudness range (LRA) of programme in LU.
     *
     *  Calculates loudness range according to EBU Tech 3342.
     *
     *  @return loudness range (LRA) in LU.
     */
    double loudnessRange() requires ((mode & EBUR128_MODE_LRA) == EBUR128_MODE_LRA){
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
    double samplePeak(unsigned int channel_number) const requires ((mode & EBUR128_MODE_SAMPLE_PEAK) == EBUR128_MODE_SAMPLE_PEAK){
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
    double lastSamplePeak(unsigned int channel_number) const requires ((mode & EBUR128_MODE_SAMPLE_PEAK) == EBUR128_MODE_SAMPLE_PEAK) {
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
    double truePeak(unsigned int channel_number) const requires ((mode & EBUR128_MODE_TRUE_PEAK) == EBUR128_MODE_TRUE_PEAK) {
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
    double lastTruePeak(unsigned int channel_number) const requires ((mode & EBUR128_MODE_TRUE_PEAK) == EBUR128_MODE_TRUE_PEAK) {
        return meter.lastTruePeak(channel_number);
    }

    /** @brief Get relative threshold in LUFS.
     *
     *  @return relative threshold in LUFS.
     */
    double relativeThreshold() requires ((mode & EBUR128_MODE_I) == EBUR128_MODE_I){
        return meter.relativeThreshold();
    }

private:
    detail::Ebur128 meter;
};

/** @brief Get global integrated loudness in LUFS across multiple instances.
 *
 *  @param  meters range of loudness meters
 *  @return integrated loudness in LUFS. -HUGE_VAL if result is negative infinity.
 */
template<unsigned mode>
double loudnessGlobalMultiple(std::ranges::range auto&& meters) requires ((mode & EBUR128_MODE_I) == EBUR128_MODE_I)
{
    std::vector<const detail::Ebur128Impl*> pimpls;
    meters.reserve(meters.size());
    for (const auto& meter : meters){
        meters.push_back(meter->meter.pimpl.get());
    }
    return detail::loudnessGlobalMultiple(pimpls);
}

/** @brief Get loudness range (LRA) in LU across multiple instances.
 *
 *  Calculates loudness range according to EBU Tech 3342.
 *
 *  @param  meters range of loudness meters
 *  @return loudness range (LRA) in LU.
 */
template<unsigned mode>
double loudnessRangeMultiple(std::ranges::range auto&& meters) requires ((mode & EBUR128_MODE_LRA) == EBUR128_MODE_LRA)
{
    std::vector<const detail::Ebur128Impl*> pimpls;
    meters.reserve(meters.size());
    for (const auto& meter : meters){
        meters.push_back(meter->meter.pimpl.get());
    }
    if constexpr (mode & EBUR128_MODE_HISTOGRAM){
        return detail::loudnessRangeMultipleHist(pimpls);
    } else {
        return detail::loudnessRangeMultipleBlocks(pimpls);
    }
}

#endif /* EBUR128_HPP */
