/* See COPYING file for copyright and license details. */

#ifndef EBUR128_HPP_
#define EBUR128_HPP_

/** \file ebur128.hpp
 *  \brief libebur128 - a library for loudness measurement according to
 *         the EBU R128 standard.
 */

#define EBUR128_VERSION_MAJOR 1
#define EBUR128_VERSION_MINOR 2
#define EBUR128_VERSION_PATCH 6

#include <cstddef> /* for size_t */
#include <memory>

/** \enum channel
 *  Use these values when setting the channel map with ebur128_set_channel().
 *  See definitions in ITU R-REC-BS 1770-4
 */
enum class Channel {
    UNUSED = 0,         /**< unused channel (for example LFE channel) */
    LEFT = 1,           /**<           */
    Mp030 = 1,          /**< itu M+030 */
    RIGHT = 2,          /**<           */
    Mm030 = 2,          /**< itu M-030 */
    CENTER = 3,         /**<           */
    Mp000 = 3,          /**< itu M+000 */
    LEFT_SURROUND = 4,  /**<           */
    Mp110 = 4,          /**< itu M+110 */
    RIGHT_SURROUND = 5, /**<           */
    Mm110 = 5,          /**< itu M-110 */
    DUAL_MONO,          /**< a channel that is counted twice */
    MpSC,               /**< itu M+SC  */
    MmSC,               /**< itu M-SC  */
    Mp060,              /**< itu M+060 */
    Mm060,              /**< itu M-060 */
    Mp090,              /**< itu M+090 */
    Mm090,              /**< itu M-090 */
    Mp135,              /**< itu M+135 */
    Mm135,              /**< itu M-135 */
    Mp180,              /**< itu M+180 */
    Up000,              /**< itu U+000 */
    Up030,              /**< itu U+030 */
    Um030,              /**< itu U-030 */
    Up045,              /**< itu U+045 */
    Um045,              /**< itu U-030 */
    Up090,              /**< itu U+090 */
    Um090,              /**< itu U-090 */
    Up110,              /**< itu U+110 */
    Um110,              /**< itu U-110 */
    Up135,              /**< itu U+135 */
    Um135,              /**< itu U-135 */
    Up180,              /**< itu U+180 */
    Tp000,              /**< itu T+000 */
    Bp000,              /**< itu B+000 */
    Bp045,              /**< itu B+045 */
    Bm045               /**< itu B-045 */
};

/** \enum error
 *  Error return values.
 */
enum error {
    EBUR128_SUCCESS = 0,
    EBUR128_ERROR_NOMEM,
    EBUR128_ERROR_INVALID_MODE,
    EBUR128_ERROR_INVALID_CHANNEL_INDEX,
    EBUR128_ERROR_NO_CHANGE
};

/** \enum mode
 *  Use these values in ebur128_init (or'ed). Try to use the lowest possible
 *  modes that suit your needs, as performance will be better.
 */
enum mode : unsigned {
    /** can call ebur128_loudness_momentary */
    EBUR128_MODE_M = (1U << 0U),
    /** can call ebur128_loudness_shortterm */
    EBUR128_MODE_S = (1U << 1U) | EBUR128_MODE_M,
    /** can call ebur128_loudness_global_* and ebur128_relative_threshold */
    EBUR128_MODE_I = (1U << 2U) | EBUR128_MODE_M,
    /** can call ebur128_loudness_range */
    EBUR128_MODE_LRA = (1U << 3U) | EBUR128_MODE_S,
    /** can call ebur128_sample_peak */
    EBUR128_MODE_SAMPLE_PEAK = (1U << 4U) | EBUR128_MODE_M,
    /** can call ebur128_true_peak */
    EBUR128_MODE_TRUE_PEAK = (1U << 5U) | EBUR128_MODE_M | EBUR128_MODE_SAMPLE_PEAK,
    /** uses histogram algorithm to calculate loudness */
    EBUR128_MODE_HISTOGRAM = (1U << 6U)
};

enum window : unsigned {
    EBUR128_M = 400,
    EBUR128_S = 3000,
};

// Template window
// Template gating
class Ebur128 {
public:
    /** \brief Initialize library state.
     *
     *  @param channels the number of channels.
     *  @param samplerate the sample rate.
     *  @param mode see the mode enum for possible values.
     *  @return an initialized library state, or NULL on error.
     */
    Ebur128(unsigned int channels, unsigned long samplerate, unsigned int mode);

    ~Ebur128();

    /** \brief Set channel type.
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
     *  @return
     *    - EBUR128_SUCCESS on success.
     *    - EBUR128_ERROR_INVALID_CHANNEL_INDEX if invalid channel index.
     */
    int setChannel(unsigned int channel_number, Channel value);

    /** \brief Change library parameters.
     *
     *  Note that the channel map will be reset when setting a different number of
     *  channels. The current unfinished block will be lost.
     *
     *  @param channels new number of channels.
     *  @param samplerate new sample rate.
     *  @return
     *    - EBUR128_SUCCESS on success.
     *    - EBUR128_ERROR_NOMEM on memory allocation error. The state will be
     *      invalid and must be destroyed.
     *    - EBUR128_ERROR_NO_CHANGE if channels and sample rate were not changed.
     */
    int changeParameters(unsigned int channels, unsigned long samplerate);

    /** \brief Set the maximum window duration.
     *
     *  Set the maximum duration that will be used for ebur128_loudness_window().
     *  Note that this destroys the current content of the audio buffer.
     *
     *  @param window duration of the window in ms.
     *  @return
     *    - EBUR128_SUCCESS on success.
     *    - EBUR128_ERROR_NOMEM on memory allocation error. The state will be
     *      invalid and must be destroyed.
     *    - EBUR128_ERROR_NO_CHANGE if window duration not changed.
     */
    int setMaxWindow(unsigned long window);

    /** \brief Set the maximum history.
     *
     *  Set the maximum history that will be stored for loudness integration.
     *  More history provides more accurate results, but requires more resources.
     *
     *  Applies to ebur128_loudness_range() and ebur128_loudness_global() when
     *  EBUR128_MODE_HISTOGRAM is not set.
     *
     *  Default is ULONG_MAX (at least ~50 days).
     *  Minimum is 3000ms for EBUR128_MODE_LRA and 400ms for EBUR128_MODE_M.
     *
     *  @param history duration of history in ms.
     *  @return
     *    - EBUR128_SUCCESS on success.
     *    - EBUR128_ERROR_NO_CHANGE if history not changed.
     */
    int setMaxHistory(unsigned long history);

    /** \brief Add frames to be processed.
     *
     *  @param src array of source frames. Channels must be interleaved.
     *  @param frames number of frames. Not number of samples!
     *  @return
     *    - EBUR128_SUCCESS on success.
     *    - EBUR128_ERROR_NOMEM on memory allocation error.
     */
    template <typename T>
    int addFrames(const T* src, size_t frames);

    /** \brief Get global integrated loudness in LUFS.
     *
     *  @param out integrated loudness in LUFS. -HUGE_VAL if result is negative
     *             infinity.
     *  @return
     *    - EBUR128_SUCCESS on success.
     *    - EBUR128_ERROR_INVALID_MODE if mode "EBUR128_MODE_I" has not been set.
     */
    int loudnessGlobal(double* out);

    /** \brief Get momentary loudness (last 400ms) in LUFS.
     *
     *  @param out momentary loudness in LUFS. -HUGE_VAL if result is negative
     *             infinity.
     *  @return
     *    - EBUR128_SUCCESS on success.
     */
    int loudnessMomentary(double* out);

    /** \brief Get short-term loudness (last 3s) in LUFS.
     *
     *  @param out short-term loudness in LUFS. -HUGE_VAL if result is negative
     *             infinity.
     *  @return
     *    - EBUR128_SUCCESS on success.
     *    - EBUR128_ERROR_INVALID_MODE if mode "EBUR128_MODE_S" has not been set.
     */
    int loudnessShortterm(double* out);

    /** \brief Get loudness of the specified window in LUFS.
     *
     *  window must not be larger than the current window set in st.
     *  The current window can be changed by calling ebur128_set_max_window().
     *
     *  @param window window in ms to calculate loudness.
     *  @param out loudness in LUFS. -HUGE_VAL if result is negative infinity.
     *  @return
     *    - EBUR128_SUCCESS on success.
     *    - EBUR128_ERROR_INVALID_MODE if window larger than current window in st.
     */
    int loudnessWindow(unsigned long window, double* out);

    /** \brief Get loudness range (LRA) of programme in LU.
     *
     *  Calculates loudness range according to EBU 3342.
     *
     *  @param out loudness range (LRA) in LU. Will not be changed in case of
     *             error. EBUR128_ERROR_NOMEM or EBUR128_ERROR_INVALID_MODE will be
     *             returned in this case.
     *  @return
     *    - EBUR128_SUCCESS on success.
     *    - EBUR128_ERROR_INVALID_MODE if mode "EBUR128_MODE_LRA" has not been set.
     */
    int loudnessRange(double* out);

    /** \brief Get maximum sample peak from all frames that have been processed.
     *
     *  The equation to convert to dBFS is: 20 * log10(out)
     *
     *  @param channel_number channel to analyse
     *  @param out maximum sample peak in float format (1.0 is 0 dBFS)
     *  @return
     *    - EBUR128_SUCCESS on success.
     *    - EBUR128_ERROR_INVALID_MODE if mode "EBUR128_MODE_SAMPLE_PEAK" has not
     *      been set.
     *    - EBUR128_ERROR_INVALID_CHANNEL_INDEX if invalid channel index.
     */
    int samplePeak(unsigned int channel_number, double* out) const;

    /** \brief Get maximum sample peak from the last call to add_frames().
     *
     *  The equation to convert to dBFS is: 20 * log10(out)
     *
     *  @param channel_number channel to analyse
     *  @param out maximum sample peak in float format (1.0 is 0 dBFS)
     *  @return
     *    - EBUR128_SUCCESS on success.
     *    - EBUR128_ERROR_INVALID_MODE if mode "EBUR128_MODE_SAMPLE_PEAK" has not
     *      been set.
     *    - EBUR128_ERROR_INVALID_CHANNEL_INDEX if invalid channel index.
     */
    int prevSamplePeak(unsigned int channel_number, double* out) const;

    /** \brief Get maximum true peak from all frames that have been processed.
     *
     *  Uses an implementation defined algorithm to calculate the true peak. Do not
     *  try to compare resulting values across different versions of the library,
     *  as the algorithm may change.
     *
     *  The current implementation uses a custom polyphase FIR interpolator to
     *  calculate true peak. Will oversample 4x for sample rates < 96000 Hz, 2x for
     *  sample rates < 192000 Hz and leave the signal unchanged for 192000 Hz.
     *
     *  The equation to convert to dBTP is: 20 * log10(out)
     *
     *  @param channel_number channel to analyse
     *  @param out maximum true peak in float format (1.0 is 0 dBTP)
     *  @return
     *    - EBUR128_SUCCESS on success.
     *    - EBUR128_ERROR_INVALID_MODE if mode "EBUR128_MODE_TRUE_PEAK" has not
     *      been set.
     *    - EBUR128_ERROR_INVALID_CHANNEL_INDEX if invalid channel index.
     */
    int truePeak(unsigned int channel_number, double* out) const;

    /** \brief Get maximum true peak from the last call to add_frames().
     *
     *  Uses an implementation defined algorithm to calculate the true peak. Do not
     *  try to compare resulting values across different versions of the library,
     *  as the algorithm may change.
     *
     *  The current implementation uses a custom polyphase FIR interpolator to
     *  calculate true peak. Will oversample 4x for sample rates < 96000 Hz, 2x for
     *  sample rates < 192000 Hz and leave the signal unchanged for 192000 Hz.
     *
     *  The equation to convert to dBTP is: 20 * log10(out)
     *
     *  @param channel_number channel to analyse
     *  @param out maximum true peak in float format (1.0 is 0 dBTP)
     *  @return
     *    - EBUR128_SUCCESS on success.
     *    - EBUR128_ERROR_INVALID_MODE if mode "EBUR128_MODE_TRUE_PEAK" has not
     *      been set.
     *    - EBUR128_ERROR_INVALID_CHANNEL_INDEX if invalid channel index.
     */
    int prevTruePeak(unsigned int channel_number, double* out) const;

    /** \brief Get relative threshold in LUFS.
     *
     *  @param out relative threshold in LUFS.
     *  @return
     *    - EBUR128_SUCCESS on success.
     *    - EBUR128_ERROR_INVALID_MODE if mode "EBUR128_MODE_I" has not
     *      been set.
     */
    int relativeThreshold(double* out);

    static constexpr unsigned int MAX_CHANNELS = 64;
    static constexpr unsigned long MIN_SAMPLERATE = 16;
    static constexpr unsigned long MAX_SAMPLERATE = 2822400;

    const unsigned int mode;                            /**< The current mode. */
    unsigned int channels;                     /**< The number of channels. */
    unsigned long samplerate;                  /**< The sample rate. */
    std::unique_ptr<struct Ebur128Impl> pimpl; /**< Internal state. */

private:
    template <typename T>
    void filter(const T* src, size_t frames);
};

/** \brief Get library version number. Do not pass null pointers here.
 *
 *  @param major major version number of library
 *  @param minor minor version number of library
 *  @param patch patch version number of library
 */
void ebur128_get_version(int* major, int* minor, int* patch);

/** \brief Get global integrated loudness in LUFS across multiple instances.
 *
 *  @param sts array of library states.
 *  @param size length of sts
 *  @param out integrated loudness in LUFS. -HUGE_VAL if result is negative
 *             infinity.
 *  @return
 *    - EBUR128_SUCCESS on success.
 *    - EBUR128_ERROR_INVALID_MODE if mode "EBUR128_MODE_I" has not been set.
 */
int ebur128_loudness_global_multiple(Ebur128* begin, Ebur128* end, double* out);

/** \brief Get loudness range (LRA) in LU across multiple instances.
 *
 *  Calculates loudness range according to EBU 3342.
 *
 *  @param sts array of library states.
 *  @param size length of sts
 *  @param out loudness range (LRA) in LU. Will not be changed in case of
 *             error. EBUR128_ERROR_NOMEM or EBUR128_ERROR_INVALID_MODE will be
 *             returned in this case.
 *  @return
 *    - EBUR128_SUCCESS on success.
 *    - EBUR128_ERROR_NOMEM in case of memory allocation error.
 *    - EBUR128_ERROR_INVALID_MODE if mode "EBUR128_MODE_LRA" has not been set.
 */
// int ebur128_loudness_range_multiple(Ebur128* begin,
//                                     Ebur128* end,
//                                     double* out);

#endif /* EBUR128_HPP_ */
