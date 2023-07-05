#ifndef DEFINES_HPP
#define DEFINES_HPP

#include <type_traits>

/** \enum channel
 *  Use these values when setting the channel map with Ebur128::setChannel().
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

/** \enum mode
 *  Use these values for Ebur128 mode. Try to use the lowest possible
 *  modes that suit your needs, as performance will be better.
 */
enum mode : unsigned {
    /** can call Ebur128::loudnessMomentary */
    EBUR128_MODE_M = (1U << 0U),
    /** can call Ebur128::loudnessShortterm */
    EBUR128_MODE_S = (1U << 1U) | EBUR128_MODE_M,
    /** can call Ebur128::loudnessGlobal_* and Ebur128::relativeThreshold */
    EBUR128_MODE_I = (1U << 2U) | EBUR128_MODE_M,
    /** can call Ebur128::loudnessRange */
    EBUR128_MODE_LRA = (1U << 3U) | EBUR128_MODE_M,
    /** can call Ebur128::samplePeak */
    EBUR128_MODE_SAMPLE_PEAK = (1U << 4U),
    /** can call Ebur128::truePeak */
    EBUR128_MODE_TRUE_PEAK = (1U << 5U) | EBUR128_MODE_SAMPLE_PEAK,
    /** uses histogram algorithm to calculate loudness */
    EBUR128_MODE_HISTOGRAM = (1U << 6U)
};

template <typename T>
concept ConstData = std::is_const_v<T> || std::is_const_v<std::remove_pointer_t<T>>;

#endif // DEFINES_HPP
