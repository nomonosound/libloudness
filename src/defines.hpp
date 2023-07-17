#ifndef DEFINES_HPP
#define DEFINES_HPP

#include <type_traits>
#include <cstdint>
#include <variant>

namespace loudness {
    /** \enum Channel
     *  Use these values when setting the channel map with Meter::setChannel().
     *  See definitions in ITU R-REC-BS 1770-4
     */
    enum class Channel : uint_fast8_t {
        UNUSED = 0,         /**< unused channel (for example LFE channel) */
        LEFT = 1,           /**<           */
        Mp030 = 1,          /**< ITU M+030 */
        RIGHT = 2,          /**<           */
        Mm030 = 2,          /**< ITU M-030 */
        CENTER = 3,         /**<           */
        Mp000 = 3,          /**< ITU M+000 */
        LEFT_SURROUND = 4,  /**<           */
        Mp110 = 4,          /**< ITU M+110 */
        RIGHT_SURROUND = 5, /**<           */
        Mm110 = 5,          /**< ITU M-110 */
        DUAL_MONO,          /**< a channel that is counted twice */
        MpSC,               /**< ITU M+SC  */
        MmSC,               /**< ITU M-SC  */
        Mp060,              /**< ITU M+060 */
        Mm060,              /**< ITU M-060 */
        Mp090,              /**< ITU M+090 */
        Mm090,              /**< ITU M-090 */
        Mp135,              /**< ITU M+135 */
        Mm135,              /**< ITU M-135 */
        Mp180,              /**< ITU M+180 */
        Up000,              /**< ITU U+000 */
        Up030,              /**< ITU U+030 */
        Um030,              /**< ITU U-030 */
        Up045,              /**< ITU U+045 */
        Um045,              /**< ITU U-030 */
        Up090,              /**< ITU U+090 */
        Um090,              /**< ITU U-090 */
        Up110,              /**< ITU U+110 */
        Um110,              /**< ITU U-110 */
        Up135,              /**< ITU U+135 */
        Um135,              /**< ITU U-135 */
        Up180,              /**< ITU U+180 */
        Tp000,              /**< ITU T+000 */
        Bp000,              /**< ITU B+000 */
        Bp045,              /**< ITU B+045 */
        Bm045               /**< ITU B-045 */
    };

    /** \enum Mode
     *  Use these values for Meter mode. Try to use the lowest possible
     *  combination that suit your needs, as performance will be better.
     */
    enum class Mode : uint_fast8_t {
        EBU_M = (1U << 0U),
        EBU_S = (1U << 1U) | EBU_M,
        EBU_I = (1U << 2U) | EBU_M,
        EBU_LRA = (1U << 3U) | EBU_M,
        SamplePeak = (1U << 4U),
        TruePeak = (1U << 5U) | SamplePeak,
        /** uses histogram algorithm to calculate global loudness */
        Histogram = (1U << 6U)
    };

    [[nodiscard]] constexpr Mode operator& (Mode x, Mode y)
    {
        using utype = std::underlying_type_t<Mode>;
        return static_cast<Mode>(static_cast<utype>(x) & static_cast<utype>(y));
    }

    [[nodiscard]] constexpr Mode operator| (Mode x, Mode y)
    {
        using utype = std::underlying_type_t<Mode>;
        return static_cast<Mode>(static_cast<utype>(x) | static_cast<utype>(y));
    }

    using DataType = std::variant<const float*, const float**, const double*, const double**, const short*, const short**, const int*, const int**>;
} // namespace loudness
#endif // DEFINES_HPP
