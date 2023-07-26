/* See LICENSE file for copyright and license details. */

#ifndef LIBLOUDNESS_DEFINES_HPP
#define LIBLOUDNESS_DEFINES_HPP

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <variant>

namespace loudness {

    static constexpr int max_channels = 64;
    static constexpr long min_samplerate = 16;
    static constexpr long max_samplerate = 2822400;

    /* @warning this is the theoretical max, your system will run out of memory before this.
     */
    static constexpr unsigned long max_window_ms = std::numeric_limits<unsigned long>::max() / max_samplerate;

    /** \enum Channel
     *  Use these values when setting the channel map with Meter::setChannel().
     *  See definitions in ITU R-REC-BS 1770-4
     */
    enum class Channel : std::uint_fast8_t {
        Unused = 0,        /**< Unused channel (for example LFE channel) */
        Left = 1,          /**<           */
        Mp030 = 1,         /**< ITU M+030 */
        Right = 2,         /**<           */
        Mm030 = 2,         /**< ITU M-030 */
        Center = 3,        /**<           */
        Mp000 = 3,         /**< ITU M+000 */
        LeftSurround = 4,  /**<           */
        Mp110 = 4,         /**< ITU M+110 */
        RightSurround = 5, /**<           */
        Mm110 = 5,         /**< ITU M-110 */
        DualMono,          /**< a channel that is counted twice */
        MpSC,              /**< ITU M+SC  */
        MmSC,              /**< ITU M-SC  */
        Mp060,             /**< ITU M+060 */
        Mm060,             /**< ITU M-060 */
        Mp090,             /**< ITU M+090 */
        Mm090,             /**< ITU M-090 */
        Mp135,             /**< ITU M+135 */
        Mm135,             /**< ITU M-135 */
        Mp180,             /**< ITU M+180 */
        Up000,             /**< ITU U+000 */
        Up030,             /**< ITU U+030 */
        Um030,             /**< ITU U-030 */
        Up045,             /**< ITU U+045 */
        Um045,             /**< ITU U-030 */
        Up090,             /**< ITU U+090 */
        Um090,             /**< ITU U-090 */
        Up110,             /**< ITU U+110 */
        Um110,             /**< ITU U-110 */
        Up135,             /**< ITU U+135 */
        Um135,             /**< ITU U-135 */
        Up180,             /**< ITU U+180 */
        Tp000,             /**< ITU T+000 */
        Bp000,             /**< ITU B+000 */
        Bp045,             /**< ITU B+045 */
        Bm045              /**< ITU B-045 */
    };

    /** \enum Mode
     *  Use these values for Meter mode. Try to use the lowest possible
     *  combination that suit your needs, as performance will be better.
     */
    enum class Mode : std::uint_fast8_t {
        EBU_M = (1U << 0U),
        EBU_S = (1U << 1U) | EBU_M,
        EBU_I = (1U << 2U) | EBU_M,
        EBU_LRA = (1U << 3U) | EBU_M,
        SamplePeak = (1U << 4U),
        TruePeak = (1U << 5U) | SamplePeak,
        /** uses histogram algorithm to calculate global loudness */
        Histogram = (1U << 6U)
    };

    [[nodiscard]] constexpr Mode operator&(Mode left, Mode right)
    {
        using utype = std::underlying_type_t<Mode>;
        return static_cast<Mode>(static_cast<utype>(left) & static_cast<utype>(right));
    }

    [[nodiscard]] constexpr Mode operator|(Mode left, Mode right)
    {
        using utype = std::underlying_type_t<Mode>;
        return static_cast<Mode>(static_cast<utype>(left) | static_cast<utype>(right));
    }

    template <std::signed_integral T, T min_val, T max_val>
    class BoundNaturalInteger {
    public:
        static constexpr T min() { return min_val; }
        static constexpr T max() { return max_val; }
        explicit constexpr BoundNaturalInteger(T value) : value_(value)
        {
            if (value < min_val || value > max_val) {
                throw std::domain_error("Requested value not within allowed range");
            }
        }
        [[nodiscard]] constexpr auto get() const { return value_; }

    private:
        std::make_unsigned_t<T> value_;
    };

    using Samplerate = BoundNaturalInteger<long, min_samplerate, max_samplerate>;
    using NumChannels = BoundNaturalInteger<int, 1, max_channels>;

    using DataType = std::variant<const float*, const float* const*, const double*, const double* const*,
                                  const std::int16_t*, const std::int16_t* const*, const std::int32_t*, const std::int32_t* const*>;
}  // namespace loudness
#endif  // LIBLOUDNESS_DEFINES_HPP
