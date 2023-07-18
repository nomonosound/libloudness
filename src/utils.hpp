#ifndef LOUDNESS_UTILS_HPP
#define LOUDNESS_UTILS_HPP

#include <algorithm>
#include <cmath>
#include <gcem.hpp>
#include <ranges>
#include <vector>


#if defined(__SSE2_MATH__) || defined(_M_X64) || _M_IX86_FP >= 2
// TODO: Add support for arm
#include <immintrin.h>

class ScopedFTZ {
public:
    ScopedFTZ() noexcept: mxcsr_(_mm_getcsr())
    {
        _mm_setcsr(mxcsr_ | _MM_FLUSH_ZERO_ON | _MM_DENORMALS_ZERO_ON);
    }
    ~ScopedFTZ(){
        _mm_setcsr(mxcsr_);
    }
    ScopedFTZ& operator=(const ScopedFTZ&) = delete;
    ScopedFTZ& operator=(ScopedFTZ&&) = delete;
    ScopedFTZ(const ScopedFTZ&) = delete;
    ScopedFTZ(ScopedFTZ&&) = delete;
private:
    const unsigned int mxcsr_;
};

#else
class ScopedFTZ {};
#warning "manual FTZ is being used, please enable SSE2 (-msse2 -mfpmath=sse)"
#define MANUALLY_FTZ
#endif

namespace loudness {

    template <typename T>
    concept ConstData = std::is_const_v<T> || std::is_const_v<std::remove_pointer_t<T>>;

    inline double energyToLoudness(double energy) { return 10 * std::log10(energy) - 0.691; }

    consteval double loudnessToEnergy(double lufs) { return gcem::pow(10.0, (lufs + 0.691) / 10.0); }

    template <std::floating_point T>
    inline T sinc(T x)
    {
        return std::sin(x) / x;
    }

    template <std::unsigned_integral T>
    static constexpr bool safeSizeMul(T nmemb, T size, T* result)
    {
        /* Adapted from OpenBSD reallocarray. */
        // Sqrt of numeric max
        constexpr T mul_no_overflow = static_cast<T>(1) << sizeof(T) * 4;
        if ((nmemb >= mul_no_overflow || size >= mul_no_overflow) && /**/
            nmemb > 0 && std::numeric_limits<T>::max() / nmemb < size) {
            return false;
        }
        *result = nmemb * size;
        return true;
    }

    template <std::integral T>
    consteval double getScalingFactor()
    {
        return std::max<double>(-static_cast<double>(std::numeric_limits<T>::min()),
                                static_cast<double>(std::numeric_limits<T>::max()));
    }

    template <std::floating_point T>
    consteval double getScalingFactor()
    {
        return 1.0;
    }

    /* This changes the order of the range */
    inline auto medianInPlace(std::ranges::random_access_range auto&& range)
    {
        const size_t n = range.size() / 2;
        if (n == 0) [[unlikely]]{
            return -HUGE_VAL;
        }
        std::ranges::nth_element(range, range.begin() + n);
        return (range.size() % 2) ? range[n] : (range[n] + *std::max_element(range.begin(), range.begin() + n)) / 2;
    }

    inline auto median(std::ranges::range auto&& range)
    {
        using T = std::ranges::range_value_t<decltype(range)>;
        std::vector<T> copied;
        std::ranges::copy(range, std::back_inserter(copied));
        return medianInPlace(copied);
    }
} // namespace loudness
#endif  // LOUDNESS_UTILS_HPP
